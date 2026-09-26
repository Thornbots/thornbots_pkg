# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Track the selected robot as a spinning 4-panel armor model.

/cv/robot_panels (target_selector's robot, winner first) -> /cv/target_state
(TargetState, odom): chassis center, velocity, acceleration, panel yaw, spin
rate, both pairs' radii and heights, from target_tracker_core.ArmorTracker.
A panel seen alone also measures yaw (it faces us). Capture time
is the detection stamp less camera_latency_s; each detection waits (up to
tf_max_wait_s) for the camera's TF at that time, and the state is predicted to
its publish time and stamped with it. See README.md's ### target_tracker.py.
"""
from collections import deque
import threading

from dji_serial_bridge.msg import PanelDetectionArray, TargetState
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException

from thornbots_pkg.target_tracker_core import (
    ACC, ArmorTracker, DZ, POS, R, ray_covariance, VEL, W, YAW,
)


def _quat_to_rot(x, y, z, w):
    """Convert a quaternion to a 3x3 rotation matrix (Hamilton convention)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class TargetTracker(Node):

    def __init__(self):
        super().__init__('target_tracker')

        self.declare_parameter('robot_panels_topic', '/cv/robot_panels')
        self.declare_parameter('output_topic', '/cv/target_state')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('pose_latency_s', 0.01)
        # Capture time = detection stamp - camera_latency_s. Unmeasured on
        # hardware (CV_SPLIT_PLAN.md, Estimation); match the emulator's in sim.
        self.declare_parameter('camera_latency_s', 0.0)
        self.declare_parameter('track_max_gap_s', 0.5)
        # How long a detection waits for the camera's TF at its capture
        # time before it is dropped. Never matched to a newer pose: the
        # error would be the wait times the head's slew rate. See README.md.
        self.declare_parameter('tf_max_wait_s', 0.25)
        self.declare_parameter('panel_radius_m', 0.27)  # initial radius, both panel pairs
        self.declare_parameter('meas_noise_base_m', 0.03)
        self.declare_parameter('meas_noise_range_coeff', 0.01)  # stddev += coeff * range_m^2
        # Across the camera ray; depth noise above applies along it only.
        self.declare_parameter('meas_noise_lateral_m', 0.04)
        self.declare_parameter('process_noise_accel', 2.0)  # m/s^2, centre
        # Horizontal acceleration: white jerk (m/s^3) on a Singer model that
        # decays over accel_time_constant_s. Kept well under the spin rate's
        # bandwidth, or it explains the spinning panel as a jinking centre.
        self.declare_parameter('process_noise_jerk', 3.0)
        self.declare_parameter('accel_time_constant_s', 1.0)
        # A panel seen alone faces the camera within about this (rad).
        self.declare_parameter('single_panel_yaw_std', 0.3)
        self.declare_parameter('process_noise_yaw_accel', 5.0)  # rad/s^2, spin rate drift
        self.declare_parameter('process_noise_radius', 0.02)  # m/sqrt(s)
        # A parked, non-spinning target: a hypothesis with v, a and w pinned
        # at 0, its centre and yaw random-walking; it hands the lead back once
        # the summed log-likelihood ratio passes still_exit_llr. See README.md.
        self.declare_parameter('still_hypothesis', True)
        self.declare_parameter('still_process_noise_pos', 0.02)  # m/sqrt(s)
        self.declare_parameter('still_process_noise_yaw', 0.05)  # rad/sqrt(s)
        self.declare_parameter('still_exit_llr', 15.0)
        # chi-square(3) innovation gate; this many outliers in a row re-seed
        # the position and keep the spin estimate. See README.md.
        self.declare_parameter('gate_nis', 16.3)
        self.declare_parameter('max_outliers', 3)

        gp = self.get_parameter
        self.robot_panels_topic = gp('robot_panels_topic').value
        self.output_topic = gp('output_topic').value
        self.odom_frame = gp('odom_frame').value
        self.pose_latency_s = float(gp('pose_latency_s').value)
        self.camera_latency_s = float(gp('camera_latency_s').value)
        self.track_max_gap_s = float(gp('track_max_gap_s').value)
        self.tf_max_wait_s = float(gp('tf_max_wait_s').value)
        self.panel_radius_m = float(gp('panel_radius_m').value)
        self.meas_noise_base_m = float(gp('meas_noise_base_m').value)
        self.meas_noise_range_coeff = float(gp('meas_noise_range_coeff').value)
        self.meas_noise_lateral_m = float(gp('meas_noise_lateral_m').value)
        self.process_noise_accel = float(gp('process_noise_accel').value)
        self.process_noise_yaw_accel = float(gp('process_noise_yaw_accel').value)
        self.process_noise_radius = float(gp('process_noise_radius').value)
        self.process_noise_jerk = float(gp('process_noise_jerk').value)
        self.accel_time_constant_s = float(gp('accel_time_constant_s').value)
        self.single_panel_yaw_std = float(gp('single_panel_yaw_std').value)
        self.still_hypothesis = bool(gp('still_hypothesis').value)
        self.still_process_noise_pos = float(gp('still_process_noise_pos').value)
        self.still_process_noise_yaw = float(gp('still_process_noise_yaw').value)
        self.still_exit_llr = float(gp('still_exit_llr').value)
        self.gate_nis = float(gp('gate_nis').value)
        self.max_outliers = int(gp('max_outliers').value)

        # /tf gets its own node and thread, so a slow tracker callback can't
        # back it up; lookups here stay non-blocking. See README.md.
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_node = rclpy.create_node(
            'target_tracker_tf', parameter_overrides=[
                self.get_parameter('use_sim_time')])
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self._tf_node)
        self._tf_executor = SingleThreadedExecutor()
        self._tf_executor.add_node(self._tf_node)
        threading.Thread(target=self._spin_tf, daemon=True).start()

        self._waiting = deque()  # (PanelDetectionArray, arrival Time) awaiting TF
        self._tf_waits = []  # seconds each processed detection waited for TF
        self._tf_drops = 0

        self.pub = self.create_publisher(TargetState, self.output_topic, 10)
        self.sub = self.create_subscription(
            PanelDetectionArray, self.robot_panels_topic, self.on_robot_panels, 10)
        # Retries on the wall clock: a bench that holds sim time until this
        # node publishes would otherwise never let a sim-time retry fire.
        self.create_timer(0.005, self._drain, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(5.0, self._log_tf_waits)

        self._track_id = None
        self._ekf = None
        self._last_stamp = None  # rclpy.time.Time of last accepted detection
        self._n_updates = 0

        self.get_logger().info(
            f'target_tracker ready\n'
            f'  {self.robot_panels_topic} -> {self.output_topic} (frame={self.odom_frame})\n'
            f'  pose_latency_s={self.pose_latency_s:.3f} '
            f'camera_latency_s={self.camera_latency_s:.3f} '
            f'track_max_gap_s={self.track_max_gap_s:.2f} '
            f'tf_max_wait_s={self.tf_max_wait_s:.2f}\n'
            f'  panel_radius_m={self.panel_radius_m:.2f} (approximation, see README.md)'
        )

    def _spin_tf(self):
        try:
            self._tf_executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass  # Ctrl-C reaches every wait set; main() reports the shutdown

    def _reset(self, track_id):
        self._track_id = track_id
        self._ekf = None
        self._last_stamp = None
        self._n_updates = 0

    def _capture_time(self, msg):
        return Time.from_msg(msg.header.stamp) - Duration(seconds=self.camera_latency_s)

    def on_robot_panels(self, msg: PanelDetectionArray):
        if msg.detections:
            self._waiting.append((msg, self.get_clock().now()))
            self._drain()

    def _drain(self):
        """Process waiting detections, oldest first, once TF covers each capture time."""
        while self._waiting:
            msg, arrival = self._waiting[0]
            waited_s = (self.get_clock().now() - arrival).nanoseconds / 1e9
            camera_frame = msg.header.frame_id or 'camera'
            query_time = self._capture_time(msg) + Duration(seconds=self.pose_latency_s)
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.odom_frame, camera_frame, query_time)
            except TransformException as ex:
                if waited_s <= self.tf_max_wait_s and self._tf_behind(camera_frame, query_time):
                    return  # TF hasn't reached the capture time yet
                self._waiting.popleft()
                self._tf_drops += 1
                self.get_logger().error(
                    f'TF {self.odom_frame}<-{camera_frame} at capture '
                    f'{query_time.nanoseconds / 1e9:.3f} unavailable after {waited_s:.3f} s '
                    f'(tf_max_wait_s={self.tf_max_wait_s:.2f}) -- dropping: {ex}',
                    throttle_duration_sec=1.0)
                continue
            self._waiting.popleft()
            self._tf_waits.append(waited_s)
            self._update(msg, tf)

    def _tf_behind(self, camera_frame, query_time):
        """Return True if the newest camera TF is older than query_time (worth waiting for)."""
        try:
            newest = self.tf_buffer.lookup_transform(self.odom_frame, camera_frame, Time())
        except TransformException:
            return True  # no TF yet at all: the tree may still be coming up
        return Time.from_msg(newest.header.stamp) < query_time

    def _log_tf_waits(self):
        if not self._tf_waits and not self._tf_drops:
            return
        waits = np.array(self._tf_waits) if self._tf_waits else np.zeros(1)
        self.get_logger().info(
            f'camera TF wait over {len(self._tf_waits)} detections: mean '
            f'{waits.mean():.3f} s, max {waits.max():.3f} s; dropped {self._tf_drops}')
        self._tf_waits = []
        self._tf_drops = 0

    def _update(self, msg, tf):
        first = msg.detections[0]
        stamp = self._capture_time(msg)

        max_gap_ns = int(self.track_max_gap_s * 1e9)
        if (self._track_id is None
                or first.robot_track_id != self._track_id
                or (self._last_stamp is not None
                    and (stamp - self._last_stamp).nanoseconds > max_gap_ns)):
            self._reset(first.robot_track_id)

        self._last_stamp = stamp

        t = tf.transform.translation
        q = tf.transform.rotation
        rot = _quat_to_rot(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        t_sec = stamp.nanoseconds / 1e9
        facing_std = self.single_panel_yaw_std if len(msg.detections) == 1 else None
        panels_odom = []
        for det in msg.detections:
            panel_cam = np.array([det.center.x, det.center.y, det.center.z])
            panel_odom = rot @ panel_cam + T
            range_m = float(np.linalg.norm(panel_cam))
            stddev = self.meas_noise_base_m + self.meas_noise_range_coeff * range_m * range_m
            panels_odom.append(panel_odom)
            R_meas = ray_covariance(panel_odom, T, stddev, self.meas_noise_lateral_m)

            if self._ekf is None:
                self._ekf = ArmorTracker(
                    panel_odom, T, t_sec, R_meas, self.panel_radius_m,
                    self.process_noise_accel, self.process_noise_yaw_accel,
                    self.process_noise_radius, q_jerk=self.process_noise_jerk,
                    accel_tau_s=self.accel_time_constant_s, still=self.still_hypothesis,
                    still_exit_llr=self.still_exit_llr,
                    q_still_pos=self.still_process_noise_pos,
                    q_still_yaw=self.still_process_noise_yaw)
            elif self._ekf.step(panel_odom, T, t_sec, R_meas, self.gate_nis,
                                self.max_outliers, facing_std) == 'reacquire':
                self.get_logger().info(
                    f'track {self._track_id}: {self.max_outliers} outliers in a row, '
                    're-seeding position (spin estimate kept)', throttle_duration_sec=1.0)
        self._n_updates += 1

        # TargetState describes the target now: predict to the publish time.
        now = max(self.get_clock().now(), stamp)
        state, P = self._ekf.predicted(now.nanoseconds / 1e9)
        out = TargetState()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.odom_frame
        out.robot_track_id = first.robot_track_id
        out.confidence = float(first.confidence)
        out.center.x, out.center.y, out.center.z = (float(v) for v in state[POS])
        out.velocity.x, out.velocity.y, out.velocity.z = (float(v) for v in state[VEL])
        out.acceleration.x, out.acceleration.y, out.acceleration.z = (
            float(v) for v in state[ACC])
        out.variance = [float(v) for v in np.concatenate(
            (np.diag(P)[POS], np.diag(P)[VEL]))]
        out.panel.x, out.panel.y, out.panel.z = (float(v) for v in panels_odom[0])
        out.yaw = float(state[YAW])
        out.yaw_rate = float(state[W])
        out.yaw_rate_variance = float(P[W, W])
        out.radius = [float(state[R]), float(self._ekf.other_r)]
        out.z_offset = [float(state[DZ]), float(-state[DZ])]  # the other pair at -dz
        # Two updates before consumers lead on it; they weigh variance
        # and yaw_rate_variance for anything finer.
        out.valid = self._n_updates >= 2

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTracker()
    rclpy.spin(node)
    node._tf_executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
