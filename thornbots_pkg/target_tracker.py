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
(TargetState, odom): chassis center, velocity, panel yaw, spin rate and
both panel radii, from target_tracker_core.ArmorTracker. Consumed by
point_to_cv_target.py. See README.md's ### target_tracker.py Notes.
"""
from dji_serial_bridge.msg import PanelDetectionArray, TargetState
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException

from thornbots_pkg.target_tracker_core import ArmorTracker, ray_covariance


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
        self.declare_parameter('track_max_gap_s', 0.5)
        # How far the TF chain may lag the detection stamp before a
        # detection is dropped rather than matched to the newest camera
        # pose. See README.md.
        self.declare_parameter('tf_future_tolerance_s', 0.25)
        self.declare_parameter('panel_radius_m', 0.27)  # initial radius, both panel pairs
        self.declare_parameter('meas_noise_base_m', 0.03)
        self.declare_parameter('meas_noise_range_coeff', 0.01)  # stddev += coeff * range_m^2
        # Across the camera ray; depth noise above applies along it only.
        self.declare_parameter('meas_noise_lateral_m', 0.04)
        self.declare_parameter('process_noise_accel', 2.0)  # m/s^2, centre
        self.declare_parameter('process_noise_yaw_accel', 5.0)  # rad/s^2, spin rate drift
        self.declare_parameter('process_noise_radius', 0.02)  # m/sqrt(s)
        # chi-square(3) innovation gate; this many outliers in a row re-seed
        # the position and keep the spin estimate. See README.md.
        self.declare_parameter('gate_nis', 16.3)
        self.declare_parameter('max_outliers', 3)

        gp = self.get_parameter
        self.robot_panels_topic = gp('robot_panels_topic').value
        self.output_topic = gp('output_topic').value
        self.odom_frame = gp('odom_frame').value
        self.pose_latency_s = float(gp('pose_latency_s').value)
        self.track_max_gap_s = float(gp('track_max_gap_s').value)
        self.tf_future_tolerance_s = float(gp('tf_future_tolerance_s').value)
        self.panel_radius_m = float(gp('panel_radius_m').value)
        self.meas_noise_base_m = float(gp('meas_noise_base_m').value)
        self.meas_noise_range_coeff = float(gp('meas_noise_range_coeff').value)
        self.meas_noise_lateral_m = float(gp('meas_noise_lateral_m').value)
        self.process_noise_accel = float(gp('process_noise_accel').value)
        self.process_noise_yaw_accel = float(gp('process_noise_yaw_accel').value)
        self.process_noise_radius = float(gp('process_noise_radius').value)
        self.gate_nis = float(gp('gate_nis').value)
        self.max_outliers = int(gp('max_outliers').value)

        self.tf_buffer = tf2_ros.Buffer()
        # /tf shares this node's executor, so every lookup below is
        # non-blocking: a timeout wait in a callback starves /tf. See README.md.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(TargetState, self.output_topic, 10)
        self.sub = self.create_subscription(
            PanelDetectionArray, self.robot_panels_topic, self.on_robot_panels, 10)

        self._track_id = None
        self._ekf = None
        self._last_stamp = None  # rclpy.time.Time of last accepted detection
        self._n_updates = 0

        self.get_logger().info(
            f'target_tracker ready\n'
            f'  {self.robot_panels_topic} -> {self.output_topic} (frame={self.odom_frame})\n'
            f'  pose_latency_s={self.pose_latency_s:.3f} '
            f'track_max_gap_s={self.track_max_gap_s:.2f}\n'
            f'  panel_radius_m={self.panel_radius_m:.2f} (approximation, see README.md)'
        )

    def _reset(self, track_id):
        self._track_id = track_id
        self._ekf = None
        self._last_stamp = None
        self._n_updates = 0

    def _lookup_camera_tf(self, camera_frame, query_time):
        """
        Look up odom<-camera at query_time, or the newest TF within tolerance.

        Returns None (logged, never silent) if TF is missing outright or
        lags further than tf_future_tolerance_s -- see README.md for why
        the fallback exists.
        """
        try:
            return self.tf_buffer.lookup_transform(
                self.odom_frame, camera_frame, query_time)
        except TransformException as ex:
            first_ex = ex

        # A detection stamp newer than the newest TF is normal when the TF
        # chain runs behind (a cold-started node on a loaded box, or sim's
        # high-rate /clock); the camera pose is then stale by that gap
        # rather than wrong. Accept it up to tf_future_tolerance_s, which
        # bounds the induced bearing error by gap x head slew rate, instead
        # of dropping every detection and publishing nothing at all.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, camera_frame, Time())
        except TransformException:
            self.get_logger().error(
                f'TF lookup {self.odom_frame}<-{camera_frame}@'
                f'{query_time.nanoseconds} failed: {first_ex}',
                throttle_duration_sec=1.0)
            return None

        gap_s = (query_time - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        if gap_s > self.tf_future_tolerance_s:
            self.get_logger().error(
                f'TF {self.odom_frame}<-{camera_frame} is {gap_s:.3f} s behind the '
                f'detection stamp (> tf_future_tolerance_s='
                f'{self.tf_future_tolerance_s:.2f}) -- dropping. The TF chain is '
                f'not keeping up: {first_ex}',
                throttle_duration_sec=1.0)
            return None

        self.get_logger().warn(
            f'TF {self.odom_frame}<-{camera_frame} is {gap_s:.3f} s behind the '
            f'detection stamp -- using the newest available camera pose.',
            throttle_duration_sec=5.0)
        return tf

    def on_robot_panels(self, msg: PanelDetectionArray):
        if not msg.detections:
            return
        first = msg.detections[0]
        stamp = Time.from_msg(msg.header.stamp)

        max_gap_ns = int(self.track_max_gap_s * 1e9)
        if (self._track_id is None
                or first.robot_track_id != self._track_id
                or (self._last_stamp is not None
                    and (stamp - self._last_stamp).nanoseconds > max_gap_ns)):
            self._reset(first.robot_track_id)

        self._last_stamp = stamp

        camera_frame = msg.header.frame_id or 'camera'
        query_time = stamp + Duration(seconds=self.pose_latency_s)
        tf = self._lookup_camera_tf(camera_frame, query_time)
        if tf is None:
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        t_sec = stamp.nanoseconds / 1e9
        panels_odom = []
        for det in msg.detections:
            panel_cam = np.array([det.center.x, det.center.y, det.center.z])
            panel_odom = R @ panel_cam + T
            range_m = float(np.linalg.norm(panel_cam))
            stddev = self.meas_noise_base_m + self.meas_noise_range_coeff * range_m * range_m
            panels_odom.append(panel_odom)
            R_meas = ray_covariance(panel_odom, T, stddev, self.meas_noise_lateral_m)

            if self._ekf is None:
                self._ekf = ArmorTracker(
                    panel_odom, T, t_sec, R_meas, self.panel_radius_m,
                    self.process_noise_accel, self.process_noise_yaw_accel,
                    self.process_noise_radius)
            elif self._ekf.step(panel_odom, T, t_sec, R_meas,
                                self.gate_nis, self.max_outliers) == 'reacquire':
                self.get_logger().info(
                    f'track {self._track_id}: {self.max_outliers} outliers in a row, '
                    're-seeding position (spin estimate kept)', throttle_duration_sec=1.0)
        self._n_updates += 1

        state, P = self._ekf.predicted(t_sec)
        out = TargetState()
        # Still the detection stamp, not the publish time TargetState.msg
        # asks for: predicting forward to it is CV_SPLIT_PLAN.md Phase 2.
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.robot_track_id = first.robot_track_id
        out.confidence = float(first.confidence)
        out.center.x, out.center.y, out.center.z = (float(v) for v in state[0:3])
        out.velocity.x, out.velocity.y, out.velocity.z = (float(v) for v in state[3:6])
        # acceleration stays 0: the EKF is constant-velocity (Phase 2).
        out.variance = [float(P[i, i]) for i in range(6)]
        out.panel.x, out.panel.y, out.panel.z = (float(v) for v in panels_odom[0])
        out.yaw = float(state[6])
        out.yaw_rate = float(state[7])
        out.yaw_rate_variance = float(P[7, 7])
        out.radius = [float(state[8]), float(self._ekf.other_r)]
        out.z_offset = [0.0, 0.0]  # one-height model until per-pair z (Phase 2)
        # Two updates before consumers lead on it; they weigh variance
        # and yaw_rate_variance for anything finer.
        out.valid = self._n_updates >= 2

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
