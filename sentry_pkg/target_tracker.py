"""
target_tracker.py -- WHERE IT'S GOING: consumes target_selector's per-frame
panel pick (/cv/panel_detection), estimates the tracked robot's spin-centre
position/velocity in the odom frame, and publishes
dji_serial_bridge/msg/TargetState on /cv/target_state for
point_to_cv_target.py's intercept solver. See README.md's
### target_tracker.py Notes for the spin-branch/normal-correction design
rationale and open items (width-incidence refinement is deferred, gated
behind the verification harness).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
from tf2_ros import TransformException

from dji_serial_bridge.msg import PanelDetection, TargetState

from sentry_pkg.target_tracker_core import (
    KalmanFilter6D, SpinDetector, corrected_centre,
)


def _quat_to_rot(x, y, z, w):
    """Standard quaternion->3x3 rotation matrix (Hamilton convention)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')

        self.declare_parameter('panel_topic', '/cv/panel_detection')
        self.declare_parameter('output_topic', '/cv/target_state')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('pose_latency_s', 0.01)
        self.declare_parameter('track_max_gap_s', 0.5)
        self.declare_parameter('panel_radius_m', 0.27)  # mean of panel_radius_x/y
        self.declare_parameter('spin_handoff_timeout_s', 1.5)
        self.declare_parameter('spin_min_handoffs', 3)
        self.declare_parameter('spin_cv_max', 0.35)  # coefficient of variation gate
        self.declare_parameter('spin_window_s', 0.5)  # running-mean window while spinning
        self.declare_parameter('meas_noise_base_m', 0.03)
        self.declare_parameter('meas_noise_range_coeff', 0.01)  # extra stddev per metre of range
        # 1.0 (no inflation) by default: a spin_window_s running mean of
        # ~30 samples is LESS noisy than a single raw sample, so inflating
        # R here would be backwards unless hedging against the mean
        # lagging a rotating orbit -- a real but unquantified effect, left
        # as a tunable knob rather than a default guess. See README.md.
        self.declare_parameter('spin_meas_inflation', 1.0)
        self.declare_parameter('process_noise_accel', 2.0)  # m/s^2, drives KF Q

        gp = self.get_parameter
        self.panel_topic = gp('panel_topic').value
        self.output_topic = gp('output_topic').value
        self.odom_frame = gp('odom_frame').value
        self.pose_latency_s = float(gp('pose_latency_s').value)
        self.track_max_gap_s = float(gp('track_max_gap_s').value)
        self.panel_radius_m = float(gp('panel_radius_m').value)
        self.spin_handoff_timeout_s = float(gp('spin_handoff_timeout_s').value)
        self.spin_min_handoffs = int(gp('spin_min_handoffs').value)
        self.spin_cv_max = float(gp('spin_cv_max').value)
        self.spin_window_s = float(gp('spin_window_s').value)
        self.meas_noise_base_m = float(gp('meas_noise_base_m').value)
        self.meas_noise_range_coeff = float(gp('meas_noise_range_coeff').value)
        self.spin_meas_inflation = float(gp('spin_meas_inflation').value)
        self.process_noise_accel = float(gp('process_noise_accel').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(TargetState, self.output_topic, 10)
        self.sub = self.create_subscription(
            PanelDetection, self.panel_topic, self.on_panel, 10)

        self._track_id = None
        self._kf = None
        self._last_stamp = None  # rclpy.time.Time of last accepted detection
        self._spin = SpinDetector(
            handoff_timeout_s=self.spin_handoff_timeout_s,
            min_handoffs=self.spin_min_handoffs,
            cv_max=self.spin_cv_max)
        self._window = []  # [(t_sec, x, y, z)] while spinning, for the running mean
        self._n_updates = 0

        self.get_logger().info(
            f"target_tracker ready\n"
            f"  {self.panel_topic} -> {self.output_topic} (frame={self.odom_frame})\n"
            f"  pose_latency_s={self.pose_latency_s:.3f} track_max_gap_s={self.track_max_gap_s:.2f}\n"
            f"  panel_radius_m={self.panel_radius_m:.2f} (approximation, see README.md)"
        )

    def _reset(self, track_id):
        self._track_id = track_id
        self._kf = None
        self._last_stamp = None
        self._spin.reset()
        self._window = []
        self._n_updates = 0

    def on_panel(self, msg: PanelDetection):
        stamp = Time.from_msg(msg.header.stamp)

        max_gap_ns = int(self.track_max_gap_s * 1e9)
        if (self._track_id is None
                or msg.robot_track_id != self._track_id
                or (self._last_stamp is not None
                    and (stamp - self._last_stamp).nanoseconds > max_gap_ns)):
            self._reset(msg.robot_track_id)

        self._last_stamp = stamp

        camera_frame = msg.header.frame_id or 'camera'
        query_time = stamp + Duration(seconds=self.pose_latency_s)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, camera_frame, query_time,
                timeout=Duration(seconds=0.05))
        except TransformException as ex:
            # Loud, not silent. A missing camera frame in TF must surface
            # as an error, not a stale or
            # phantom TargetState publish.
            self.get_logger().error(
                f"TF lookup {self.odom_frame}<-{camera_frame}@{query_time.nanoseconds}"
                f" failed: {ex}", throttle_duration_sec=1.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        panel_cam = np.array([msg.center.x, msg.center.y, msg.center.z])
        panel_odom = R @ panel_cam + T

        range_m = float(np.linalg.norm(panel_cam))
        centre_cam = corrected_centre(panel_cam, self.panel_radius_m)
        centre_odom = R @ centre_cam + T

        t_sec = stamp.nanoseconds / 1e9
        spinning, spin_hz, spin_phase = self._spin.update(t_sec, msg.class_id)

        estimator = 0  # running_mean -- width-refined (1) deferred, see module docstring
        if spinning:
            self._window.append((t_sec, *centre_odom))
            self._window = [w for w in self._window if t_sec - w[0] <= self.spin_window_s]
            xs = np.array([w[1:] for w in self._window])
            meas = xs.mean(axis=0)
            meas_inflation = self.spin_meas_inflation
        else:
            self._window = []
            meas = centre_odom
            meas_inflation = 1.0

        base_stddev = self.meas_noise_base_m + self.meas_noise_range_coeff * range_m * range_m
        pos_var = (base_stddev * meas_inflation) ** 2

        if self._kf is None:
            self._kf = KalmanFilter6D(meas, t_sec, pos_var)
        else:
            self._kf.predict(t_sec, self.process_noise_accel)
            self._kf.update(meas, pos_var)
        self._n_updates += 1

        out = TargetState()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.robot_track_id = msg.robot_track_id
        cx, cy, cz, vx, vy, vz = self._kf.state
        out.centre.x, out.centre.y, out.centre.z = float(cx), float(cy), float(cz)
        out.velocity.x, out.velocity.y, out.velocity.z = float(vx), float(vy), float(vz)
        out.variance = [float(v) for v in self._kf.variance]
        out.panel.x, out.panel.y, out.panel.z = panel_odom.tolist()
        out.spin_hz = float(spin_hz)
        out.spin_phase = float(spin_phase)
        out.estimator = estimator
        # Two updates minimum for a meaningful velocity estimate; don't wait
        # for spin-period convergence (a real engagement may be shorter) --
        # the consumer weighs the KF variance instead.
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
