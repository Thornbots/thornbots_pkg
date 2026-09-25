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

from dji_serial_bridge.msg import CVTarget, RobotPose, TargetState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException

from thornbots_pkg.point_to_cv_target_core import LatencyStat, plan_shot


def _quat_to_rot(x, y, z, w):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _apply(R, T, p):
    x, y, z = p
    return (R[0][0] * x + R[0][1] * y + R[0][2] * z + T[0],
            R[1][0] * x + R[1][1] * y + R[1][2] * z + T[1],
            R[2][0] * x + R[2][1] * y + R[2][2] * z + T[2])


def _rotate(R, v):
    x, y, z = v
    return (R[0][0] * x + R[0][1] * y + R[0][2] * z,
            R[1][0] * x + R[1][1] * y + R[1][2] * z,
            R[2][0] * x + R[2][1] * y + R[2][2] * z)


class PointToCvTarget(Node):
    """
    Turn target_tracker's target state into a root-frame aim point.

    /cv/target_state (armor model, odom) -> /cv/target (root-frame aim
    point carrying its own fire decision), via
    point_to_cv_target_core.plan_shot. Each cv_target_publish_rate_hz tick
    aims, and sets fire/delay_ms (at most fire_rate_hz) with the delay that
    times a spinning target's panel to the shot. Liveness, confidence and
    track id come off TargetState too; it subscribes nothing else but
    RobotPose. See README.md's ### point_to_cv_target.py Notes.
    """

    def __init__(self):
        super().__init__('point_to_cv_target')

        self.declare_parameter('target_state_topic', '/cv/target_state')
        self.declare_parameter('robot_pose_topic', '/pose')
        self.declare_parameter('output_topic', '/cv/target')
        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('fire_confidence_threshold', 0.5)
        self.declare_parameter('fire_rate_hz', 2.0)
        self.declare_parameter('root_frame', 'root')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('lead_enabled', True)
        # Fire decision to projectile exit; times the fire against the spin.
        self.declare_parameter('firmware_latency_s', 0.05)
        # Setpoint to gimbal pointing there, on a moving setpoint; sets how
        # far ahead the aim point leads. See README.md.
        self.declare_parameter('gimbal_lag_s', 0.05)
        self.declare_parameter('v_muzzle', 25.0)
        self.declare_parameter('tof_iterations', 3)
        self.declare_parameter('cv_target_publish_rate_hz', 30.0)
        # Spin mode (center aim + timed fire) above enter, back to panel
        # aim below exit, in |yaw_rate| rad/s.
        self.declare_parameter('spin_enter_rad_s', 3.0)
        self.declare_parameter('spin_exit_rad_s', 2.0)
        # Spin mode aims at the center line and times the fire (< 0), or,
        # when >= 0, chases the facing panel and fires on any tick whose
        # panel has faced us this long and will for chase_margin_s more.
        # Both cover the gimbal's jump between panels. See README.md.
        self.declare_parameter('chase_settle_s', -1.0)
        self.declare_parameter('chase_margin_s', 0.0)

        gp = self.get_parameter
        self.target_state_topic = gp('target_state_topic').value
        self.robot_pose_topic = gp('robot_pose_topic').value
        self.output_topic = gp('output_topic').value
        self.target_timeout_s = float(gp('target_timeout_s').value)
        self.fire_confidence_threshold = float(gp('fire_confidence_threshold').value)
        self.fire_rate_hz = float(gp('fire_rate_hz').value)
        self.root_frame = gp('root_frame').value
        self.odom_frame = gp('odom_frame').value
        self.lead_enabled = bool(gp('lead_enabled').value)
        self.firmware_latency_s = float(gp('firmware_latency_s').value)
        self.gimbal_lag_s = float(gp('gimbal_lag_s').value)
        self.v_muzzle = float(gp('v_muzzle').value)
        self.tof_iterations = int(gp('tof_iterations').value)
        publish_rate_hz = float(gp('cv_target_publish_rate_hz').value)
        self.tick_s = 1.0 / publish_rate_hz
        self.spin_enter_rad_s = float(gp('spin_enter_rad_s').value)
        self.spin_exit_rad_s = float(gp('spin_exit_rad_s').value)
        chase_settle_s = float(gp('chase_settle_s').value)
        self.chase_settle_s = chase_settle_s if chase_settle_s >= 0.0 else None
        self.chase_margin_s = float(gp('chase_margin_s').value)

        self.tf_buffer = tf2_ros.Buffer()
        # /tf shares this node's executor, so every lookup below is
        # non-blocking: a timeout wait in a callback starves /tf. See README.md.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sensor-like, best-effort traffic: a dropped target update is far
        # less harmful than blocking on a slow/disconnected subscriber, and
        # this matches mcb_relay's cv_target subscriber QoS.
        self.pub = self.create_publisher(
            CVTarget, self.output_topic, qos_profile_sensor_data)
        self.target_state_sub = self.create_subscription(
            TargetState, self.target_state_topic, self.on_target_state, 10)
        self.robot_pose_sub = self.create_subscription(
            RobotPose, self.robot_pose_topic, self.on_robot_pose, qos_profile_sensor_data)

        self.watchdog_timer = self.create_timer(0.1, self.check_timeout)
        self.publish_timer = self.create_timer(1.0 / publish_rate_hz, self.on_publish_tick)

        self.last_fire_time = None
        self.spinning = False

        self.target_active = False

        self.latest_state = None  # last TargetState received
        self.chassis_vel_root = (0.0, 0.0, 0.0)  # from RobotPose, root-frame
        self.latency_stat = LatencyStat()

        self.get_logger().info(
            f'point_to_cv_target ready\n'
            f'  {self.target_state_topic} + {self.robot_pose_topic}\n'
            f'  -> {self.output_topic} (CVTarget, ROOT frame, aim + fire, '
            f'@ {publish_rate_hz:.1f}Hz)\n'
            f'  lead_enabled={self.lead_enabled} v_muzzle={self.v_muzzle} '
            f'firmware_latency_s={self.firmware_latency_s} gimbal_lag_s={self.gimbal_lag_s}\n'
            f'  target_timeout_s={self.target_timeout_s:.2f}\n'
            f'  fire <= {self.fire_rate_hz:.2f}Hz, spin-timed above '
            f'{self.spin_enter_rad_s} rad/s, confidence >= {self.fire_confidence_threshold}'
        )

    def on_target_state(self, msg):
        if self.latest_state is None or msg.robot_track_id != self.latest_state.robot_track_id:
            self.get_logger().info(f'target_state: tracking robot {msg.robot_track_id}')
        self.latest_state = msg
        self.target_active = True
        now = self.get_clock().now()
        latency_s = (now - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if latency_s >= 0.0:
            self.latency_stat.add(latency_s)
            # Diagnostic only -- the lead solve uses each tick's own state
            # age instead, which is larger and varies. See README.md.
            self.get_logger().info(
                f'target_state age on arrival: {latency_s * 1e3:.1f} ms now, '
                f'{self.latency_stat.mean * 1e3:.1f} ms mean over '
                f'{self.latency_stat.count} samples',
                throttle_duration_sec=10.0)

    def on_robot_pose(self, msg):
        self.chassis_vel_root = (msg.vel_x, msg.vel_y, 0.0)

    def _fire_decision(self, delay_s, now):
        """
        Return (fire, delay_ms) for this tick's CVTarget.

        The delay is measured from the message's own header.stamp, so aim
        and fire cross the wire as one frame -- see UART_PROTOCOL.md.
        Rate-limited to fire_rate_hz; delay_ms is clamped to the uint16 field.
        """
        if delay_s is None or self.fire_rate_hz <= 0.0:
            return False, 0
        if self.latest_state.confidence < self.fire_confidence_threshold:
            return False, 0
        if (self.last_fire_time is not None
                and (now - self.last_fire_time).nanoseconds / 1e9 < 1.0 / self.fire_rate_hz):
            return False, 0
        self.last_fire_time = now
        return True, max(0, min(65535, int(round(delay_s * 1000.0))))

    def check_timeout(self):
        if not self.target_active:
            return
        stamp = Time.from_msg(self.latest_state.header.stamp)
        age_s = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age_s <= self.target_timeout_s:
            return

        self.target_active = False
        self.get_logger().info(
            f"Newest '{self.target_state_topic}' is {age_s:.2f} s old - publishing "
            f'zero-confidence CVTarget until the next one arrives.'
        )

    def on_publish_tick(self):
        now = self.get_clock().now()
        out = CVTarget()
        out.header.stamp = now.to_msg()

        if not self.target_active:
            self.pub.publish(out)  # all-zero: confidence=0, no fire
            return

        aim_root = self._compute_aim_point()
        if aim_root is None:
            self.pub.publish(out)  # still all-zero
            return
        aim_pos, lead_applied, track_valid, fire_delay_s = aim_root

        out.x, out.y, out.z = (float(v) for v in aim_pos)
        out.confidence = float(self.latest_state.confidence)
        out.lead_applied = lead_applied
        out.track_valid = track_valid
        out.fire, out.delay_ms = self._fire_decision(fire_delay_s, now)
        self.pub.publish(out)

    def _compute_aim_point(self):
        """
        Return the root-frame aim point, or None if none is available yet.

        Returns (aim_pos_root, lead_applied, track_valid, fire_delay_s or
        None), or None if the newest target_state is stale or TF fails
        (logged loudly, never silently) -- caller emits zero-confidence.
        """
        state = self.latest_state
        now = self.get_clock().now()
        state_age_s = (now - Time.from_msg(state.header.stamp)).nanoseconds / 1e9

        if state_age_s > self.target_timeout_s:
            self.get_logger().warn(
                f"Newest '{self.target_state_topic}' is {state_age_s:.2f} s old "
                f'(> target_timeout_s={self.target_timeout_s:.2f}) -- not aiming on it.',
                throttle_duration_sec=1.0)
            return None

        try:
            tf = self.tf_buffer.lookup_transform(
                self.root_frame, self.odom_frame, Time())
        except TransformException as ex:
            self.get_logger().error(
                f'TF lookup {self.root_frame}<-{self.odom_frame} failed: {ex}',
                throttle_duration_sec=1.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        T = (t.x, t.y, t.z)

        if not state.valid:
            # Unconverged: aim at the measured panel, no lead, no fire.
            self.spinning = False
            panel_odom = (state.panel.x, state.panel.y, state.panel.z)
            return _apply(R, T, panel_odom), False, False, None

        try:
            tf_shooter = self.tf_buffer.lookup_transform(
                self.odom_frame, self.root_frame, Time())
        except TransformException as ex:
            self.get_logger().error(
                f'TF lookup {self.odom_frame}<-{self.root_frame} failed: {ex}',
                throttle_duration_sec=1.0)
            return None

        st = tf_shooter.transform.translation
        sq = tf_shooter.transform.rotation
        shooter_R = _quat_to_rot(sq.x, sq.y, sq.z, sq.w)
        shooter_pos_odom = (st.x, st.y, st.z)
        shooter_vel_odom = _rotate(shooter_R, self.chassis_vel_root)

        threshold = self.spin_exit_rad_s if self.spinning else self.spin_enter_rad_s
        self.spinning = abs(state.yaw_rate) > threshold
        armor = (state.center.x, state.center.y, state.center.z,
                 state.velocity.x, state.velocity.y, state.velocity.z,
                 state.yaw, state.yaw_rate)
        # Age of THIS state at THIS tick, not latency_stat.mean: publishing
        # runs on its own timer over a cached state. See README.md.
        aim_odom, fire_delay_s = plan_shot(
            armor, tuple(state.radius), tuple(state.z_offset), state_age_s,
            shooter_pos_odom, self.v_muzzle, self.spinning, self.tick_s,
            gimbal_lag_s=self.gimbal_lag_s, firmware_latency_s=self.firmware_latency_s,
            lead=self.lead_enabled, iterations=self.tof_iterations,
            shooter_vel=shooter_vel_odom, chase_settle_s=self.chase_settle_s,
            chase_margin_s=self.chase_margin_s,
            accel=(state.acceleration.x, state.acceleration.y, state.acceleration.z))

        return _apply(R, T, aim_odom), self.lead_enabled, True, fire_delay_s


def main(args=None):
    rclpy.init(args=args)
    node = PointToCvTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
