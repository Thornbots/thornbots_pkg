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

import math

from dji_serial_bridge.msg import CVTarget, RefSysStatus, RobotPose, TargetState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Header
import tf2_ros
from tf2_ros import TransformException

from thornbots_pkg.mcb_axes import from_mcb
from thornbots_pkg.point_to_cv_target_core import (
    LatencyStat, Patrol, patrol_point, plan_shot,
)

# RefSysStatus.delta_angle_got_hit_in when not hit (HitRing::PLACEHOLDER_ANGLE).
NOT_HIT = 123.0


def _quat_to_rot(x, y, z, w):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _rotate(R, v):
    x, y, z = v
    return (R[0][0] * x + R[0][1] * y + R[0][2] * z,
            R[1][0] * x + R[1][1] * y + R[1][2] * z,
            R[2][0] * x + R[2][1] * y + R[2][2] * z)


class PointToCvTarget(Node):
    """
    Turn target_tracker's target state into a world-frame aim point.

    /cv/target_state (armor model, odom) -> /cv/target (odom aim point,
    held by the MCB as we move, carrying its own fire decision), via
    point_to_cv_target_core.plan_shot. Each cv_target_publish_rate_hz tick
    aims, and sets fire/delay_ms (at most fire_rate_hz) with the delay that
    times a spinning target's panel to the shot. With no target it patrols:
    sweeps, or faces the last hit off RefSysStatus, never firing. See
    README.md's ### point_to_cv_target.py Notes.
    """

    def __init__(self):
        super().__init__('point_to_cv_target')

        self.declare_parameter('target_state_topic', '/cv/target_state')
        self.declare_parameter('robot_pose_topic', '/dji_serial_bridge/pose')
        self.declare_parameter('mcb_x_right', False)  # as pose_translator's
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
        self.declare_parameter('cv_target_publish_rate_hz', 40.0)
        # Spin mode (shotgating, or chase) above enter, back to panel
        # aim below exit, in |yaw_rate| rad/s.
        self.declare_parameter('spin_enter_rad_s', 3.0)
        self.declare_parameter('spin_exit_rad_s', 2.0)
        # Spin mode, >= 0 (default): chase the facing panel, firing on any
        # tick whose panel has faced us this long and will for
        # chase_margin_s more; both cover the gimbal's jump between panels.
        # < 0: shotgating, hold the center line and time the fire. README.md.
        self.declare_parameter('chase_settle_s', 0.0)
        self.declare_parameter('chase_margin_s', 0.0)
        # Sent with every aim point: whether the MCB may patrol on its own
        # (off: the Jetson owns where it looks), and turn toward a hit (on).
        self.declare_parameter('type_c_based_patrol', False)
        self.declare_parameter('turn_to_hit', True)
        # Jetson patrol with no target, mirroring the MCB's own (README.md):
        # sweep at patrol_rate_rad_s (MCB's -0.002 rad per 1 ms cycle), a
        # point patrol_range_m out and patrol_pitch_down_rad below level.
        self.declare_parameter('patrol_enabled', True)
        self.declare_parameter('patrol_after_s', 0.2)
        self.declare_parameter('patrol_rate_rad_s', -2.0)
        self.declare_parameter('patrol_range_m', 3.0)
        self.declare_parameter('patrol_pitch_down_rad', 0.05)
        # Face a hit for hit_turn_s: hit yaw = gun yaw + sign * delta angle.
        self.declare_parameter('hit_turn_s', 0.5)
        self.declare_parameter('hit_angle_sign', -1.0)
        self.declare_parameter('muzzle_frame', 'muzzle')
        self.declare_parameter('ref_sys_topic', '/dji_serial_bridge/ref_sys')
        # Bench only: a Header per publish tick, sent or not, for lockstep.
        self.declare_parameter('tick_topic', '')

        gp = self.get_parameter
        self.target_state_topic = gp('target_state_topic').value
        self.robot_pose_topic = gp('robot_pose_topic').value
        self.mcb_x_right = gp('mcb_x_right').value
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
        self.type_c_based_patrol = bool(gp('type_c_based_patrol').value)
        self.turn_to_hit = bool(gp('turn_to_hit').value)
        self.patrol = (Patrol(float(gp('patrol_rate_rad_s').value),
                              float(gp('patrol_after_s').value),
                              float(gp('hit_turn_s').value))
                       if gp('patrol_enabled').value else None)
        self.patrol_range_m = float(gp('patrol_range_m').value)
        self.patrol_pitch_down_rad = float(gp('patrol_pitch_down_rad').value)
        self.hit_angle_sign = float(gp('hit_angle_sign').value)
        self.muzzle_frame = gp('muzzle_frame').value

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
        tick_topic = gp('tick_topic').value
        self.tick_pub = (self.create_publisher(Header, tick_topic, 10)
                         if tick_topic else None)
        self.robot_pose_sub = self.create_subscription(
            RobotPose, self.robot_pose_topic, self.on_robot_pose, qos_profile_sensor_data)
        if self.patrol is not None and self.turn_to_hit:
            # Matches dji_serial_bridge_node's ~/ref_sys SensorDataQoS publisher.
            self.ref_sys_sub = self.create_subscription(
                RefSysStatus, gp('ref_sys_topic').value, self.on_ref_sys,
                qos_profile_sensor_data)

        self.watchdog_timer = self.create_timer(0.1, self.check_timeout)
        self.publish_timer = self.create_timer(1.0 / publish_rate_hz, self.on_publish_tick)

        self.last_fire_time = None
        self.spinning = False

        self.target_active = False

        self.latest_state = None  # last TargetState received
        self.chassis_vel_root = (0.0, 0.0, 0.0)  # from RobotPose, root frame
        self.latency_stat = LatencyStat()

        self.get_logger().info(
            f'point_to_cv_target ready\n'
            f'  {self.target_state_topic} + {self.robot_pose_topic}\n'
            f'  -> {self.output_topic} (CVTarget, {self.odom_frame} frame, aim + fire, '
            f'@ {publish_rate_hz:.1f}Hz)\n'
            f'  lead_enabled={self.lead_enabled} v_muzzle={self.v_muzzle} '
            f'firmware_latency_s={self.firmware_latency_s} gimbal_lag_s={self.gimbal_lag_s}\n'
            f'  target_timeout_s={self.target_timeout_s:.2f}\n'
            f'  fire <= {self.fire_rate_hz:.2f}Hz, spin-timed above '
            f'{self.spin_enter_rad_s} rad/s, confidence >= {self.fire_confidence_threshold}\n'
            f'  patrol: ' + (f'{self.patrol.rate_rad_s} rad/s, turn_to_hit={self.turn_to_hit}'
                             if self.patrol is not None else 'off')
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
        vx, vy = msg.vel_x, msg.vel_y
        if self.mcb_x_right:
            vx, vy = from_mcb(vx, vy)
        self.chassis_vel_root = (vx, vy, 0.0)

    def on_ref_sys(self, msg):
        delta = msg.delta_angle_got_hit_in
        if delta == NOT_HIT or not abs(delta) <= math.pi + 1e-3:
            return
        gun = self._gun_pose()
        if gun is None:
            return
        hit_yaw = gun[1] + self.hit_angle_sign * delta
        self.patrol.hit(hit_yaw, self.get_clock().now().nanoseconds / 1e9)
        self.get_logger().info(
            f'hit at {delta:+.2f} rad from the gun: facing odom yaw {hit_yaw:+.2f}')

    def _gun_pose(self):
        """Return the muzzle's newest odom (position, yaw), or None."""
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.muzzle_frame, Time())
        except TransformException as ex:
            self.get_logger().error(
                f'TF lookup {self.odom_frame}<-{self.muzzle_frame} failed: {ex}',
                throttle_duration_sec=1.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        return (t.x, t.y, t.z), math.atan2(R[1][0], R[0][0])

    def _patrol_point(self, now):
        gun = self._gun_pose()
        if gun is None:
            return None
        yaw = self.patrol.step(now.nanoseconds / 1e9, gun[1])
        if yaw is None:
            return None
        return patrol_point(gun[0], yaw, self.patrol_range_m, self.patrol_pitch_down_rad)

    def _fire_decision(self, delay_s, now):
        """
        Return (fire, delay_ms) for this tick's CVTarget.

        The MCB runs the delay from receiving this frame, so aim and fire
        cross the wire as one frame -- see UART_PROTOCOL.md.
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
            f'no CVTarget until the next one arrives.'
        )

    def on_publish_tick(self):
        now = self.get_clock().now()
        aim = self._compute_aim_point() if self.target_active else None
        if aim is not None and self.patrol is not None:
            self.patrol.target_seen(now.nanoseconds / 1e9)
        elif aim is None and self.patrol is not None:
            point = self._patrol_point(now)
            aim = (point, None) if point is not None else None
        if aim is not None:  # no target and no patrol: send nothing, the MCB holds still
            aim_pos, fire_delay_s = aim
            out = CVTarget()
            out.header.stamp = now.to_msg()
            out.header.frame_id = self.odom_frame
            out.x, out.y, out.z = (float(v) for v in aim_pos)
            out.fire, out.delay_ms = self._fire_decision(fire_delay_s, now)
            out.type_c_based_patrol = self.type_c_based_patrol
            out.turn_to_hit = self.turn_to_hit
            self.pub.publish(out)
        if self.tick_pub is not None:
            self.tick_pub.publish(Header(stamp=now.to_msg()))

    def _compute_aim_point(self):
        """
        Return the odom aim point, or None if none is available yet.

        Returns (aim_pos_odom, fire_delay_s or None), or None if the newest
        target_state is stale or TF fails (logged loudly, never silently) --
        caller sends no CVTarget.
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

        if not state.valid:
            # Unconverged: aim at the measured panel, no lead, no fire.
            self.spinning = False
            return (state.panel.x, state.panel.y, state.panel.z), None

        # Our pose at the state's stamp; if TF hasn't reached it yet, the
        # newest one, carried forward below.
        state_stamp = Time.from_msg(state.header.stamp)
        try:
            if self.tf_buffer.can_transform(self.odom_frame, self.root_frame, state_stamp):
                tf_shooter = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.root_frame, state_stamp)
            else:
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
        shooter_vel_odom = _rotate(shooter_R, self.chassis_vel_root)
        # plan_shot wants us at the state's stamp, not the transform's.
        tf_to_state_s = (state_stamp
                         - Time.from_msg(tf_shooter.header.stamp)).nanoseconds / 1e9
        shooter_pos_odom = (st.x + shooter_vel_odom[0] * tf_to_state_s,
                            st.y + shooter_vel_odom[1] * tf_to_state_s,
                            st.z + shooter_vel_odom[2] * tf_to_state_s)

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
        return aim_odom, fire_delay_s


def main(args=None):
    rclpy.init(args=args)
    node = PointToCvTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
