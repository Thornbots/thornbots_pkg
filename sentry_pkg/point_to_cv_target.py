import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PolygonStamped
import tf2_ros
from tf2_ros import TransformException
from dji_serial_bridge.msg import CVTarget, FireCommand, PanelDetection, RobotPose, TargetState

from sentry_pkg.point_to_cv_target_core import LatencyStat, solve_intercept


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
    WHERE TO AIM: turns target_tracker's /cv/target_state (odom-frame
    spin-centre KF estimate) into a root-frame /cv/target aim point for
    mcb_relay, applying latency + time-of-flight lead. Confidence
    and liveness still come from panel_topic directly (target_state carries
    no confidence field) -- see README.md's ### point_to_cv_target.py Notes.

    Publishes at cv_target_publish_rate_hz, decoupled from the ~60Hz
    detection rate -- Type-C's PID doesn't need 60Hz setpoints.
    """

    def __init__(self):
        super().__init__('point_to_cv_target')

        self.declare_parameter('panel_topic', '/cv/panel_detection')
        self.declare_parameter('target_state_topic', '/cv/target_state')
        self.declare_parameter('robot_pose_topic', '/pose')
        self.declare_parameter('polygon_topic', '/cv/panel_polygon')
        self.declare_parameter('output_topic', '/cv/target')
        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('default_confidence', 1.0)
        self.declare_parameter('fire_topic', '/sentry/fire_command')
        self.declare_parameter('fire_confidence_threshold', 0.5)
        self.declare_parameter('fire_rate_hz', 2.0)
        self.declare_parameter('root_frame', 'root')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('lead_enabled', False)
        self.declare_parameter('firmware_latency_s', 0.0)
        self.declare_parameter('v_muzzle', 25.0)
        self.declare_parameter('tof_iterations', 3)
        self.declare_parameter('cv_target_publish_rate_hz', 30.0)

        gp = self.get_parameter
        self.panel_topic = gp('panel_topic').value
        self.target_state_topic = gp('target_state_topic').value
        self.robot_pose_topic = gp('robot_pose_topic').value
        self.polygon_topic = gp('polygon_topic').value
        self.output_topic = gp('output_topic').value
        self.target_timeout_s = float(gp('target_timeout_s').value)
        self.default_confidence = float(gp('default_confidence').value)
        self.fire_topic = gp('fire_topic').value
        self.fire_confidence_threshold = float(gp('fire_confidence_threshold').value)
        self.fire_rate_hz = float(gp('fire_rate_hz').value)
        self.root_frame = gp('root_frame').value
        self.odom_frame = gp('odom_frame').value
        self.lead_enabled = bool(gp('lead_enabled').value)
        self.firmware_latency_s = float(gp('firmware_latency_s').value)
        self.v_muzzle = float(gp('v_muzzle').value)
        self.tof_iterations = int(gp('tof_iterations').value)
        publish_rate_hz = float(gp('cv_target_publish_rate_hz').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sensor-like, best-effort traffic: a dropped target update is far
        # less harmful than blocking on a slow/disconnected subscriber, and
        # this matches mcb_relay's cv_target subscriber QoS.
        self.pub = self.create_publisher(
            CVTarget, self.output_topic, qos_profile_sensor_data)
        self.polygon_pub = self.create_publisher(
            PolygonStamped, self.polygon_topic, 10)

        self.panel_sub = self.create_subscription(
            PanelDetection, self.panel_topic, self.on_panel, 10)
        self.target_state_sub = self.create_subscription(
            TargetState, self.target_state_topic, self.on_target_state, 10)
        self.robot_pose_sub = self.create_subscription(
            RobotPose, self.robot_pose_topic, self.on_robot_pose, qos_profile_sensor_data)

        self.watchdog_timer = self.create_timer(0.1, self.check_timeout)
        self.publish_timer = self.create_timer(1.0 / publish_rate_hz, self.on_publish_tick)

        self.fire_pub = self.create_publisher(FireCommand, self.fire_topic, 10)
        if self.fire_rate_hz > 0.0:
            self.fire_timer = self.create_timer(1.0 / self.fire_rate_hz, self.maybe_fire)

        self.latest_confidence = 1.0
        self.have_confidence = False
        self.target_active = False
        self.last_panel_wall_time = self.get_clock().now()

        self.latest_state = None  # last TargetState received
        self.chassis_vel_root = (0.0, 0.0, 0.0)  # from RobotPose, root-frame
        self.latency_stat = LatencyStat()

        self.get_logger().info(
            f"point_to_cv_target ready\n"
            f"  {self.panel_topic} (confidence/liveness) + {self.target_state_topic} (position)\n"
            f"  -> {self.output_topic} (CVTarget, ROOT frame, @ {publish_rate_hz:.1f}Hz)\n"
            f"  lead_enabled={self.lead_enabled} v_muzzle={self.v_muzzle} "
            f"firmware_latency_s={self.firmware_latency_s}\n"
            f"  target_timeout_s={self.target_timeout_s:.2f}\n"
            f"  -> {self.fire_topic} (FireCommand, placeholder trigger @ "
            f"{self.fire_rate_hz:.2f}Hz when confidence >= {self.fire_confidence_threshold})"
        )

    def on_panel(self, msg):
        if msg.confidence > 0.0:
            self.latest_confidence = msg.confidence
            self.have_confidence = True

        self.last_panel_wall_time = self.get_clock().now()
        self.target_active = True

        polygon = PolygonStamped()
        polygon.header = msg.header
        polygon.polygon.points = list(msg.corners)
        self.polygon_pub.publish(polygon)

    def on_target_state(self, msg):
        self.latest_state = msg
        now = self.get_clock().now()
        detection_time = Time.from_msg(msg.header.stamp)
        latency_s = (now - detection_time).nanoseconds / 1e9
        if latency_s >= 0.0:
            self.latency_stat.add(latency_s)

    def on_robot_pose(self, msg):
        self.chassis_vel_root = (msg.vel_x, msg.vel_y, 0.0)

    def maybe_fire(self):
        if not self.target_active:
            return
        confidence = self.latest_confidence if self.have_confidence else self.default_confidence
        if confidence < self.fire_confidence_threshold:
            return

        cmd = FireCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.fire = True
        cmd.delay_ms = 0
        self.fire_pub.publish(cmd)

    def check_timeout(self):
        if not self.target_active:
            return
        idle_s = (self.get_clock().now() - self.last_panel_wall_time).nanoseconds / 1e9
        if idle_s <= self.target_timeout_s:
            return

        self.target_active = False
        self.get_logger().info(
            f"No message on '{self.panel_topic}' for {idle_s:.2f} s - publishing "
            f"zero-confidence CVTarget and pausing until the next detection arrives."
        )

    def on_publish_tick(self):
        out = CVTarget()
        out.header.stamp = self.get_clock().now().to_msg()

        if not self.target_active:
            self.pub.publish(out)  # all-zero: confidence=0, flags clear
            return

        aim_root = self._compute_aim_point()
        if aim_root is None:
            self.pub.publish(out)  # still all-zero
            return
        aim_pos, lead_applied, track_valid = aim_root

        out.x, out.y, out.z = (float(v) for v in aim_pos)
        out.confidence = float(
            self.latest_confidence if self.have_confidence else self.default_confidence)
        out.lead_applied = lead_applied
        out.track_valid = track_valid
        self.pub.publish(out)

    def _compute_aim_point(self):
        """Returns (aim_pos_root, lead_applied, track_valid) or None if no
        usable position exists yet (no target_state received) or TF fails
        (logged loudly, never silently) -- caller emits zero-confidence."""
        state = self.latest_state
        if state is None:
            self.get_logger().warn(
                f"No message on '{self.target_state_topic}' yet -- "
                "point_to_cv_target has confidence but no position to emit.",
                throttle_duration_sec=5.0)
            return None

        try:
            tf = self.tf_buffer.lookup_transform(
                self.root_frame, self.odom_frame, Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as ex:
            self.get_logger().error(
                f"TF lookup {self.root_frame}<-{self.odom_frame} failed: {ex}",
                throttle_duration_sec=1.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        T = (t.x, t.y, t.z)

        if not state.valid:
            # Never emit an unconverged extrapolation -- raw panel position,
            # no lead.
            panel_odom = (state.panel.x, state.panel.y, state.panel.z)
            return _apply(R, T, panel_odom), False, False

        centre_odom = (state.centre.x, state.centre.y, state.centre.z)
        if not self.lead_enabled:
            return _apply(R, T, centre_odom), False, True

        vel_odom = (state.velocity.x, state.velocity.y, state.velocity.z)

        try:
            tf_shooter = self.tf_buffer.lookup_transform(
                self.odom_frame, self.root_frame, Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as ex:
            self.get_logger().error(
                f"TF lookup {self.odom_frame}<-{self.root_frame} failed: {ex}",
                throttle_duration_sec=1.0)
            return None

        st = tf_shooter.transform.translation
        sq = tf_shooter.transform.rotation
        shooter_R = _quat_to_rot(sq.x, sq.y, sq.z, sq.w)
        shooter_pos_odom = (st.x, st.y, st.z)
        shooter_vel_odom = _rotate(shooter_R, self.chassis_vel_root)

        tau = self.latency_stat.mean + self.firmware_latency_s
        aim_odom, _t_flight = solve_intercept(
            centre_odom, vel_odom, shooter_pos_odom, tau, self.v_muzzle,
            iterations=self.tof_iterations, shooter_vel=shooter_vel_odom)

        return _apply(R, T, aim_odom), True, True


def main(args=None):
    rclpy.init(args=args)
    node = PointToCvTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
