import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PolygonStamped
from dji_serial_bridge.msg import CVTarget, FireCommand, PanelDetection


class PointToCvTarget(Node):
    """
    Bridges the vision pipeline's bbox+depth panel detection to a CVTarget
    message for mcb_relay, and republishes the 3-D panel polygon for
    visualization/future firing logic. See README.md for design rationale.

    Subscribes panel_topic (PanelDetection: 4 corners + center + depth +
    confidence, camera REP-103 frame). Publishes:
    - polygon_topic (PolygonStamped) -- the panel corners, unmodified, for
      rviz/foxglove visualization and future geometric firing logic.
    - output_topic (CVTarget) -- center + confidence only, frame conversion
      cv.x=-p.y, cv.y=p.z, cv.z=p.x. Resets to zero-confidence after
      target_timeout_s idle.
    - fire_topic (FireCommand) at fire_rate_hz whenever confidence is above
      fire_confidence_threshold -- a placeholder firing trigger (no
      lead/prediction, no HP/heat/power gating, no use of the polygon
      shape), standing in until sentry_pkg's real firing logic is written.
      See README.md.
    """

    def __init__(self):
        super().__init__('point_to_cv_target')

        self.declare_parameter('panel_topic', '/cv/panel_detection')
        self.declare_parameter('polygon_topic', '/cv/panel_polygon')
        self.declare_parameter('output_topic', '/cv/target')
        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('default_confidence', 1.0)
        self.declare_parameter('fire_topic', '/sentry/fire_command')
        self.declare_parameter('fire_confidence_threshold', 0.5)
        self.declare_parameter('fire_rate_hz', 2.0)

        self.panel_topic = self.get_parameter('panel_topic').value
        self.polygon_topic = self.get_parameter('polygon_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_timeout_s = self.get_parameter('target_timeout_s').value
        self.default_confidence = self.get_parameter('default_confidence').value
        self.fire_topic = self.get_parameter('fire_topic').value
        self.fire_confidence_threshold = self.get_parameter('fire_confidence_threshold').value
        self.fire_rate_hz = self.get_parameter('fire_rate_hz').value

        # Sensor-like, best-effort traffic: a dropped target update is far
        # less harmful than blocking on a slow/disconnected subscriber, and
        # this matches mcb_relay's cv_target subscriber QoS.
        self.pub = self.create_publisher(
            CVTarget, self.output_topic, qos_profile_sensor_data)

        self.polygon_pub = self.create_publisher(
            PolygonStamped, self.polygon_topic, 10)

        # Matches roi_depth_node's "/cv/panel_detection" publisher (plain depth-10, reliable).
        self.panel_sub = self.create_subscription(
            PanelDetection, self.panel_topic, self.on_panel, 10)

        self.watchdog_timer = self.create_timer(0.1, self.check_timeout)

        # Placeholder firing trigger -- see class docstring.
        self.fire_pub = self.create_publisher(FireCommand, self.fire_topic, 10)
        if self.fire_rate_hz > 0.0:
            self.fire_timer = self.create_timer(1.0 / self.fire_rate_hz, self.maybe_fire)

        # Confidence cache
        self.latest_confidence = 1.0
        self.have_confidence = False

        # Stale-target watchdog
        self.target_active = False
        self.last_panel_wall_time = self.get_clock().now()

        self.get_logger().info(
            f"point_to_cv_target ready\n"
            f"  {self.panel_topic} (PanelDetection: corners+center+depth+confidence)\n"
            f"  -> {self.polygon_topic} (PolygonStamped, panel corners, camera REP-103 frame)\n"
            f"  -> {self.output_topic} (CVTarget, camera frame X-right Y-up Z-forward)\n"
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

        # Frame conversion: REP-103 (fwd, left, up) -> CVTarget (right, up, forward)
        out = CVTarget()
        out.header = msg.header
        out.x = -float(msg.center.y)
        out.y = float(msg.center.z)
        out.z = float(msg.center.x)
        out.confidence = float(
            self.latest_confidence if self.have_confidence else self.default_confidence)

        self.pub.publish(out)

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

        lost = CVTarget()
        lost.header.stamp = self.get_clock().now().to_msg()
        lost.confidence = 0.0
        self.pub.publish(lost)

        self.get_logger().info(
            f"No message on '{self.panel_topic}' for {idle_s:.2f} s - published a "
            f"zero-confidence CVTarget and paused until the next detection arrives."
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointToCvTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
