import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2D
from dji_serial_bridge.msg import CVTarget, FireCommand


class PointToCvTarget(Node):
    """
    Bridges the vision pipeline's 3-D target estimate to a CVTarget message
    for mcb_relay. See README.md for design rationale.

    Subscribes point_topic/confidence_topic (PointStamped/Detection2D);
    publishes output_topic (CVTarget), position + confidence only. Frame
    conversion: cv.x=-p.y, cv.y=p.z, cv.z=p.x. Resets to zero-confidence
    after target_timeout_s idle.

    Also publishes fire_topic (FireCommand) at fire_rate_hz whenever
    confidence is above fire_confidence_threshold -- a placeholder firing
    trigger (no lead/prediction, no HP/heat/power gating), standing in
    until sentry_pkg's real firing logic is written. See README.md.
    """

    def __init__(self):
        super().__init__('point_to_cv_target')

        self.declare_parameter('point_topic', 'roi_point')
        self.declare_parameter('confidence_topic', 'roi')
        self.declare_parameter('output_topic', '/cv/target')
        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('default_confidence', 1.0)
        self.declare_parameter('fire_topic', '/sentry/fire_command')
        self.declare_parameter('fire_confidence_threshold', 0.5)
        self.declare_parameter('fire_rate_hz', 2.0)

        self.point_topic = self.get_parameter('point_topic').value
        self.confidence_topic = self.get_parameter('confidence_topic').value
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

        # Matches roi_depth_node's "/roi_point" publisher (plain depth-10, reliable).
        self.point_sub = self.create_subscription(
            PointStamped, self.point_topic, self.on_point, 10)

        self.confidence_sub = self.create_subscription(
            Detection2D, self.confidence_topic, self.on_detection, 10)

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
        self.last_point_wall_time = self.get_clock().now()

        self.get_logger().info(
            f"point_to_cv_target ready\n"
            f"  {self.point_topic} (PointStamped, REP-103) + {self.confidence_topic}"
            f" (Detection2D, confidence only)\n"
            f"  -> {self.output_topic} (CVTarget, camera frame X-right Y-up Z-forward)\n"
            f"  target_timeout_s={self.target_timeout_s:.2f}\n"
            f"  -> {self.fire_topic} (FireCommand, placeholder trigger @ "
            f"{self.fire_rate_hz:.2f}Hz when confidence >= {self.fire_confidence_threshold})"
        )

    def on_detection(self, msg):
        score = 0.0
        for hyp in msg.results:
            score = max(score, hyp.hypothesis.score)
        if score > 0.0:
            self.latest_confidence = score
            self.have_confidence = True

    def on_point(self, msg):
        # Frame conversion: REP-103 (fwd, left, up) -> CVTarget (right, up, forward)
        x = -msg.point.y
        y = msg.point.z
        z = msg.point.x

        self.last_point_wall_time = self.get_clock().now()
        self.target_active = True

        out = CVTarget()
        out.header = msg.header
        out.x = float(x)
        out.y = float(y)
        out.z = float(z)
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
        idle_s = (self.get_clock().now() - self.last_point_wall_time).nanoseconds / 1e9
        if idle_s <= self.target_timeout_s:
            return

        self.target_active = False

        lost = CVTarget()
        lost.header.stamp = self.get_clock().now().to_msg()
        lost.confidence = 0.0
        self.pub.publish(lost)

        self.get_logger().info(
            f"No message on '{self.point_topic}' for {idle_s:.2f} s - published a "
            f"zero-confidence CVTarget and paused until the next point arrives."
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointToCvTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
