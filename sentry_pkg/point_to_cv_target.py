import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2D
from dji_serial_bridge.msg import CVTarget


class PointToCvTarget(Node):
    """
    Bridges the vision pipeline's 3-D target estimate to a CVTarget message
    for mcb_relay -- this is the missing link between roi_depth_query/
    roi_depth_node and mcb_relay's cv_target input, since only sentry_pkg is
    allowed to publish the messages that eventually reach dji_serial_bridge.

    Subscribes:
      point_topic       (geometry_msgs/PointStamped) -- REP-103 camera body
                         frame (X forward, Y left, Z up). Default "roi_point",
                         published by roi_depth_query/roi_depth_node.
      confidence_topic   (vision_msgs/Detection2D, optional) -- read only for
                         a confidence score (max of all hypothesis scores,
                         same rule detection_picker_node uses). Default "roi".

    Publishes:
      output_topic       (dji_serial_bridge/msg/CVTarget) -- camera-frame
                         convention (X right, Y up, Z forward), see
                         CVTarget.msg. Default "/cv/target", matching
                         mcb_relay's cv_target_input_topic default.

    Frame conversion (REP-103 -> CVTarget convention):
      cv.x =  -p.y   (right    = -left)
      cv.y =   p.z   (up       =  up)
      cv.z =   p.x   (forward  =  forward)

    Velocity / acceleration:
      roi_depth_node only publishes position. When estimate_velocity is true
      (default), v_x/v_y/v_z and a_x/a_y/a_z are estimated by finite-
      differencing consecutive PointStamped samples (using the message
      timestamps, not wall-clock arrival time) and smoothed with a simple
      exponential moving average (velocity_filter_alpha). This is a coarse
      estimate, not a proper tracker/filter -- if you already have a tracked
      velocity upstream, publish it separately and set
      estimate_velocity:=false to leave those fields at zero.

    Stale-target watchdog:
      If no new point arrives for target_timeout_s, a single zero-confidence
      CVTarget is published (so the MCB/gimbal can stop tracking a ghost
      target) and the velocity filter resets; publishing resumes cleanly on
      the next fresh point.
    """

    def __init__(self):
        super().__init__('point_to_cv_target')

        self.declare_parameter('point_topic', 'roi_point')
        self.declare_parameter('confidence_topic', 'roi')
        self.declare_parameter('output_topic', '/cv/target')
        self.declare_parameter('estimate_velocity', True)
        self.declare_parameter('velocity_filter_alpha', 0.4)
        self.declare_parameter('max_extrapolation_gap_s', 0.5)
        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('default_confidence', 1.0)

        self.point_topic = self.get_parameter('point_topic').value
        self.confidence_topic = self.get_parameter('confidence_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.estimate_velocity = self.get_parameter('estimate_velocity').value
        self.velocity_filter_alpha = min(
            1.0, max(0.0, self.get_parameter('velocity_filter_alpha').value))
        self.max_gap_s = self.get_parameter('max_extrapolation_gap_s').value
        self.target_timeout_s = self.get_parameter('target_timeout_s').value
        self.default_confidence = self.get_parameter('default_confidence').value

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

        # Confidence cache
        self.latest_confidence = 1.0
        self.have_confidence = False

        # Finite-difference filter state
        self.have_prev = False
        self.prev_x = self.prev_y = self.prev_z = 0.0
        self.prev_vx = self.prev_vy = self.prev_vz = 0.0
        self.prev_ax = self.prev_ay = self.prev_az = 0.0
        self.prev_t = 0.0

        # Stale-target watchdog
        self.target_active = False
        self.last_point_wall_time = self.get_clock().now()

        self.get_logger().info(
            f"point_to_cv_target ready\n"
            f"  {self.point_topic} (PointStamped, REP-103) + {self.confidence_topic}"
            f" (Detection2D, confidence only)\n"
            f"  -> {self.output_topic} (CVTarget, camera frame X-right Y-up Z-forward)\n"
            f"  estimate_velocity={self.estimate_velocity}"
            f"  target_timeout_s={self.target_timeout_s:.2f}"
        )

    def on_detection(self, msg):
        score = 0.0
        for hyp in msg.results:
            score = max(score, hyp.hypothesis.score)
        if score > 0.0:
            self.latest_confidence = score
            self.have_confidence = True

    def on_point(self, msg):
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds / 1e9

        # Frame conversion: REP-103 (fwd, left, up) -> CVTarget (right, up, forward)
        x = -msg.point.y
        y = msg.point.z
        z = msg.point.x

        vx = vy = vz = 0.0
        ax = ay = az = 0.0

        if self.estimate_velocity:
            dt = t - self.prev_t
            if self.have_prev and 0.0 < dt <= self.max_gap_s:
                rvx = (x - self.prev_x) / dt
                rvy = (y - self.prev_y) / dt
                rvz = (z - self.prev_z) / dt

                a = self.velocity_filter_alpha
                vx = a * rvx + (1.0 - a) * self.prev_vx
                vy = a * rvy + (1.0 - a) * self.prev_vy
                vz = a * rvz + (1.0 - a) * self.prev_vz

                rax = (vx - self.prev_vx) / dt
                ray = (vy - self.prev_vy) / dt
                raz = (vz - self.prev_vz) / dt

                ax = a * rax + (1.0 - a) * self.prev_ax
                ay = a * ray + (1.0 - a) * self.prev_ay
                az = a * raz + (1.0 - a) * self.prev_az
            # else: first sample, or the gap since the last one is too large
            # to trust a finite difference (track loss / reacquisition) --
            # leave velocity & acceleration at zero rather than emit a spike.

        self.prev_x, self.prev_y, self.prev_z = x, y, z
        self.prev_vx, self.prev_vy, self.prev_vz = vx, vy, vz
        self.prev_ax, self.prev_ay, self.prev_az = ax, ay, az
        self.prev_t = t
        self.have_prev = True

        self.last_point_wall_time = self.get_clock().now()
        self.target_active = True

        out = CVTarget()
        out.header = msg.header
        out.x = float(x)
        out.y = float(y)
        out.z = float(z)
        out.v_x = float(vx)
        out.v_y = float(vy)
        out.v_z = float(vz)
        out.a_x = float(ax)
        out.a_y = float(ay)
        out.a_z = float(az)
        out.confidence = float(
            self.latest_confidence if self.have_confidence else self.default_confidence)

        self.pub.publish(out)

    def check_timeout(self):
        if not self.target_active:
            return
        idle_s = (self.get_clock().now() - self.last_point_wall_time).nanoseconds / 1e9
        if idle_s <= self.target_timeout_s:
            return

        self.target_active = False
        self.have_prev = False  # force a clean restart of the velocity filter

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
