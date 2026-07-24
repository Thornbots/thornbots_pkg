import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from dji_serial_bridge.msg import CVTarget


class McbRelay(Node):
    """
    Sole relay between sentry_pkg/sentry_localization/CV and
    dji_serial_bridge_node's topics. Per project convention, only sentry_pkg
    is allowed to publish/subscribe directly on dji_serial_bridge's topics --
    dji_serial_bridge itself stays a pure UART/DJI-protocol translator, with
    no other package wired to it. This node reads each upstream package's own
    output and reshapes it into whatever dji_serial_bridge_node's
    subscription expects.

    relocalize: compares localization_odom_topic (/localization/odom --
    sentry_localization's one guaranteed output, published by every
    localization_mode: slam/mapping/amcl/ekf -- see
    sentry_localization/README.md) against raw_odom_topic (/odom --
    sentry_pkg's own pose_translator, i.e. the MCB's raw uncorrected wheel
    odometry). Deliberately backend-agnostic: no TF lookups, no assumption
    about which localization_mode is running, just two Odometry topics. Once
    they've drifted apart by more than error_threshold_meters *and* the
    chassis is nearly stationary (raw odom speed below max_move_speed, so
    the correction isn't stale by the time the MCB applies it), publishes
    the localized (x, y) as a Point on relocalize_output_topic --
    dji_serial_bridge_node's ~/relocalize (default resolved name
    /dji_serial_bridge/relocalize) packs this into a RelocalizePayload and
    sends it over UART so the MCB can reset its own odometry origin.

    cv_target: republishes cv_target_input_topic (dji_serial_bridge/msg/
    CVTarget, from the CV pipeline) unchanged onto cv_target_output_topic --
    dji_serial_bridge_node's ~/cv_target (default resolved name
    /dji_serial_bridge/cv_target).
    """

    def __init__(self):
        super().__init__('mcb_relay')

        self.declare_parameter('localization_odom_topic', '/localization/odom')
        self.declare_parameter('raw_odom_topic', '/odom')
        self.declare_parameter('relocalize_output_topic', '/dji_serial_bridge/relocalize')
        self.declare_parameter('error_threshold_meters', 0.05)
        self.declare_parameter('max_move_speed', 0.05)
        self.declare_parameter('cv_target_input_topic', '/cv/target')
        self.declare_parameter('cv_target_output_topic', '/dji_serial_bridge/cv_target')

        localization_odom_topic = self.get_parameter('localization_odom_topic').value
        raw_odom_topic = self.get_parameter('raw_odom_topic').value
        relocalize_out = self.get_parameter('relocalize_output_topic').value
        self._error_threshold = self.get_parameter('error_threshold_meters').value
        self._max_move_speed = self.get_parameter('max_move_speed').value
        cv_target_in = self.get_parameter('cv_target_input_topic').value
        cv_target_out = self.get_parameter('cv_target_output_topic').value

        self._raw_x = 0.0
        self._raw_y = 0.0
        self._raw_speed = 0.0
        self._have_raw_odom = False

        self.relocalize_pub = self.create_publisher(Point, relocalize_out, 10)
        self.raw_odom_sub = self.create_subscription(
            Odometry, raw_odom_topic, self._raw_odom_callback, 10)
        self.localization_odom_sub = self.create_subscription(
            Odometry, localization_odom_topic, self._localization_odom_callback, 10)

        # Matches dji_serial_bridge_node's SensorDataQoS ~/cv_target sub.
        self.cv_target_pub = self.create_publisher(
            CVTarget, cv_target_out, qos_profile_sensor_data)
        self.cv_target_sub = self.create_subscription(
            CVTarget, cv_target_in, self.cv_target_pub.publish, qos_profile_sensor_data)

        self.get_logger().info(
            f"mcb_relay ready\n"
            f"  {localization_odom_topic} vs {raw_odom_topic} -> {relocalize_out}"
            f" (threshold={self._error_threshold}m, max_move_speed={self._max_move_speed}m/s)\n"
            f"  {cv_target_in} -> {cv_target_out}"
        )

    def _raw_odom_callback(self, msg):
        self._raw_x = msg.pose.pose.position.x
        self._raw_y = msg.pose.pose.position.y
        self._raw_speed = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self._have_raw_odom = True

    def _localization_odom_callback(self, msg):
        # Abort if we haven't seen raw odom yet, or the chassis is moving too
        # fast for a relocalize correction to still be valid by the time the
        # MCB applies it.
        if not self._have_raw_odom or self._raw_speed > self._max_move_speed:
            return

        loc_x = msg.pose.pose.position.x
        loc_y = msg.pose.pose.position.y
        error = math.hypot(loc_x - self._raw_x, loc_y - self._raw_y)
        if error <= self._error_threshold:
            return

        point = Point(x=loc_x, y=loc_y, z=0.0)
        self.relocalize_pub.publish(point)
        self.get_logger().info(
            f"Localization drifted {error:.3f}m from raw odom - sent "
            f"relocalize correction x={loc_x:.3f} y={loc_y:.3f}",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = McbRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
