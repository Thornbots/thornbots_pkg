import math

from dji_serial_bridge.msg import CVTarget, FireCommand
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class McbRelay(Node):
    """
    Relay all traffic to and from dji_serial_bridge_node.

    Sole relay onto dji_serial_bridge_node's topics -- only thornbots_pkg talks
    to dji_serial_bridge directly. See README.md for design rationale.

    relocalize: publishes corrected (x, y) on relocalize_output_topic when
    localization_odom_topic and raw_odom_topic drift apart while stationary.
    cv_target: republishes cv_target_input_topic onto cv_target_output_topic.
    fire_command: republishes fire_command_input_topic onto
    fire_command_output_topic -- the firing-logic node publishes on the
    input side, this is what actually reaches dji_serial_bridge_node (and
    the real launcher hardware).
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
        self.declare_parameter('fire_command_input_topic', '/sentry/fire_command')
        self.declare_parameter('fire_command_output_topic', '/dji_serial_bridge/fire_command')

        localization_odom_topic = self.get_parameter('localization_odom_topic').value
        raw_odom_topic = self.get_parameter('raw_odom_topic').value
        relocalize_out = self.get_parameter('relocalize_output_topic').value
        self._error_threshold = self.get_parameter('error_threshold_meters').value
        self._max_move_speed = self.get_parameter('max_move_speed').value
        cv_target_in = self.get_parameter('cv_target_input_topic').value
        cv_target_out = self.get_parameter('cv_target_output_topic').value
        fire_command_in = self.get_parameter('fire_command_input_topic').value
        fire_command_out = self.get_parameter('fire_command_output_topic').value

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

        # Fire decisions are discrete events, not a sensor stream -- default
        # reliable QoS so none get dropped, unlike cv_target's best-effort.
        self.fire_command_pub = self.create_publisher(
            FireCommand, fire_command_out, 10)
        self.fire_command_sub = self.create_subscription(
            FireCommand, fire_command_in, self.fire_command_pub.publish, 10)

        self.get_logger().info(
            f'mcb_relay ready\n'
            f'  {localization_odom_topic} vs {raw_odom_topic} -> {relocalize_out}'
            f' (threshold={self._error_threshold}m, max_move_speed={self._max_move_speed}m/s)\n'
            f'  {cv_target_in} -> {cv_target_out}\n'
            f'  {fire_command_in} -> {fire_command_out}'
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
            f'Localization drifted {error:.3f}m from raw odom - sent '
            f'relocalize correction x={loc_x:.3f} y={loc_y:.3f}',
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
