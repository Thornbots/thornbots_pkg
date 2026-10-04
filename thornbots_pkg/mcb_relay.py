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

from dji_serial_bridge.msg import CVTarget
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from thornbots_pkg.mcb_axes import aim_to_mcb, to_mcb
from thornbots_pkg.mcb_relay_core import Relocalizer


class McbRelay(Node):
    """
    Relay all traffic to and from dji_serial_bridge_node.

    Sole relay onto dji_serial_bridge_node's topics -- only thornbots_pkg talks
    to dji_serial_bridge directly. See README.md for design rationale.

    relocalize: publishes corrected (x, y) on relocalize_output_topic when
    localization_odom_topic and raw_odom_topic drift apart and the offset is
    confident, compensated for UART lag and the MCB's read delay
    (mcb_relay_core.Relocalizer). Stamped with when the MCB should adopt it.
    cv_target: republishes cv_target_input_topic onto cv_target_output_topic.
    The aim point carries the fire decision (fire, delay_ms), so this is the
    only CV relay -- it is what reaches dji_serial_bridge_node and the real
    launcher hardware.
    mcb_x_right: both go out in the MCB's odometry axes (mcb_axes.py).
    """

    def __init__(self):
        super().__init__('mcb_relay')

        self.declare_parameter('localization_odom_topic', '/localization/odom')
        self.declare_parameter('raw_odom_topic', '/odom')
        self.declare_parameter('relocalize_output_topic', '/dji_serial_bridge/relocalize')
        self.declare_parameter('error_threshold_meters', 0.05)
        self.declare_parameter('n_sigma', 3.0)  # offset must clear this many std
        self.declare_parameter('max_std_m', 0.02)  # EKF reads ~0.001 still, ~0.007 moving
        # Placeholders, unmeasured: one UART leg (USB-serial latency timer
        # included), the MCB's poll of its RX buffer, and their spread.
        self.declare_parameter('uart_latency_s', 0.005)
        self.declare_parameter('mcb_read_delay_s', 0.002)
        self.declare_parameter('latency_std_s', 0.003)
        self.declare_parameter('hold_off_s', 0.3)  # after a send, for /odom to show it
        self.declare_parameter('cv_target_input_topic', '/cv/target')
        self.declare_parameter('cv_target_output_topic', '/dji_serial_bridge/cv_target')
        self.declare_parameter('mcb_x_right', False)  # as pose_translator's

        localization_odom_topic = self.get_parameter('localization_odom_topic').value
        raw_odom_topic = self.get_parameter('raw_odom_topic').value
        relocalize_out = self.get_parameter('relocalize_output_topic').value
        gp = self.get_parameter
        self._relocalizer = Relocalizer(
            error_threshold_m=gp('error_threshold_meters').value,
            n_sigma=gp('n_sigma').value, max_std_m=gp('max_std_m').value,
            uart_latency_s=gp('uart_latency_s').value,
            mcb_read_delay_s=gp('mcb_read_delay_s').value,
            latency_std_s=gp('latency_std_s').value, hold_off_s=gp('hold_off_s').value)
        cv_target_in = self.get_parameter('cv_target_input_topic').value
        cv_target_out = self.get_parameter('cv_target_output_topic').value
        self._x_right = self.get_parameter('mcb_x_right').value

        self.relocalize_pub = self.create_publisher(PointStamped, relocalize_out, 10)
        self.raw_odom_sub = self.create_subscription(
            Odometry, raw_odom_topic, self._raw_odom_callback, 10)
        self.localization_odom_sub = self.create_subscription(
            Odometry, localization_odom_topic, self._localization_odom_callback, 10)

        # Matches dji_serial_bridge_node's SensorDataQoS ~/cv_target sub.
        self.cv_target_pub = self.create_publisher(
            CVTarget, cv_target_out, qos_profile_sensor_data)
        self.cv_target_sub = self.create_subscription(
            CVTarget, cv_target_in, self._cv_target_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'mcb_relay ready\n'
            f'  {localization_odom_topic} vs {raw_odom_topic} -> {relocalize_out}'
            f' (threshold={self._relocalizer.error_threshold_m}m,'
            f' max_std={self._relocalizer.max_std_m}m)\n'
            f'  {cv_target_in} -> {cv_target_out} (aim + fire decision)\n'
            f'  mcb_x_right={self._x_right}'
        )

    def _cv_target_callback(self, msg):
        if self._x_right:
            # The MCB subtracts its odometry as it reads the frame.
            rel = self._relocalizer
            o = rel.odom_at(self.get_clock().now().nanoseconds * 1e-9
                            + rel.uart_latency_s + rel.mcb_read_delay_s)
            if o is None:
                self.get_logger().warn(
                    'no recent /odom: dropping cv_target, the MCB would '
                    'aim it in the wrong axes', throttle_duration_sec=1.0)
                return
            msg.x, msg.y = aim_to_mcb(msg.x, msg.y, *o)
        self.cv_target_pub.publish(msg)

    def _raw_odom_callback(self, msg):
        # Twist is in the child frame; the chassis holds its heading, so
        # it reads as odom (the old speed gate relied on the same).
        self._relocalizer.add_odom(
            Time.from_msg(msg.header.stamp).nanoseconds * 1e-9,
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def _localization_odom_callback(self, msg):
        cov = msg.pose.covariance
        now = self.get_clock().now()
        out = self._relocalizer.decide(
            Time.from_msg(msg.header.stamp).nanoseconds * 1e-9,
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            max(cov[0], cov[7]), now.nanoseconds * 1e-9)
        if out is None:
            return
        x, y, apply_t, error, std = out
        point = PointStamped()
        point.header.stamp = Time(nanoseconds=int(apply_t * 1e9)).to_msg()
        point.header.frame_id = msg.header.frame_id
        point.point.x, point.point.y = to_mcb(x, y) if self._x_right else (x, y)
        self.relocalize_pub.publish(point)
        self.get_logger().info(
            f'Localization {error:.3f}m (std {std:.3f}m) from raw odom - sent '
            f'relocalize x={x:.3f} y={y:.3f}', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = McbRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
