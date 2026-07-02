#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# TODO: Replace with your actual package and message name
from YOUR_PACKAGE_NAME.msg import YOUR_CUSTOM_MSG_TYPE 

class SlamRelocalizePublisher(Node):
    def __init__(self):
        super().__init__("slam_relocalize_publisher")
        
        # Declare parameters
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "root")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("relocalize_topic", "relocalize")
        self.declare_parameter("chassis_state_topic", "chassis_state")
        self.declare_parameter("error_threshold_meters", 0.05)         # Max allowed drift before resetting MCB
        self.declare_parameter("max_move_speed", 0.05)                 # Velocity suppression limit (m/s)

        # Get parameter values
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        publish_rate = self.get_parameter("publish_rate_hz").value
        topic = self.get_parameter("relocalize_topic").value
        chassis_topic = self.get_parameter("chassis_state_topic").value
        self.threshold = self.get_parameter("error_threshold_meters").value
        self.max_move_speed = self.get_parameter("max_move_speed").value

        # Data stores for incoming 100Hz telemetry
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.current_chassis_speed = 0.0
        self.has_odom_data = False

        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers and Publishers
        self.chassis_sub = self.create_subscription(
            YOUR_CUSTOM_MSG_TYPE, chassis_topic, self.chassis_callback, 10
        )
        self.pub = self.create_publisher(Point, topic, 10)
        
        # Timer loop for processing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_pose)
        
        self.get_logger().info(
            f"slam_relocalize_publisher: monitoring absolute error between SLAM and MCB odom, "
            f"threshold: {self.threshold}m, max speed: {self.max_move_speed}m/s"
        )

    def chassis_callback(self, msg):
        # Cache raw MCB positions
        self.odom_x = msg.x
        self.odom_y = msg.y
        # Compute instantaneous magnitude of velocity
        self.current_chassis_speed = math.sqrt(msg.vel_x**2 + msg.vel_y**2)
        self.has_odom_data = True

    def publish_pose(self):
        # Rule 1: Abort if we haven't received MCB data yet or if moving too fast
        if not self.has_odom_data or (self.current_chassis_speed > self.max_move_speed):
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup {self.map_frame}->{self.base_frame} failed: {e}",
                throttle_duration_sec=5.0,
            )
            return

        slam_x = tf.transform.translation.x
        slam_y = tf.transform.translation.y

        # Rule 2: Compute absolute distance error between SLAM truth and drifted MCB coordinates
        error = math.sqrt((slam_x - self.odom_x) ** 2 + (slam_y - self.odom_y) ** 2)

        if error > self.threshold:
            msg = Point()
            msg.x = slam_x
            msg.y = slam_y
            msg.z = 0.0

            self.pub.publish(msg)

            self.get_logger().info(
                f"MCB drifted by {error:.3f}m. Sent reset coordinates: X={slam_x:.3f}, Y={slam_y:.3f}",
                throttle_duration_sec=1.0
            )


def main():
    rclpy.init()
    node = SlamRelocalizePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
