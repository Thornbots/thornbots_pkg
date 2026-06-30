#!/usr/bin/env python3
"""
slam_relocalize_publisher.py

Watches the map -> base_frame transform produced by slam_toolbox (via TF,
since async_slam_toolbox_node only publishes its correction as a transform,
not a topic) and publishes a geometry_msgs/Point on ~/relocalize at a fixed
rate.

This feeds dji_serial_bridge's existing `~/relocalize` subscriber, which
sends a RELOCALIZE frame (expectedX, expectedY) to the MCB. Only position
is corrected this way -- the firmware payload has no yaw field.

Parameters:
  map_frame          (string, default "map")
  base_frame          (string, default "root")     -- must match slam.yaml base_frame
  publish_rate_hz      (double, default 1.0)         -- how often to send a correction
  relocalize_topic    (string, default "relocalize") -- remap if bridge topic differs
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class SlamRelocalizePublisher(Node):
    def __init__(self):
        super().__init__("slam_relocalize_publisher")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "root")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("relocalize_topic", "relocalize")

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        publish_rate = self.get_parameter("publish_rate_hz").value
        topic = self.get_parameter("relocalize_topic").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Point, topic, 10)

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_pose)

        self.get_logger().info(
            f"slam_relocalize_publisher: watching {self.map_frame}->{self.base_frame}, "
            f"publishing at {publish_rate} Hz to '{topic}'"
        )

    def publish_pose(self):
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

        msg = Point()
        msg.x = tf.transform.translation.x
        msg.y = tf.transform.translation.y
        msg.z = 0.0
        self.pub.publish(msg)


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
