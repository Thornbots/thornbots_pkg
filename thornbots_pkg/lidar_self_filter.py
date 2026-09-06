"""
Filter the robot's own head out of the lidar scan.

Blanks out a fixed angular sector of /scan_raw where the robot's own head
sits in the lidar's FOV (fixed in the lidar's frame; no joint-state sub
needed), republishing on /scan. Works for sim and real hardware.

Current values: blind_angle_start=2.20, blind_angle_end=3.20 (1.0 rad),
sim-mesh-derived -- see README.md for design rationale and retuning notes.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSelfFilter(Node):

    def __init__(self):
        super().__init__('lidar_self_filter')

        self.declare_parameter('blind_angle_start', 2.20)
        self.declare_parameter('blind_angle_end', 3.20)

        self.sub = self.create_subscription(
            LaserScan, 'scan_raw', self.scan_callback, 10
        )
        self.pub = self.create_publisher(LaserScan, 'scan', 10)

    def scan_callback(self, msg):
        start = self.get_parameter('blind_angle_start').value
        end = self.get_parameter('blind_angle_end').value

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        ranges = list(msg.ranges)
        intensities = list(msg.intensities)
        two_pi = 2.0 * math.pi

        for i in range(len(ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            # Wrap into [0, 2*pi) so this compares correctly regardless of
            # whether the scan's own angle_min/angle_max convention starts
            # at 0 (as sim's gpu_lidar does) or spans a signed range like
            # [-pi, pi] (as real RPLIDAR drivers may).
            wrapped = angle % two_pi
            if start <= wrapped <= end:
                ranges[i] = float('inf')
                if i < len(intensities):
                    intensities[i] = 0.0

        out.ranges = ranges
        out.intensities = intensities
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSelfFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
