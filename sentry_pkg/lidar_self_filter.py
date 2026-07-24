"""
Blanks out a fixed angular sector of /scan_raw where the robot's own head
sits in the lidar's field of view, republishing the result on /scan.

The lidar is mounted rigidly on the head (see sim/urdf/sentry.urdf.xacro's
lidarlink joint), so head and lidar always rotate together as one unit --
whatever part of the head's own structure blocks the lidar's view sits at a
FIXED angle in the lidar's own frame regardless of headlink's current yaw,
even though that blocked WORLD-frame bearing sweeps around as headlink
rotates (see sim/head_sweep.py's docstring: sweeping headlink moves this
blind wedge around the world so SLAM eventually gets full coverage). That
fixed relationship is what makes a static angular filter here viable at
all -- no joint-state subscription needed.

Runs for both sim and real hardware (see sentry_pkg/launch/auto.launch.py):
sim's gpu_lidar is a rendering sensor with no physics collision to fall
back on, so trying to model this via mesh visibility in the URDF instead
(gz-sim's visibility_mask/visibility_flags) was unreliable -- it could
only ever be all-or-nothing per visual (either the whole head is invisible
to the lidar, seeing straight through it even where it should genuinely
occlude, or fully visible and back to reporting self-hits) and had no way
to express "block the beam here without counting it as a false detection
of the head's own mesh". Real hardware has no such trick available at all
(it's a physical beam). A software filter with a known fixed blind sector
is the one approach that actually works for both.

Defaults were measured empirically in sim with headlink at rest (0 rad):
/scan showed a contiguous cluster of near-range returns (indices ~1417 to
~1443 of 3000, angle_increment ~0.002094 rad) at roughly 2.967-3.022 rad,
consistent with the head's own bulk rather than noise. Widened somewhat
for margin. These are sim-mesh-derived and will likely need retuning
against a real /scan_raw capture before trusting them on real hardware.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSelfFilter(Node):
    def __init__(self):
        super().__init__('lidar_self_filter')

        self.declare_parameter('blind_angle_start', 2.90)
        self.declare_parameter('blind_angle_end', 3.10)

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
