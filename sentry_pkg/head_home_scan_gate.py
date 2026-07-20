import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState


class HeadHomeScanGate(Node):
    """
    Only forwards /scan -> /scan_gated while the head is near its home
    (yaw ~ 0) position.

    rf2o_laser_odometry caches the lidar->base_frame transform once, on its
    first received scan, and reuses it for the node's lifetime (it never
    re-queries TF per scan) -- so it implicitly assumes a rigidly-fixed
    sensor mount. Our lidar is head-mounted and the head moves
    independently under firmware control, which breaks that assumption
    whenever the head isn't at the angle it happened to be at on rf2o's
    first scan. Gating rf2o's input to only the (rare, but recurring
    whenever the robot returns the head home) windows where the head is
    at home keeps rf2o's cached transform valid for every scan it ever
    sees, at the cost of only producing /scan_odom updates during those
    windows.
    """

    def __init__(self):
        super().__init__('head_home_scan_gate')

        self.declare_parameter('head_joint_name', 'headlink')
        self.declare_parameter('home_yaw_tolerance', 0.05)

        self._head_yaw = None

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._scan_pub = self.create_publisher(LaserScan, '/scan_gated', 10)

    def _joint_state_cb(self, msg):
        joint_name = self.get_parameter('head_joint_name').value
        if joint_name in msg.name:
            self._head_yaw = msg.position[msg.name.index(joint_name)]

    def _scan_cb(self, msg):
        if self._head_yaw is None:
            return
        tolerance = self.get_parameter('home_yaw_tolerance').value
        if abs(self._head_yaw) <= tolerance:
            self._scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadHomeScanGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
