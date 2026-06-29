import rclpy
from rclpy.node import Node
from math import sin, cos
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from dji_serial_bridge.msg import RobotPose

class PoseTranslator(Node):
    def __init__(self):
        super().__init__('pose_translator')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'root')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Subscribe to your Type-C board custom interface topic
        self.sub = self.create_subscription(
            RobotPose,
            '/dji_serial_bridge/pose',
            self.pose_callback,
            qos_profile
        )

        # Setup standard publishers and TF broadcasters for mapping
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_callback(self, msg):
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        
        # 1. Broadcast spatial transforms to the /tf tree for SLAM
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = odom_frame
        t.child_frame_id = base_frame
        
        t.transform.translation.x = float(msg.x)
        t.transform.translation.y = float(msg.y)
        t.transform.translation.z = 0.0
        
        q = self.euler_to_quaternion(0.0, 0.0, msg.head_yaw)
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Build standard Odometry information for your local planner
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        
        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x = float(msg.vel_x)
        odom.twist.twist.linear.y = float(msg.vel_y)
        self.odom_pub.publish(odom)

        # 3. Stream ONLY the yaw encoder joint state to your visual URDF layout
        js = JointState()
        js.header.stamp = msg.header.stamp
        js.name = ['headlink']
        js.position = [float(msg.head_yaw)]
        self.joint_pub.publish(js)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = cos(yaw * 0.5), sin(yaw * 0.5)
        cp, sp = cos(pitch * 0.5), sin(pitch * 0.5)
        cr, sr = cos(roll * 0.5), sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cp
        return q

def main(args=None):
    rclpy.init(args=args)
    node = PoseTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
