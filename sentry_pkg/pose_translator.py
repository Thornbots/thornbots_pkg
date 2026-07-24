import rclpy
from rclpy.node import Node
from math import sin, cos
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from dji_serial_bridge.msg import RobotPose


class PoseTranslator(Node):
    """
    Turns /pose (dji_serial_bridge/msg/RobotPose, from real hardware or
    sim/pose_emulator.py) into /odom and /joint_states -- raw, uncorrected
    wheel odometry, not the localized odom->root pose. sentry_localization
    consumes /odom and publishes the corrected result on /localization/odom;
    this package's odom_tf_broadcaster turns that back into odom->root TF.
    See sentry_pkg/README.md for the full pose_translator ->
    sentry_localization -> odom_tf_broadcaster pipeline.
    """

    def __init__(self):
        super().__init__('pose_translator')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'root')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Subscribe to the Type-C board custom interface topic (or sim's
        # pose_emulator, which publishes the same topic/message)
        self.sub = self.create_subscription(
            RobotPose,
            '/pose',
            self.pose_callback,
            qos_profile
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self._warned_zero_stamp = False

        # First-pass covariance, not measured/validated (same disclaimer as
        # config/ekf.yaml's process_noise_covariance) -- but non-zero is the
        # important part. Left at all-zero, robot_localization's EKF has no
        # signal that this source's absolute x/y is any more or less
        # trustworthy than /scan_odom's, so it can't weight rf2o's
        # scan-matched estimate more heavily even when it should be -- see
        # SESSION_NOTES.md's 2026-07-24 EKF investigation. 1cm stddev on
        # position/velocity is a reasonable per-sample encoder-noise order
        # of magnitude to start from; unset fields (z/roll/pitch, and yaw --
        # this chassis is holonomic and never reports real orientation, see
        # the class docstring) are left at 0, which is fine since
        # odom0_config in ekf.yaml excludes them from fusion.
        POS_VAR = 0.01 ** 2
        VEL_VAR = 0.01 ** 2
        self._pose_covariance = [0.0] * 36
        self._pose_covariance[0] = POS_VAR   # x
        self._pose_covariance[7] = POS_VAR   # y
        self._twist_covariance = [0.0] * 36
        self._twist_covariance[0] = VEL_VAR  # vx
        self._twist_covariance[7] = VEL_VAR  # vy

    def pose_callback(self, msg):
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value

        # Use the sensor's own timestamp so TF lines up with the LiDAR's
        # scan timestamps. Falling back to wall-clock time here causes the
        # slam_toolbox message_filter to reject scans once the two clocks
        # drift apart, which fills the filter queue and drops every scan.
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            if not self._warned_zero_stamp:
                self.get_logger().warn(
                    'RobotPose header.stamp is unset (0,0) - falling back to '
                    'wall-clock time. TF may drift out of sync with the '
                    'LiDAR and cause slam_toolbox to drop scans.'
                )
                self._warned_zero_stamp = True
            stamp = self.get_clock().now().to_msg()

        # Chassis is holonomic and does not rotate; head_yaw is gimbal-only
        # heading, not chassis heading, so chassis orientation stays identity.
        q_chassis = self.euler_to_quaternion(0.0, 0.0, 0.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame

        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.orientation = q_chassis
        odom.pose.covariance = self._pose_covariance

        odom.twist.twist.linear.x = float(msg.vel_x)
        odom.twist.twist.linear.y = float(msg.vel_y)
        odom.twist.covariance = self._twist_covariance
        self.odom_pub.publish(odom)

        js = JointState()
        js.header.stamp = stamp
        js.name = ['headlink', 'headpitch']
        js.position = [float(msg.head_yaw), float(msg.head_pitch)]
        self.joint_pub.publish(js)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = cos(yaw * 0.5), sin(yaw * 0.5)
        cp, sp = cos(pitch * 0.5), sin(pitch * 0.5)
        cr, sr = cos(roll * 0.5), sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy  # fixed: was "* cp" instead of "* cy"
        return q

def main(args=None):
    rclpy.init(args=args)
    node = PoseTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
