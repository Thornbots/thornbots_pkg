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

from dji_serial_bridge.msg import RobotPose
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class PoseTranslator(Node):
    """
    Turn /pose into /odom and /joint_states.

    Turns /pose (dji_serial_bridge/msg/RobotPose, from real hardware or
    sim/pose_emulator.py) into /odom and /joint_states -- raw, uncorrected
    wheel odometry, not the localized odom->root pose. sentry_localization
    consumes /odom and publishes the corrected result on /localization/odom;
    this package's odom_tf_broadcaster turns that back into odom->root TF.
    See thornbots_pkg/README.md for the full pose_translator ->
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

        # First-pass covariance, not measured/validated. Non-zero (1cm stddev)
        # is required so the EKF can weight rf2o's scan-matched estimate
        # against this source; unset fields (z/roll/pitch/yaw) stay 0, which
        # is fine since odom0_config in ekf.yaml excludes them from fusion.
        # see README.md for design rationale
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
        q_chassis = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

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


def main(args=None):
    rclpy.init(args=args)
    node = PoseTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
