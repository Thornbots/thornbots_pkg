"""
Broadcasts the odom->root TF that slam_toolbox needs, straight from /odom.

We only ever have one odometry source at a time (gz sim's OdometryPublisher
bridged to /odom, or later a real one), so there's nothing to fuse; this
is a plain republish, not an EKF. robot_localization's ekf_node would do
the same job here but currently can't run in this environment (ABI
mismatch between the installed robot_localization and diagnostic_updater
packages, with no compatible diagnostic_updater available to install
instead). If that gets fixed upstream, this node can be swapped back out
for ekf_node with no other changes needed; both just need /odom in and
produce the same TF out.

yaw_joint_name/y_joint_name are sim-only workarounds: gz sim's
OdometryPublisher plugin reports /odom's Y position and orientation as
always 0/identity regardless of root's actual pose once the robot's base
is joint-constrained (see sim/urdf/sentry.urdf.xacro's planar_x/planar_y/
root chain); X position is, oddly, unaffected and reports correctly. This
was confirmed by checking gz's own ground-truth link pose directly (via
/world/.../pose/info), which is always correct even when /odom isn't; some
assumption of the plugin's about a freely-floating base no longer holds.
When set, Y/yaw are read from their respective joints' own positions (via
/joint_states) instead of /odom's position.y/orientation fields. Leave
both empty (the default) for real hardware, where pose_translator already
publishes correct odometry directly.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'root')
        self.declare_parameter('yaw_joint_name', '')
        self.declare_parameter('y_joint_name', '')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.yaw_joint_name = self.get_parameter('yaw_joint_name').value
        self.y_joint_name = self.get_parameter('y_joint_name').value
        self.yaw_override = None
        self.y_override = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )
        if self.yaw_joint_name or self.y_joint_name:
            self.joint_sub = self.create_subscription(
                JointState, '/joint_states', self.joint_state_callback, 10
            )

    def joint_state_callback(self, msg):
        if self.yaw_joint_name in msg.name:
            idx = msg.name.index(self.yaw_joint_name)
            self.yaw_override = msg.position[idx]
        if self.y_joint_name in msg.name:
            idx = msg.name.index(self.y_joint_name)
            self.y_override = msg.position[idx]

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = (
            self.y_override if self.y_override is not None else msg.pose.pose.position.y
        )
        t.transform.translation.z = msg.pose.pose.position.z
        if self.yaw_override is not None:
            t.transform.rotation = Quaternion(
                x=0.0, y=0.0,
                z=math.sin(self.yaw_override / 2.0),
                w=math.cos(self.yaw_override / 2.0),
            )
        else:
            t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
