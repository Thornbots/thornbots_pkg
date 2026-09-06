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

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    """
    Broadcasts odom_frame->base_frame TF from /localization/odom.

    sentry_localization owns the actual localization computation (its
    backend -- passthrough, EKF fusion, slam_toolbox, or AMCL -- decides
    what odom->root really is) and always publishes the result on
    /localization/odom (nav_msgs/Odometry), regardless of backend. This
    node's only job is turning that into the odom->root TF edge, so
    thornbots_pkg never needs to know which localization_mode is active.
    """

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'root')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, '/localization/odom', self._odom_cb, 10
        )

    def _odom_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.get_parameter('odom_frame').value
        t.child_frame_id = self.get_parameter('base_frame').value
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
