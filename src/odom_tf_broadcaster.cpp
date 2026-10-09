// Copyright 2026 Thornbots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Broadcast the odom_frame->base_frame TF from /localization/odom.
// sentry_localization owns the localization computation (passthrough, EKF
// fusion, slam_toolbox or AMCL decides what odom->root really is) and always
// publishes the result on /localization/odom (nav_msgs/Odometry). This node
// only turns that into the odom->root TF edge, so thornbots_pkg never needs
// to know which localization_mode is active.

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace thornbots_pkg
{

class OdomTfBroadcaster : public rclcpp::Node
{
public:
  OdomTfBroadcaster()
  : Node("odom_tf_broadcaster")
  {
    declare_parameter("odom_frame", std::string("odom"));
    declare_parameter("base_frame", std::string("root"));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {odom_cb(*m);});
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry & msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg.header.stamp;
    t.header.frame_id = get_parameter("odom_frame").as_string();
    t.child_frame_id = get_parameter("base_frame").as_string();
    t.transform.translation.x = msg.pose.pose.position.x;
    t.transform.translation.y = msg.pose.pose.position.y;
    t.transform.translation.z = msg.pose.pose.position.z;
    t.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

}  // namespace thornbots_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_pkg::OdomTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
