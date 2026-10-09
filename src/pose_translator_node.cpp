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

// Raw wheel odometry and joint state from RobotPose; see README.md for design rationale.

#include <string>

#include "thornbots_pkg/pose_translator_node.hpp"

namespace thornbots_pkg
{

using dji_serial_bridge::msg::RobotPose;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::JointState;

PoseTranslator::PoseTranslator(const rclcpp::NodeOptions & options)
: Node("pose_translator", options)
{
  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("base_frame", std::string("root"));

  // Subscribe to the Type-C board custom interface topic (or sim's
  // pose_emulator, which publishes the same topic/message).
  sub_ = create_subscription<RobotPose>(
    "/dji_serial_bridge/pose", rclcpp::QoS(10).best_effort(),
    [this](RobotPose::ConstSharedPtr m) {pose_callback(*m);});
  odom_pub_ = create_publisher<Odometry>("/odom", 10);
  joint_pub_ = create_publisher<JointState>("/joint_states", 10);

  // First-pass covariance, not measured/validated. Non-zero (1cm stddev) is
  // required so the EKF can weight rf2o's scan-matched estimate against this
  // source; unset fields (z/roll/pitch/yaw) stay 0, which is fine since
  // odom0_config in ekf.yaml excludes them from fusion.
  // see README.md for design rationale
  constexpr double POS_VAR = 0.01 * 0.01;
  constexpr double VEL_VAR = 0.01 * 0.01;
  pose_covariance_.fill(0.0);
  pose_covariance_[0] = POS_VAR;  // x
  pose_covariance_[7] = POS_VAR;  // y
  twist_covariance_.fill(0.0);
  twist_covariance_[0] = VEL_VAR;  // vx
  twist_covariance_[7] = VEL_VAR;  // vy
}

void PoseTranslator::pose_callback(const RobotPose & msg)
{
  const auto odom_frame = get_parameter("odom_frame").as_string();
  const auto base_frame = get_parameter("base_frame").as_string();

  // Preserve capture time exactly, including simulation epoch zero and negative seconds.
  Odometry odom;
  odom.header.stamp = msg.header.stamp;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_frame;
  odom.pose.pose.position.x = msg.x;
  odom.pose.pose.position.y = msg.y;
  // root is heading-fixed: chassis yaw goes out as the chassis_yaw joint,
  // never into /odom, so localization doesn't see it. README.md.
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.covariance = pose_covariance_;
  odom.twist.twist.linear.x = msg.vel_x;
  odom.twist.twist.linear.y = msg.vel_y;
  odom.twist.covariance = twist_covariance_;

  JointState js;
  js.header.stamp = msg.header.stamp;
  js.name = {"chassis_yaw", "headlink", "headpitch"};
  js.position = {msg.chassis_yaw, msg.head_yaw, msg.head_pitch};
  publish_outputs(odom, js);
}

void PoseTranslator::publish_outputs(const Odometry & odom, const JointState & joints)
{
  odom_pub_->publish(odom);
  joint_pub_->publish(joints);
}

}  // namespace thornbots_pkg
