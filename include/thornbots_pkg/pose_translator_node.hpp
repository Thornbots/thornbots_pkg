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

#ifndef THORNBOTS_PKG__POSE_TRANSLATOR_NODE_HPP_
#define THORNBOTS_PKG__POSE_TRANSLATOR_NODE_HPP_

#include <array>

#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace thornbots_pkg
{

class PoseTranslator : public rclcpp::Node
{
public:
  explicit PoseTranslator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void pose_callback(const dji_serial_bridge::msg::RobotPose & msg);
  // Capture production outputs in unit tests without spinning a ROS graph.
  virtual void publish_outputs(
    const nav_msgs::msg::Odometry & odom, const sensor_msgs::msg::JointState & joints);

private:
  rclcpp::Subscription<dji_serial_bridge::msg::RobotPose>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::array<double, 36> pose_covariance_, twist_covariance_;
};

}  // namespace thornbots_pkg

#endif  // THORNBOTS_PKG__POSE_TRANSLATOR_NODE_HPP_
