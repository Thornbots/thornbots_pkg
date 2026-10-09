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

// Turn /dji_serial_bridge/pose (RobotPose, from real hardware or sim's
// pose_emulator) into /odom and /joint_states: raw, uncorrected wheel
// odometry, not the localized odom->root pose. sentry_localization consumes
// /odom and publishes the corrected result on /localization/odom;
// odom_tf_broadcaster turns that back into odom->root TF. See README.md for
// the full pipeline.

#include <array>
#include <memory>
#include <string>

#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace thornbots_pkg
{

using dji_serial_bridge::msg::RobotPose;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::JointState;

class PoseTranslator : public rclcpp::Node
{
public:
  PoseTranslator()
  : Node("pose_translator")
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

private:
  void pose_callback(const RobotPose & msg)
  {
    const auto odom_frame = get_parameter("odom_frame").as_string();
    const auto base_frame = get_parameter("base_frame").as_string();

    // Use the sensor's own timestamp so TF lines up with the LiDAR's scan
    // timestamps. Falling back to wall-clock time here causes the slam_toolbox
    // message_filter to reject scans once the two clocks drift apart, which
    // fills the filter queue and drops every scan.
    builtin_interfaces::msg::Time stamp = msg.header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      if (!warned_zero_stamp_) {
        RCLCPP_WARN(
          get_logger(),
          "RobotPose header.stamp is unset (0,0) - falling back to wall-clock time. "
          "TF may drift out of sync with the LiDAR and cause slam_toolbox to drop scans.");
        warned_zero_stamp_ = true;
      }
      stamp = get_clock()->now();
    }

    Odometry odom;
    odom.header.stamp = stamp;
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
    odom_pub_->publish(odom);

    JointState js;
    js.header.stamp = stamp;
    js.name = {"chassis_yaw", "headlink", "headpitch"};
    js.position = {msg.chassis_yaw, msg.head_yaw, msg.head_pitch};
    joint_pub_->publish(js);
  }

  rclcpp::Subscription<RobotPose>::SharedPtr sub_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<JointState>::SharedPtr joint_pub_;
  bool warned_zero_stamp_ = false;
  std::array<double, 36> pose_covariance_, twist_covariance_;
};

}  // namespace thornbots_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_pkg::PoseTranslator>());
  rclcpp::shutdown();
  return 0;
}
