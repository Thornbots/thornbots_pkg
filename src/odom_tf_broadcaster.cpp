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

// Raw odom owns root; localization owns only the upstream navigation correction.
// See README.md, pose_translator, for the frame tree and heading-fixed base.

#include <memory>
#include <stdexcept>
#include <string>

#include "thornbots_pkg/aim_frames.hpp"
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
    aim_frame_ = declare_parameter("aim_frame", std::string("odom"));
    navigation_frame_ = declare_parameter("odom_frame", std::string("nav_odom"));
    base_frame_ = declare_parameter("base_frame", std::string("root"));
    if (aim_frame_ == navigation_frame_) {
      throw std::invalid_argument("raw aim and navigation frames must differ");
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    raw_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        frames_.add_raw(seconds(m->header.stamp), pose(*m));
        send(m->header.stamp, aim_frame_, base_frame_, pose(*m));
        flush_navigation();
      });
    navigation_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        frames_.add_navigation(rclcpp::Time(m->header.stamp).nanoseconds(), pose(*m));
        flush_navigation();
      });
  }

private:
  static double seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(rclcpp::Time(stamp).nanoseconds()) * 1e-9;
  }

  static Eigen::Isometry3d pose(const nav_msgs::msg::Odometry & msg)
  {
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    const auto & p = msg.pose.pose;
    out.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    out.linear() = Eigen::Quaterniond(
      p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).toRotationMatrix();
    return out;
  }

  void flush_navigation()
  {
    for (const auto & sample : frames_.take_navigation()) {
      const builtin_interfaces::msg::Time stamp = rclcpp::Time(sample.first);
      send(stamp, navigation_frame_, aim_frame_, sample.second);
    }
  }

  void send(
    const builtin_interfaces::msg::Time & stamp, const std::string & parent,
    const std::string & child, const Eigen::Isometry3d & pose)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = pose.translation().x();
    t.transform.translation.y = pose.translation().y();
    t.transform.translation.z = pose.translation().z();
    const Eigen::Quaterniond q(pose.linear());
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_sub_, navigation_sub_;
  AimFrames frames_;
  std::string aim_frame_, navigation_frame_, base_frame_;
};

}  // namespace thornbots_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_pkg::OdomTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
