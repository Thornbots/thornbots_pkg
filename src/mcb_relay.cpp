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

// Sole outgoing application relay. Absolute map fixes and raw odometry feed
// the ROS-free Relocalizer; CV stays in raw odom. See README.md#mcb_relay.

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "thornbots_pkg/mcb_relay_core.hpp"

namespace thornbots_pkg
{

using dji_serial_bridge::msg::CVTarget;
using dji_serial_bridge::msg::TargetState;
using geometry_msgs::msg::PointStamped;
using nav_msgs::msg::Odometry;

namespace
{
double stamp_s(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(rclcpp::Time(t).nanoseconds()) * 1e-9;
}
}  // namespace

class McbRelay : public rclcpp::Node
{
public:
  McbRelay()
  : Node("mcb_relay")
  {
    const auto localization_odom_topic = declare_parameter(
      "localization_odom_topic", std::string("/localization/map_odom"));
    const auto raw_odom_topic = declare_parameter("raw_odom_topic", std::string("/odom"));
    navigation_frame_ = declare_parameter("navigation_frame", std::string("map"));
    const auto relocalize_out = declare_parameter(
      "relocalize_output_topic", std::string("/dji_serial_bridge/relocalize"));
    Relocalizer::Params p;
    p.absolute_policy = declare_parameter("absolute_pose", true);
    p.error_threshold_m = declare_parameter("error_threshold_meters", p.absolute_policy ? 0.10 : 0.05);
    p.n_sigma = declare_parameter("n_sigma", p.absolute_policy ? 0.5 : 3.0);
    p.max_std_m = declare_parameter("max_std_m", p.absolute_policy ? 0.35 : 0.02);
    // Placeholders, unmeasured: one UART leg (USB-serial latency timer
    // included), the MCB's poll of its RX buffer, and their spread.
    p.uart_latency_s = declare_parameter("uart_latency_s", 0.005);
    p.mcb_read_delay_s = declare_parameter("mcb_read_delay_s", 0.002);
    p.latency_std_s = declare_parameter("latency_std_s", 0.003);
    p.hold_off_s = declare_parameter("hold_off_s", 0.3);  // limit navigation fix rate
    const auto cv_target_in = declare_parameter("cv_target_input_topic", std::string("/cv/target"));
    const auto cv_target_out = declare_parameter(
      "cv_target_output_topic", std::string("/dji_serial_bridge/cv_target"));
    const auto target_state_in = declare_parameter(
      "target_state_input_topic", std::string("/cv/target_state"));
    const auto target_state_out = declare_parameter(
      "target_state_output_topic", std::string("/dji_serial_bridge/target_state"));
    // MCB patrol permission for both outputs; false stops firmware patrol.
    type_c_based_patrol_ = declare_parameter("type_c_based_patrol", false);
    p.max_pose_age_s = declare_parameter("max_pose_age_s", 0.2);
    p.max_jump_m = declare_parameter("max_jump_m", 1.0);
    p.max_step_m = declare_parameter("max_step_m", 0.3);
    p.startup_span_s = declare_parameter("startup_span_s", 0.1);
    p.startup_spread_m = declare_parameter("startup_spread_m", 0.15);
    p.refresh_s = declare_parameter("refresh_s", 1.0);
    relocalizer_ = Relocalizer(p);
    navigation_pub_ = create_publisher<Odometry>("/mcb/navigation_odom", 10);
    // Lockstep simulation only: decide on the localization sample's own stamp, not this
    // node's clock, which can trail the sample's by scheduling; and answer each raw
    // odometry sample so the POSE probe knows the relay has it.
    message_clock_ = declare_parameter("message_clock", false);
    const auto odom_ack_topic = declare_parameter("raw_odom_ack_topic", std::string(""));
    if (!odom_ack_topic.empty()) {
      odom_ack_pub_ = create_publisher<std_msgs::msg::Header>(odom_ack_topic, 100);
    }

    relocalize_pub_ = create_publisher<PointStamped>(relocalize_out, 10);
    // Lockstep: a Header per localization sample that sends no RELOCALIZE.
    const auto none_topic = declare_parameter("relocalize_none_topic", std::string(""));
    if (!none_topic.empty()) {
      none_pub_ = create_publisher<std_msgs::msg::Header>(none_topic, 100);
    }
    raw_odom_sub_ = create_subscription<Odometry>(
      raw_odom_topic, 10, [this](Odometry::ConstSharedPtr m) {raw_odom_callback(*m);});
    localization_odom_sub_ = create_subscription<Odometry>(
      localization_odom_topic, 10,
      [this](Odometry::ConstSharedPtr m) {localization_odom_callback(*m);});

    // Matches dji_serial_bridge_node's SensorDataQoS ~/cv_target sub.
    cv_target_pub_ = create_publisher<CVTarget>(cv_target_out, rclcpp::SensorDataQoS());
    cv_target_sub_ = create_subscription<CVTarget>(
      cv_target_in, rclcpp::SensorDataQoS(),
      [this](CVTarget::ConstSharedPtr m) {
        CVTarget out = *m;
        out.type_c_based_patrol = type_c_based_patrol_;
        cv_target_pub_->publish(out);
      });

    // Matches dji_serial_bridge_node's SensorDataQoS ~/target_state sub.
    target_state_pub_ = create_publisher<TargetState>(target_state_out, rclcpp::SensorDataQoS());
    target_state_sub_ = create_subscription<TargetState>(
      target_state_in, rclcpp::SensorDataQoS(),
      [this](TargetState::ConstSharedPtr m) {
        TargetState out = *m;
        out.type_c_based_patrol = type_c_based_patrol_;
        target_state_pub_->publish(out);
      });

    RCLCPP_INFO(
      get_logger(),
      "mcb_relay ready\n  %s vs %s -> %s (threshold=%gm, max_std=%gm)\n"
      "  %s -> %s (idle look)\n  %s -> %s (target state), type_c_based_patrol=%s",
      localization_odom_topic.c_str(), raw_odom_topic.c_str(), relocalize_out.c_str(),
      p.error_threshold_m, p.max_std_m, cv_target_in.c_str(), cv_target_out.c_str(),
      target_state_in.c_str(), target_state_out.c_str(),
      type_c_based_patrol_ ? "true" : "false");
  }

private:
  void raw_odom_callback(const Odometry & msg)
  {
    // Twist is in the child frame; the chassis holds its heading, so it reads
    // as odom (the old speed gate relied on the same).
    relocalizer_.add_odom(
      stamp_s(msg.header.stamp), msg.pose.pose.position.x, msg.pose.pose.position.y,
      msg.twist.twist.linear.x, msg.twist.twist.linear.y);
    if (const auto nav = relocalizer_.navigation_at(stamp_s(msg.header.stamp))) {
      Odometry out = msg;
      out.header.frame_id = navigation_frame_;
      out.child_frame_id = "mcb_navigation";
      out.pose.covariance[0] = out.pose.covariance[7] = std::numeric_limits<double>::infinity();
      out.pose.pose.position.x = nav->first;
      out.pose.pose.position.y = nav->second;
      // Firmware holds a translation offset; its axes remain raw field-aligned axes.
      navigation_pub_->publish(out);
    }
    if (odom_ack_pub_) {
      odom_ack_pub_->publish(msg.header);
    }
  }

  void localization_odom_callback(const Odometry & msg)
  {
    const auto & cov = msg.pose.covariance;
    const double xy = 0.5 * (cov[1] + cov[6]);
    const double var = 0.5 * (cov[0] + cov[7] + std::hypot(cov[0] - cov[7], 2.0 * xy));
    const auto & q = msg.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
    const bool valid = msg.header.frame_id == navigation_frame_ &&
      std::isfinite(cov[0]) && std::isfinite(cov[7]) && std::isfinite(xy) &&
      cov[0] >= 0.0 && cov[7] >= 0.0 && cov[0]*cov[7] >= xy*xy;
    const auto out = relocalizer_.decide(
      stamp_s(msg.header.stamp), msg.pose.pose.position.x, msg.pose.pose.position.y,
      valid ? var : std::numeric_limits<double>::infinity(),
      message_clock_ ? stamp_s(msg.header.stamp) : get_clock()->now().seconds(), yaw, cov[35]);
    if (!out) {
      if (none_pub_) {
        std_msgs::msg::Header none;
        none = msg.header;
        none_pub_->publish(none);
      }
      return;
    }
    PointStamped point;
    point.header.stamp = rclcpp::Time(static_cast<int64_t>(out->apply_t * 1e9));
    point.header.frame_id = navigation_frame_;
    point.point.x = out->x;
    point.point.y = out->y;
    relocalize_pub_->publish(point);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Localization %.3fm (std %.3fm) from estimated MCB navigation - sent relocalize x=%.3f y=%.3f",
      out->error_m, out->std_m, out->x, out->y);
  }

  Relocalizer relocalizer_;
  std::string navigation_frame_;
  bool type_c_based_patrol_ = false;
  rclcpp::Publisher<PointStamped>::SharedPtr relocalize_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr navigation_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr none_pub_, odom_ack_pub_;
  bool message_clock_ = false;
  rclcpp::Subscription<Odometry>::SharedPtr raw_odom_sub_, localization_odom_sub_;
  rclcpp::Publisher<CVTarget>::SharedPtr cv_target_pub_;
  rclcpp::Subscription<CVTarget>::SharedPtr cv_target_sub_;
  rclcpp::Publisher<TargetState>::SharedPtr target_state_pub_;
  rclcpp::Subscription<TargetState>::SharedPtr target_state_sub_;
};

}  // namespace thornbots_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_pkg::McbRelay>());
  rclcpp::shutdown();
  return 0;
}
