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

// Relay all traffic to and from dji_serial_bridge_node: the sole relay onto
// its topics; only thornbots_pkg talks to dji_serial_bridge directly.
//
// relocalize: publishes corrected (x, y) on relocalize_output_topic when
// localization_odom_topic and raw_odom_topic drift apart and the offset is
// confident, compensated for UART lag and the MCB's read delay
// (mcb_relay_core.hpp's Relocalizer). Stamped with when the MCB should adopt it.
// cv_target: republishes cv_target_input_topic onto cv_target_output_topic.
// The aim point carries the fire decision (fire, delay_ms), so this is the only
// CV relay. See README.md for design rationale.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/simulation_relocalize.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "thornbots_pkg/mcb_relay_core.hpp"

namespace thornbots_pkg
{

using dji_serial_bridge::msg::CVTarget;
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
      "localization_odom_topic", std::string("/localization/odom"));
    const auto raw_odom_topic = declare_parameter("raw_odom_topic", std::string("/odom"));
    navigation_frame_ = declare_parameter("navigation_frame", std::string("nav_odom"));
    const auto relocalize_out = declare_parameter(
      "relocalize_output_topic", std::string("/dji_serial_bridge/relocalize"));
    Relocalizer::Params p;
    p.error_threshold_m = declare_parameter("error_threshold_meters", 0.05);
    p.n_sigma = declare_parameter("n_sigma", 3.0);  // offset must clear this many std
    p.max_std_m = declare_parameter("max_std_m", 0.02);  // EKF reads ~0.001 still, ~0.007 moving
    // Placeholders, unmeasured: one UART leg (USB-serial latency timer
    // included), the MCB's poll of its RX buffer, and their spread.
    p.uart_latency_s = declare_parameter("uart_latency_s", 0.005);
    p.mcb_read_delay_s = declare_parameter("mcb_read_delay_s", 0.002);
    p.latency_std_s = declare_parameter("latency_std_s", 0.003);
    p.hold_off_s = declare_parameter("hold_off_s", 0.3);  // limit navigation fix rate
    const auto cv_target_in = declare_parameter("cv_target_input_topic", std::string("/cv/target"));
    const auto cv_target_out = declare_parameter(
      "cv_target_output_topic", std::string("/dji_serial_bridge/cv_target"));
    relocalizer_ = Relocalizer(p);
    lockstep_ = declare_parameter("lockstep_inputs", false);
    if (lockstep_) {
      command_pub_ = create_publisher<dji_serial_bridge::msg::SimulationRelocalize>(
        "/sim/lockstep/relocalize_command", 100);
      receipt_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/sim/lockstep/ekf_output", 100,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
          const auto tick = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (msg->status.size() != 1 ||
            (msg->status[0].message != "published" && msg->status[0].message != "no_output"))
          {throw std::runtime_error("invalid EKF output receipt");}
          if (!receipts_.emplace(tick, *msg).second || receipts_.size() > 100) {
            throw std::runtime_error("lockstep timeout: relay receipt overflow/duplicate");
          }
          drain();
        });
      retry_ = create_wall_timer(std::chrono::milliseconds(1), [this] {drain();});
    }

    relocalize_pub_ = create_publisher<PointStamped>(relocalize_out, 10);
    raw_odom_sub_ = create_subscription<Odometry>(
      raw_odom_topic, 10, [this](Odometry::ConstSharedPtr m) {raw_odom_callback(*m);});
    localization_odom_sub_ = create_subscription<Odometry>(
      localization_odom_topic, 10,
      [this](Odometry::ConstSharedPtr m) {localization_odom_callback(*m);});

    // Matches dji_serial_bridge_node's SensorDataQoS ~/cv_target sub.
    cv_target_pub_ = create_publisher<CVTarget>(cv_target_out, rclcpp::SensorDataQoS());
    cv_target_sub_ = create_subscription<CVTarget>(
      cv_target_in, rclcpp::SensorDataQoS(),
      [this](CVTarget::ConstSharedPtr m) {cv_target_pub_->publish(*m);});

    RCLCPP_INFO(
      get_logger(),
      "mcb_relay ready\n  %s vs %s -> %s (threshold=%gm, max_std=%gm)\n"
      "  %s -> %s (aim + fire decision)",
      localization_odom_topic.c_str(), raw_odom_topic.c_str(), relocalize_out.c_str(),
      p.error_threshold_m, p.max_std_m, cv_target_in.c_str(), cv_target_out.c_str());
  }

private:
  void raw_odom_callback(const Odometry & msg)
  {
    if (lockstep_) {
      raw_stamp_ns_ = rclcpp::Time(msg.header.stamp).nanoseconds();
      if (!raw_pending_.emplace(raw_stamp_ns_, msg).second || raw_pending_.size() > 1000) {
        throw std::runtime_error("lockstep timeout: relay raw input overflow/duplicate");
      }
      drain();
      return;
    }
    add_raw(msg);
  }

  void add_raw(const Odometry & msg)
  {
    // Twist is in the child frame; the chassis holds its heading, so it reads
    // as odom (the old speed gate relied on the same).
    relocalizer_.add_odom(
      stamp_s(msg.header.stamp), msg.pose.pose.position.x, msg.pose.pose.position.y,
      msg.twist.twist.linear.x, msg.twist.twist.linear.y);
  }

  void localization_odom_callback(const Odometry & msg)
  {
    if (lockstep_) {
      filtered_[rclcpp::Time(msg.header.stamp).nanoseconds()] = msg;
      if (filtered_.size() > 100) {
        throw std::runtime_error("lockstep timeout: relay filtered input overflow");
      }
      drain();
      return;
    }
    const auto point = correction(msg, now().nanoseconds());
    if (point) {relocalize_pub_->publish(*point);}
  }

  std::optional<PointStamped> correction(const Odometry & msg, int64_t decision_ns)
  {
    const auto & cov = msg.pose.covariance;
    const auto out = relocalizer_.decide(
      stamp_s(msg.header.stamp), msg.pose.pose.position.x, msg.pose.pose.position.y,
      std::max(cov[0], cov[7]), static_cast<double>(decision_ns) * 1e-9);
    if (!out) {
      return std::nullopt;
    }
    PointStamped point;
    point.header.stamp = rclcpp::Time(static_cast<int64_t>(out->apply_t * 1e9));
    point.header.frame_id = navigation_frame_;
    point.point.x = out->x;
    point.point.y = out->y;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Localization %.3fm (std %.3fm) from estimated MCB navigation - sent relocalize x=%.3f y=%.3f",
      out->error_m, out->std_m, out->x, out->y);
    return point;
  }

  void drain()
  {
    if (receipts_.empty()) {return;}
    const auto & [tick, receipt] = *receipts_.begin();
    if (now().nanoseconds() < tick) {return;}
    if (now().nanoseconds() > tick) {
      throw std::runtime_error("lockstep timeout: relay missed decision tick");
    }
    const auto & status = receipt.status[0];
    int64_t output_stamp = -1, raw_stamp = -1;
    for (const auto & value : status.values) {
      if (value.key == "output_stamp_ns") {output_stamp = std::stoll(value.value);}
      if (value.key == "raw_stamp_ns") {raw_stamp = std::stoll(value.value);}
    }
    if (output_stamp < 0 || raw_stamp < 0) {
      throw std::runtime_error("invalid EKF output receipt stamps");
    }
    if (raw_stamp_ns_ < raw_stamp) {return;}
    for (auto input = raw_pending_.begin(); input != raw_pending_.end() &&
      input->first <= raw_stamp;)
    {
      add_raw(input->second);
      input = raw_pending_.erase(input);
    }
    dji_serial_bridge::msg::SimulationRelocalize command;
    command.header = receipt.header;
    if (status.message == "published") {
      const auto input = filtered_.find(output_stamp);
      if (input == filtered_.end()) {return;}
      const auto point = correction(input->second, tick);
      command.has_correction = point.has_value();
      if (point) {command.correction = *point;}
      filtered_.erase(filtered_.begin(), std::next(input));
    }
    command_pub_->publish(command);
    receipts_.erase(receipts_.begin());
  }

  Relocalizer relocalizer_;
  bool lockstep_ = false;
  int64_t raw_stamp_ns_ = 0;
  std::map<int64_t, Odometry> filtered_;
  std::map<int64_t, Odometry> raw_pending_;
  std::map<int64_t, diagnostic_msgs::msg::DiagnosticArray> receipts_;
  rclcpp::Publisher<dji_serial_bridge::msg::SimulationRelocalize>::SharedPtr command_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr receipt_sub_;
  rclcpp::TimerBase::SharedPtr retry_;
  std::string navigation_frame_;
  rclcpp::Publisher<PointStamped>::SharedPtr relocalize_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr raw_odom_sub_, localization_odom_sub_;
  rclcpp::Publisher<CVTarget>::SharedPtr cv_target_pub_;
  rclcpp::Subscription<CVTarget>::SharedPtr cv_target_sub_;
};

}  // namespace thornbots_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_pkg::McbRelay>());
  rclcpp::shutdown();
  return 0;
}
