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

#include "thornbots_pkg/target_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>

namespace thornbots_pkg
{
TargetManagerNode::TargetManagerNode(const rclcpp::NodeOptions & options)
: Node("target_manager", options)
{
  ManagerParams params{
    declare_parameter("confidence_decay_tau_s", 0.30),
    // Threshold equality uses the carried confidence's float32 precision.
    static_cast<double>(static_cast<float>(declare_parameter("assign_confidence", 0.60))),
    static_cast<double>(static_cast<float>(declare_parameter("clear_confidence", 0.40))),
    declare_parameter("drift_tolerance_m", 0.100),
    declare_parameter("drift_horizon_s", 0.200),
    declare_parameter("refresh_period_s", 0.200)};
  manager_ = std::make_unique<TargetManager>(params);
  const auto input = declare_parameter("input_topic", std::string("/cv/tracker/target_state"));
  const auto output = declare_parameter("output_topic", std::string("/cv/target_state"));
  const auto ack = declare_parameter("state_ack_topic", std::string(""));
  const auto consumer_ack = declare_parameter("consumer_ack_topic", std::string(""));
  const auto none = declare_parameter("target_state_none_topic", std::string(""));
  pub_ = create_publisher<Message>(output, 10);
  send_ = [this](const Message & m) {pub_->publish(m);};
  if (!ack.empty()) {
    ack_pub_ = create_publisher<std_msgs::msg::Header>(ack, 100);
    ack_ = [this](const std_msgs::msg::Header & h) {ack_pub_->publish(h);};
  }
  if (!consumer_ack.empty()) {
    consumer_ack_sub_ = create_subscription<std_msgs::msg::Header>(
      consumer_ack, 100, [this](std_msgs::msg::Header::ConstSharedPtr) {
        if (consumed_ < emitted_) {++consumed_;}
        flush_acks();
      });
  }
  if (!none.empty()) {
    none_pub_ = create_publisher<std_msgs::msg::Header>(none, 100);
    none_ = [this](const std_msgs::msg::Header & h) {none_pub_->publish(h);};
  }
  sub_ = create_subscription<Message>(input, 10, [this](Message::ConstSharedPtr m) {consume(m);});
  auto start_timer = [this]() {
      timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(5), [this]() {tick();});
    };
  if (declare_parameter("deterministic_epoch_zero", false)) {
    if (!get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("deterministic_epoch_zero requires use_sim_time");
    }
    epoch_ready_pub_ = create_publisher<std_msgs::msg::Header>(
      "~/epoch_zero_ready", rclcpp::QoS(1).reliable().transient_local());
    epoch_clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(), [this, start_timer](rosgraph_msgs::msg::Clock::ConstSharedPtr m) {
        if (epoch_failed_ || timer_) {return;}
        if (m->clock.sec != 0 || m->clock.nanosec != 0 || now().nanoseconds() != 0) {
          epoch_failed_ = true;
          RCLCPP_ERROR(get_logger(), "target_manager missed epoch zero");
          return;
        }
        start_timer();
        if (now().nanoseconds() != 0) {
          timer_->cancel(); epoch_failed_ = true;
          RCLCPP_ERROR(get_logger(), "clock advanced while initializing target_manager");
          return;
        }
        epoch_ready_pub_->publish(std_msgs::msg::Header{});
      });
  } else {
    start_timer();
  }
}

void TargetManagerNode::consume(Message::ConstSharedPtr msg)
{
  candidate_ = msg;
  ManagedModel m;
  auto seconds = [](const builtin_interfaces::msg::Time & t) {
      return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
    };
  auto vector = [](const auto & p) {return Eigen::Vector3d(p.x, p.y, p.z);};
  m.state_s = seconds(msg->header.stamp);
  m.detection_s = seconds(msg->detection_stamp);
  m.confidence = msg->confidence;
  m.track_id = msg->robot_track_id;
  m.center = vector(msg->center);
  m.velocity = vector(msg->velocity);
  m.acceleration = vector(msg->acceleration);
  m.panel = vector(msg->panel);
  m.yaw = msg->yaw;
  m.yaw_rate = msg->yaw_rate;
  m.yaw_rate_variance = msg->yaw_rate_variance;
  std::copy(msg->radius.begin(), msg->radius.end(), m.radius.begin());
  std::copy(msg->z_offset.begin(), msg->z_offset.end(), m.z_offset.begin());
  std::copy(msg->variance.begin(), msg->variance.end(), m.variance.begin());
  m.selected = msg->selected_track;
  m.reset = msg->reset;
  m.usable = msg->valid && !msg->clear && msg->header.frame_id == "odom" &&
    msg->header.stamp.sec >= 0 && msg->header.stamp.nanosec < 1000000000U &&
    msg->detection_stamp.sec >= 0 && msg->detection_stamp.nanosec < 1000000000U;
  const auto at = decision_time();
  const auto event = manager_->consume(m, at.seconds());
  if (event == TargetEvent::Assign && manager_->clear_before_assign()) {
    emit(TargetEvent::Clear, at);
  }
  emit(event, at);
  const bool skipped = event == TargetEvent::None && static_cast<bool>(none_);
  if (!ack_ && !skipped) {return;}
  auto h = msg->header;
  // Fixture acknowledgments identify the driving input, including empty frames.
  h.stamp = (msg->input_stamp.sec != 0 || msg->input_stamp.nanosec != 0) ?
    msg->input_stamp : msg->detection_stamp;
  if (skipped) {none_(h);}
  if (ack_) {
    // One ack per input, after its sends; an ordered consumer may defer release.
    if (consumer_ack_sub_) {
      pending_acks_.emplace_back(emitted_, h);
      flush_acks();
    } else {ack_(h);}
  }
}

void TargetManagerNode::flush_acks()
{
  while (!pending_acks_.empty() && pending_acks_.front().first <= consumed_) {
    ack_(pending_acks_.front().second);
    pending_acks_.pop_front();
  }
}

void TargetManagerNode::tick()
{
  const auto at = decision_time();
  emit(manager_->tick(at.seconds()), at);
}

void TargetManagerNode::emit(TargetEvent event, const rclcpp::Time & at)
{
  if (event == TargetEvent::None) {return;}
  Message out;
  if (event == TargetEvent::Clear) {
    out.header.stamp = at;
    out.header.frame_id = "odom";
    out.robot_track_id = manager_->released_track();
    out.clear = true;
    out.selected_track = false;
  } else {
    // Forward the model and covariance together at their original true time.
    out = *candidate_;
    out.valid = true;
    out.clear = false;
    out.reset = false;
  }
  send_(out);
  if (consumer_ack_sub_) {++emitted_;}
  if (observe_) {observe_(event, out);}
}
}  // namespace thornbots_pkg
