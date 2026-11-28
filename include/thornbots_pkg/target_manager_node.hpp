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

#ifndef THORNBOTS_PKG__TARGET_MANAGER_NODE_HPP_
#define THORNBOTS_PKG__TARGET_MANAGER_NODE_HPP_

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "dji_serial_bridge/msg/target_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/header.hpp"
#include "thornbots_pkg/target_manager_core.hpp"

namespace thornbots_pkg
{
class TargetManagerNode : public rclcpp::Node
{
public:
  explicit TargetManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  using Message = dji_serial_bridge::msg::TargetState;
  void consume(Message::ConstSharedPtr msg);
  void tick();
  virtual rclcpp::Time decision_time() const {return now();}
  std::function<void(const Message &)> send_;
  std::function<void(const std_msgs::msg::Header &)> ack_, none_;
  std::function<void(TargetEvent, const Message &)> observe_;

private:
  void emit(TargetEvent event, const rclcpp::Time & at);
  // Evaluates every decision tick on the period grid up to `until`, at its grid time.
  void run_ticks(const rclcpp::Time & until);
  void finish_tick(const rclcpp::Time & at);  // advance the grid; ack it in lockstep
  void flush_acks();
  std::unique_ptr<TargetManager> manager_;
  Message::ConstSharedPtr candidate_;
  rclcpp::Publisher<Message>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ack_pub_, none_pub_, tick_ack_pub_;
  rclcpp::Subscription<Message>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr consumer_ack_sub_;
  struct PendingAck
  {
    uint64_t after_emitted;
    std_msgs::msg::Header header;
    bool tick;  // tick_ack_topic, else the input's state ack
  };
  std::deque<PendingAck> pending_acks_;
  uint64_t emitted_ = 0, consumed_ = 0;
  int64_t tick_period_ns_ = 5000000;
  std::optional<int64_t> next_tick_ns_;  // next grid time not yet evaluated
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr epoch_clock_sub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr epoch_ready_pub_;
  bool epoch_failed_ = false;
};
}  // namespace thornbots_pkg
#endif  // THORNBOTS_PKG__TARGET_MANAGER_NODE_HPP_
