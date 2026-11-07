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

// ROS wrapper for point_to_cv_target_core.hpp: idle looks while no target is
// tracked. The MCB plans aim and fire from the relayed /cv/target_state.
//
// /cv/target_state sets target-active (until target_timeout_s after its newest
// detection). Each publish tick sends an odom-frame idle CVTarget only while
// no target is active. See README.md.
//
// The protected members are seams for test/test_point_to_cv_target_node.cpp:
// the clock and the outgoing publishes.

#ifndef THORNBOTS_PKG__POINT_TO_CV_TARGET_NODE_HPP_
#define THORNBOTS_PKG__POINT_TO_CV_TARGET_NODE_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "thornbots_pkg/point_to_cv_target_core.hpp"

namespace thornbots_pkg
{

class PointToCvTarget : public rclcpp::Node
{
public:
  explicit PointToCvTarget(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void on_target_state(dji_serial_bridge::msg::TargetState::ConstSharedPtr msg);
  void on_ref_sys(const dji_serial_bridge::msg::RefSysStatus & msg);
  void check_timeout();
  void on_publish_tick();

  // The muzzle's odom (position, yaw), or nullopt: its newest TF, or with
  // allow_latest_tf:=false the one stamped tf_lookup_lag_s before now.
  std::optional<std::pair<Vector3d, double>> gun_pose();
  // Throws tf2::TransformException; see allow_latest_tf_.
  geometry_msgs::msg::TransformStamped lookup_tf(
    const std::string & target, const std::string & source, double lag_s);
  void count_latest_tf_fallback();
  Vector3d idle_point_now(const rclcpp::Time & now);
  virtual rclcpp::Time clock_now() const {return get_clock()->now();}

  // Unset tick/ack functions mean the bench topic is off.
  std::function<void(const dji_serial_bridge::msg::CVTarget &)> send_target_;
  std::function<void(const std_msgs::msg::Header &)> send_tick_, send_ack_;

  std::string target_state_topic_, odom_frame_, navigation_frame_, muzzle_frame_;
  double target_timeout_s_;
  IdleLook idle_;
  std::string idle_direction_;
  bool blue_ = false;
  double red_start_x_, red_start_y_, blue_start_x_, blue_start_y_;
  std::optional<Vector3d> idle_zone_;
  double idle_height_m_, idle_rate_rad_s_, idle_switch_rad_;
  std::optional<Vector3d> last_sent_point_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Robot default true: take the newest TF. Simulation lockstep sets false so
  // a lookup is at a stamp, never at whatever TF happened to arrive last.
  bool allow_latest_tf_ = true;
  double tf_lookup_lag_s_ = 0.0, nav_tf_lookup_lag_s_ = 0.0;
  uint32_t latest_tf_fallbacks_ = 0;
  std::set<std::string> tf_seen_;  // frame pairs that have resolved once

  bool target_active_ = false;
  dji_serial_bridge::msg::TargetState::ConstSharedPtr latest_state_;
  LatencyStat latency_stat_;

private:
  rclcpp::Publisher<dji_serial_bridge::msg::CVTarget>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr tick_pub_, state_ack_pub_, none_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr fallback_pub_;
  rclcpp::Subscription<dji_serial_bridge::msg::TargetState>::SharedPtr target_state_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::RefSysStatus>::SharedPtr ref_sys_sub_;
  void start_timers(double publish_rate_hz);
  rclcpp::TimerBase::SharedPtr watchdog_timer_, publish_timer_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr epoch_clock_sub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr epoch_ready_pub_;
  bool epoch_failed_ = false;
};

}  // namespace thornbots_pkg

#endif  // THORNBOTS_PKG__POINT_TO_CV_TARGET_NODE_HPP_
