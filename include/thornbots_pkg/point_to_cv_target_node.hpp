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

// ROS wrapper for point_to_cv_target_core.hpp: target_tracker's target state
// -> a world-frame aim point.
//
// /cv/target_state (odom armor model) -> /cv/target (odom aim plus fire decision).
// Each publish tick aims and gates firmware hit turns by estimate availability.
// No target: idle looks without firing and permit firmware hit turns. See README.md.
//
// The protected members are seams for test/test_point_to_cv_target_node.cpp:
// the clock, the outgoing publishes and the aim/fire decisions.

#ifndef THORNBOTS_PKG__POINT_TO_CV_TARGET_NODE_HPP_
#define THORNBOTS_PKG__POINT_TO_CV_TARGET_NODE_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
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
  struct Aim
  {
    Vector3d pos;
    std::optional<double> fire_delay_s;
  };

  void on_target_state(dji_serial_bridge::msg::TargetState::ConstSharedPtr msg);
  void on_robot_pose(const dji_serial_bridge::msg::RobotPose & msg);
  void on_ref_sys(const dji_serial_bridge::msg::RefSysStatus & msg);
  void check_timeout();
  bool update_fire_lock();
  void on_publish_tick();

  // The muzzle's newest odom (position, yaw), or nullopt.
  std::optional<std::pair<Vector3d, double>> gun_pose();
  Vector3d idle_point_now(const rclcpp::Time & now);
  // (fire, delay_ms) for this tick's CVTarget. The MCB runs the delay from
  // receiving the frame, so aim and fire cross the wire as one frame.
  // Every eligible tick fires; delay_ms is clamped to the uint16 field.
  virtual std::pair<bool, int> fire_decision(std::optional<double> delay_s);
  // The odom aim point, or nullopt if the newest target_state is stale or TF
  // fails (logged loudly): the caller may publish a non-firing idle point.
  virtual std::optional<Aim> compute_aim_point();
  virtual rclcpp::Time clock_now() const {return get_clock()->now();}

  // Unset tick/ack functions mean the bench topic is off.
  std::function<void(const dji_serial_bridge::msg::CVTarget &)> send_target_;
  std::function<void(const std_msgs::msg::Header &)> send_tick_, send_ack_;

  std::string target_state_topic_, odom_frame_, root_frame_, muzzle_frame_;
  double target_timeout_s_, fire_max_state_age_s_, fire_confidence_threshold_;
  double fire_lock_s_, fire_lock_grace_s_, fire_stationary_speed_mps_;
  FireLock fire_lock_;
  bool lead_enabled_;
  double firmware_latency_s_, min_fire_delay_s_, gimbal_lag_s_, v_muzzle_;
  int tof_iterations_;
  double tick_s_, spin_enter_rad_s_, spin_exit_rad_s_;
  std::optional<double> chase_settle_s_;
  double chase_margin_s_, panel_switch_margin_rad_;
  bool type_c_based_patrol_;
  IdleLook idle_;
  std::string idle_direction_;
  bool blue_ = false;
  double zone_start_x_m_, zone_start_y_m_, idle_height_m_, idle_rate_rad_s_, idle_switch_rad_;
  std::optional<Vector3d> last_sent_point_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool spinning_ = false;
  bool target_active_ = false;
  std::optional<double> held_panel_yaw_;  // Shot::panel_yaw, for latest_state_'s track
  dji_serial_bridge::msg::TargetState::ConstSharedPtr latest_state_;
  Vector3d chassis_vel_root_ = Vector3d::Zero();  // from RobotPose, root frame
  LatencyStat latency_stat_;

private:
  rclcpp::Publisher<dji_serial_bridge::msg::CVTarget>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr tick_pub_, state_ack_pub_;
  rclcpp::Subscription<dji_serial_bridge::msg::TargetState>::SharedPtr target_state_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::RobotPose>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::RefSysStatus>::SharedPtr ref_sys_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_, publish_timer_;
};

}  // namespace thornbots_pkg

#endif  // THORNBOTS_PKG__POINT_TO_CV_TARGET_NODE_HPP_
