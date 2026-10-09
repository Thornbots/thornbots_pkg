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
// /cv/target_state (armor model, odom) -> /cv/target (odom aim point, held by
// the MCB as we move, carrying its own fire decision), via plan_shot. Each
// cv_target_publish_rate_hz tick aims, and sets fire/delay_ms (at most
// fire_rate_hz) with the delay that times a spinning target's panel to the
// shot. With no target it patrols: sweeps, or faces the last hit off
// RefSysStatus, never firing. See README.md's ### point_to_cv_target Notes.
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
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "thornbots_pkg/point_to_cv_target_core.hpp"

namespace thornbots_pkg
{

// RefSysStatus.delta_angle_got_hit_in when not hit (HitRing::PLACEHOLDER_ANGLE).
constexpr double NOT_HIT = 123.0;

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
  void on_publish_tick();

  // The muzzle's newest odom (position, yaw), or nullopt.
  std::optional<std::pair<Vector3d, double>> gun_pose();
  std::optional<Vector3d> patrol_point_now(const rclcpp::Time & now);
  // (fire, delay_ms) for this tick's CVTarget. The MCB runs the delay from
  // receiving the frame, so aim and fire cross the wire as one frame. Rate
  // limited to fire_rate_hz; delay_ms is clamped to the uint16 field.
  virtual std::pair<bool, int> fire_decision(
    std::optional<double> delay_s, const rclcpp::Time & now);
  // The odom aim point, or nullopt if the newest target_state is stale or TF
  // fails (logged loudly, never silently): the caller sends no CVTarget.
  virtual std::optional<Aim> compute_aim_point();
  virtual rclcpp::Time clock_now() const {return get_clock()->now();}

  // Unset tick/ack functions mean the bench topic is off.
  std::function<void(const dji_serial_bridge::msg::CVTarget &)> send_target_;
  std::function<void(const std_msgs::msg::Header &)> send_tick_, send_ack_;

  std::string target_state_topic_, odom_frame_, root_frame_, muzzle_frame_;
  double target_timeout_s_, fire_confidence_threshold_, fire_rate_hz_;
  bool lead_enabled_;
  double firmware_latency_s_, gimbal_lag_s_, v_muzzle_;
  int tof_iterations_;
  double tick_s_, spin_enter_rad_s_, spin_exit_rad_s_;
  std::optional<double> chase_settle_s_;
  double chase_margin_s_;
  bool type_c_based_patrol_, turn_to_hit_;
  std::optional<Patrol> patrol_;
  double patrol_range_m_, patrol_pitch_down_rad_, hit_angle_sign_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::optional<rclcpp::Time> last_fire_time_;
  bool spinning_ = false;
  bool target_active_ = false;
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
