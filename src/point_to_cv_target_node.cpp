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

#include "thornbots_pkg/point_to_cv_target_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "tf2/exceptions.h"

namespace thornbots_pkg
{

using dji_serial_bridge::msg::CVTarget;
using dji_serial_bridge::msg::TargetState;
using Eigen::Matrix3d;

namespace
{
Matrix3d quat_to_rot(double x, double y, double z, double w)
{
  Matrix3d R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
  return R;
}

double seconds(const rclcpp::Duration & d) {return static_cast<double>(d.nanoseconds()) / 1e9;}
double seconds(const rclcpp::Time & t) {return static_cast<double>(t.nanoseconds()) / 1e9;}
}  // namespace

PointToCvTarget::PointToCvTarget(const rclcpp::NodeOptions & options)
: Node("point_to_cv_target", options)
{
  target_state_topic_ = declare_parameter("target_state_topic", std::string("/cv/target_state"));
  const auto output_topic = declare_parameter("output_topic", std::string("/cv/target"));
  target_timeout_s_ = declare_parameter("target_timeout_s", 0.5);
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
  const double publish_rate_hz = declare_parameter("cv_target_publish_rate_hz", 40.0);
  idle_direction_ = declare_parameter("idle_direction", std::string("zone"));
  const auto idle_zone = declare_parameter("idle_zone", std::vector<double>{});
  if (!idle_zone.empty()) {
    if (idle_zone.size() != 2 || !std::isfinite(idle_zone[0]) || !std::isfinite(idle_zone[1])) {
      throw std::invalid_argument("idle_zone must be empty or two finite field coordinates");
    }
    idle_zone_ = Vector3d(idle_zone[0], idle_zone[1], 0.0);
  }
  navigation_frame_ = declare_parameter("navigation_frame", std::string("nav_odom"));
  if (idle_direction_ != "zone" && idle_direction_ != "hold") {
    throw std::invalid_argument("idle_direction must be zone or hold");
  }
  // Field coordinates; transform into the continuous raw aim frame before use.
  red_start_x_ = declare_parameter<double>("red_start_x");
  red_start_y_ = declare_parameter<double>("red_start_y");
  blue_start_x_ = declare_parameter<double>("blue_start_x");
  blue_start_y_ = declare_parameter<double>("blue_start_y");
  if (!std::isfinite(red_start_x_) || !std::isfinite(red_start_y_) ||
    !std::isfinite(blue_start_x_) || !std::isfinite(blue_start_y_))
  {
    throw std::invalid_argument("Team starts must be finite field coordinates");
  }
  idle_height_m_ = declare_parameter("idle_height_m", 0.15);
  idle_rate_rad_s_ = declare_parameter("idle_rate_rad_s", 0.5);
  idle_switch_rad_ = declare_parameter("idle_switch_rad", M_PI / 6.0);
  if (!std::isfinite(idle_rate_rad_s_) || idle_rate_rad_s_ <= 0.0 ||
    !std::isfinite(idle_switch_rad_) || idle_switch_rad_ < 0.0)
  {
    throw std::invalid_argument("idle rate/switch must be finite and nonnegative");
  }
  muzzle_frame_ = declare_parameter("muzzle_frame", std::string("muzzle"));
  // Bench only: a Header per publish tick, sent or not, for lockstep.
  const auto tick_topic = declare_parameter("tick_topic", std::string(""));
  const auto state_ack_topic = declare_parameter("state_ack_topic", std::string(""));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // /tf shares this node's executor, so every lookup below is non-blocking: a
  // timeout wait in a callback starves /tf. See README.md.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  // Sensor-like, best-effort traffic: a dropped idle point is far less
  // harmful than blocking on a slow/disconnected subscriber, and this matches
  // mcb_relay's cv_target subscriber QoS.
  pub_ = create_publisher<CVTarget>(output_topic, rclcpp::SensorDataQoS());
  send_target_ = [this](const CVTarget & m) {pub_->publish(m);};
  target_state_sub_ = create_subscription<TargetState>(
    target_state_topic_, 10, [this](TargetState::ConstSharedPtr m) {on_target_state(m);});
  if (!tick_topic.empty()) {
    tick_pub_ = create_publisher<std_msgs::msg::Header>(tick_topic, 10);
    send_tick_ = [this](const std_msgs::msg::Header & h) {tick_pub_->publish(h);};
  }
  if (!state_ack_topic.empty()) {
    state_ack_pub_ = create_publisher<std_msgs::msg::Header>(state_ack_topic, 100);
    send_ack_ = [this](const std_msgs::msg::Header & h) {state_ack_pub_->publish(h);};
  }
  const auto ref_sys_topic = declare_parameter(
    "ref_sys_topic", std::string("/dji_serial_bridge/ref_sys"));
  ref_sys_sub_ = create_subscription<dji_serial_bridge::msg::RefSysStatus>(
    ref_sys_topic, rclcpp::SensorDataQoS(),
    [this](dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr m) {on_ref_sys(*m);});
  const bool epoch_zero = declare_parameter("deterministic_epoch_zero", false);
  if (epoch_zero) {
    if (!get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("deterministic_epoch_zero requires use_sim_time");
    }
    epoch_ready_pub_ = create_publisher<std_msgs::msg::Header>(
      "~/epoch_zero_ready", rclcpp::QoS(1).reliable().transient_local());
    epoch_clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(), [this, publish_rate_hz](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        if (epoch_failed_ || publish_timer_) {return;}
        if (msg->clock.sec != 0 || msg->clock.nanosec != 0 || now().nanoseconds() != 0) {
          epoch_failed_ = true;
          RCLCPP_ERROR(get_logger(), "point_to_cv_target missed epoch zero");
          return;
        }
        start_timers(publish_rate_hz);
        if (now().nanoseconds() != 0) {
          watchdog_timer_->cancel();
          publish_timer_->cancel();
          epoch_failed_ = true;
          RCLCPP_ERROR(get_logger(), "clock advanced while initializing point_to_cv_target");
          return;
        }
        std_msgs::msg::Header ready;
        epoch_ready_pub_->publish(ready);
      });
  } else {
    start_timers(publish_rate_hz);
  }
  RCLCPP_INFO(get_logger(), "point_to_cv_target ready: idle=%s, target timeout %.3fs",
    idle_direction_.c_str(), target_timeout_s_);
}

void PointToCvTarget::start_timers(double publish_rate_hz)
{
  watchdog_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {check_timeout();});
  publish_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate_hz),
    [this]() {on_publish_tick();});
}

void PointToCvTarget::on_target_state(TargetState::ConstSharedPtr msg)
{
  const rclcpp::Time now = clock_now();
  // Account for the held state's age before replacing it, even between ticks.
  check_timeout();
  if (!latest_state_ || msg->robot_track_id != latest_state_->robot_track_id) {
    RCLCPP_INFO(get_logger(), "target_state: tracking robot %u", msg->robot_track_id);
  }
  latest_state_ = msg;
  target_active_ = true;
  const double latency_s = seconds(now - rclcpp::Time(msg->header.stamp, now.get_clock_type()));
  if (latency_s >= 0.0) {
    latency_stat_.add(latency_s);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "target_state age on arrival: %.1f ms now, %.1f ms mean over %d samples",
      latency_s * 1e3, latency_stat_.mean() * 1e3, latency_stat_.count());
  }

  if (send_ack_) {
    send_ack_(msg->header);
  }
}

std::optional<std::pair<Vector3d, double>> PointToCvTarget::gun_pose()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(odom_frame_, muzzle_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "TF lookup %s<-%s failed: %s", odom_frame_.c_str(),
      muzzle_frame_.c_str(), ex.what());
    return std::nullopt;
  }
  const auto & t = tf.transform.translation;
  const auto & q = tf.transform.rotation;
  const Matrix3d R = quat_to_rot(q.x, q.y, q.z, q.w);
  return std::make_pair(Vector3d(t.x, t.y, t.z), std::atan2(R(1, 0), R(0, 0)));
}

void PointToCvTarget::on_ref_sys(const dji_serial_bridge::msg::RefSysStatus & msg)
{
  if (blue_ != msg.is_on_blue_team) {
    idle_.reset();
    last_sent_point_.reset();
  }
  blue_ = msg.is_on_blue_team;
  if (std::isfinite(msg.delta_angle_got_hit_in) &&
    std::abs(msg.delta_angle_got_hit_in) <= static_cast<float>(M_PI))
  {
    const auto now = clock_now();
    idle_.hit(seconds(rclcpp::Time(msg.header.stamp, now.get_clock_type())));
  }
}

Vector3d PointToCvTarget::idle_point_now(const rclcpp::Time & now)
{
  const auto gun = gun_pose();
  if (!gun) {return last_sent_point_.value_or(Vector3d::Zero());}
  Vector3d zone = gun->first + Vector3d(3.0 * std::cos(gun->second),
    3.0 * std::sin(gun->second), 0.0);
  try {
    const auto tf = tf_buffer_->lookupTransform(odom_frame_, navigation_frame_, tf2::TimePointZero);
    const auto & t = tf.transform.translation;
    const auto & q = tf.transform.rotation;
    zone = quat_to_rot(q.x, q.y, q.z, q.w) *
      idle_zone_.value_or(enemy_zone(
        blue_, red_start_x_, red_start_y_, blue_start_x_, blue_start_y_, idle_height_m_)) +
      Vector3d(t.x, t.y, t.z);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Idle zone waits for nav_odom->odom: %s", ex.what());
  }
  const auto source = idle_.source(seconds(now), target_timeout_s_, idle_direction_ == "hold");
  if (source == IdleLook::Source::Hit || source == IdleLook::Source::Hold) {
    idle_.reset();
    return {gun->first.x() + 3.0 * std::cos(gun->second),
      gun->first.y() + 3.0 * std::sin(gun->second), idle_height_m_};
  }
  Vector3d desired = source == IdleLook::Source::LastEnemy ? idle_.last_enemy() : zone;
  desired.z() = idle_height_m_;
  return idle_.point(seconds(now), gun->first, gun->second, desired,
    idle_rate_rad_s_, idle_switch_rad_);
}

void PointToCvTarget::check_timeout()
{
  if (!target_active_) {
    return;
  }
  const rclcpp::Time now = clock_now();
  const double age_s = seconds(
    now - rclcpp::Time(latest_state_->detection_stamp, now.get_clock_type()));
  if (age_s <= target_timeout_s_) {
    return;
  }

  target_active_ = false;
  RCLCPP_INFO(
    get_logger(), "Newest '%s' is %.2f s old - no target until the next one arrives.",
    target_state_topic_.c_str(), age_s);
}

void PointToCvTarget::on_publish_tick()
{
  check_timeout();
  const rclcpp::Time now = clock_now();
  std::string out_frame;  // empty means no point was published
  if (target_active_) {
    // The MCB aims from the relayed state; remember where the enemy was.
    const auto & s = *latest_state_;
    const auto & p = s.valid ? s.center : s.panel;
    idle_.target_seen(
      seconds(rclcpp::Time(s.detection_stamp, now.get_clock_type())), Vector3d(p.x, p.y, p.z));
  } else {
    const Vector3d point = idle_point_now(now);
    CVTarget out;
    out.header.stamp = now;
    out.header.frame_id = odom_frame_;
    out.x = static_cast<float>(point.x());
    out.y = static_cast<float>(point.y());
    out.z = static_cast<float>(point.z());
    out.turn_to_hit = true;
    last_sent_point_ = point;
    out_frame = out.header.frame_id;
    send_target_(out);
  }
  if (send_tick_) {
    // Empty frame means no point was published; otherwise name its output frame.
    std_msgs::msg::Header tick;
    tick.stamp = now;
    tick.frame_id = out_frame;
    send_tick_(tick);
  }
}

}  // namespace thornbots_pkg
