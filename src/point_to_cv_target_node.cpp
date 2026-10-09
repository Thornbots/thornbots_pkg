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
#include <sstream>
#include <string>

#include "tf2/exceptions.h"

namespace thornbots_pkg
{

using dji_serial_bridge::msg::CVTarget;
using dji_serial_bridge::msg::RefSysStatus;
using dji_serial_bridge::msg::RobotPose;
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
  const auto robot_pose_topic = declare_parameter(
    "robot_pose_topic", std::string("/dji_serial_bridge/pose"));
  const auto output_topic = declare_parameter("output_topic", std::string("/cv/target"));
  target_timeout_s_ = declare_parameter("target_timeout_s", 0.5);
  fire_confidence_threshold_ = declare_parameter("fire_confidence_threshold", 0.5);
  fire_rate_hz_ = declare_parameter("fire_rate_hz", 2.0);
  root_frame_ = declare_parameter("root_frame", std::string("root"));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
  lead_enabled_ = declare_parameter("lead_enabled", true);
  // Fire decision to projectile exit; times the fire against the spin.
  firmware_latency_s_ = declare_parameter("firmware_latency_s", 0.05);
  // Setpoint to gimbal pointing there, on a moving setpoint; sets how far
  // ahead the aim point leads. See README.md.
  gimbal_lag_s_ = declare_parameter("gimbal_lag_s", 0.05);
  v_muzzle_ = declare_parameter("v_muzzle", 25.0);
  tof_iterations_ = static_cast<int>(declare_parameter("tof_iterations", 3));
  const double publish_rate_hz = declare_parameter("cv_target_publish_rate_hz", 40.0);
  tick_s_ = 1.0 / publish_rate_hz;
  // Spin mode (shotgating, or chase) above enter, back to panel aim below
  // exit, in |yaw_rate| rad/s.
  spin_enter_rad_s_ = declare_parameter("spin_enter_rad_s", 3.0);
  spin_exit_rad_s_ = declare_parameter("spin_exit_rad_s", 2.0);
  // Spin mode, >= 0 (default): chase the facing panel, firing on any tick whose
  // panel has faced us this long and will for chase_margin_s more; both cover
  // the gimbal's jump between panels. < 0: shotgating, hold the center line
  // and time the fire. README.md.
  const double chase_settle_s = declare_parameter("chase_settle_s", 0.0);
  if (chase_settle_s >= 0.0) {
    chase_settle_s_ = chase_settle_s;
  }
  chase_margin_s_ = declare_parameter("chase_margin_s", 0.0);
  // Sent with every aim point: whether the MCB may patrol on its own (off: the
  // Jetson owns where it looks), and turn toward a hit (on).
  type_c_based_patrol_ = declare_parameter("type_c_based_patrol", false);
  turn_to_hit_ = declare_parameter("turn_to_hit", true);
  // Jetson patrol with no target, mirroring the MCB's own (README.md): sweep at
  // patrol_rate_rad_s (MCB's -0.002 rad per 1 ms cycle), a point
  // patrol_range_m out and patrol_pitch_down_rad below level.
  const bool patrol_enabled = declare_parameter("patrol_enabled", true);
  const double patrol_after_s = declare_parameter("patrol_after_s", 0.2);
  const double patrol_rate_rad_s = declare_parameter("patrol_rate_rad_s", -2.0);
  patrol_range_m_ = declare_parameter("patrol_range_m", 3.0);
  patrol_pitch_down_rad_ = declare_parameter("patrol_pitch_down_rad", 0.05);
  // Face a hit for hit_turn_s: hit yaw = gun yaw + sign * delta angle.
  const double hit_turn_s = declare_parameter("hit_turn_s", 0.5);
  hit_angle_sign_ = declare_parameter("hit_angle_sign", -1.0);
  muzzle_frame_ = declare_parameter("muzzle_frame", std::string("muzzle"));
  const auto ref_sys_topic = declare_parameter(
    "ref_sys_topic", std::string("/dji_serial_bridge/ref_sys"));
  // Bench only: a Header per publish tick, sent or not, for lockstep.
  const auto tick_topic = declare_parameter("tick_topic", std::string(""));
  const auto state_ack_topic = declare_parameter("state_ack_topic", std::string(""));
  if (patrol_enabled) {
    patrol_.emplace(patrol_rate_rad_s, patrol_after_s, hit_turn_s);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // /tf shares this node's executor, so every lookup below is non-blocking: a
  // timeout wait in a callback starves /tf. See README.md.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  // Sensor-like, best-effort traffic: a dropped target update is far less
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
  robot_pose_sub_ = create_subscription<RobotPose>(
    robot_pose_topic, rclcpp::SensorDataQoS(),
    [this](RobotPose::ConstSharedPtr m) {on_robot_pose(*m);});
  if (patrol_ && turn_to_hit_) {
    // Matches dji_serial_bridge_node's ~/ref_sys SensorDataQoS publisher.
    ref_sys_sub_ = create_subscription<RefSysStatus>(
      ref_sys_topic, rclcpp::SensorDataQoS(),
      [this](RefSysStatus::ConstSharedPtr m) {on_ref_sys(*m);});
  }

  watchdog_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {check_timeout();});
  publish_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate_hz),
    [this]() {on_publish_tick();});

  std::ostringstream patrol_text;
  if (patrol_) {
    patrol_text << patrol_->rate_rad_s() << " rad/s, turn_to_hit="
                << (turn_to_hit_ ? "True" : "False");
  } else {
    patrol_text << "off";
  }
  const std::string patrol_str = patrol_text.str();
  RCLCPP_INFO(
    get_logger(),
    "point_to_cv_target ready\n  %s + %s\n  -> %s (CVTarget, %s frame, aim + fire, @ %.1fHz)\n"
    "  lead_enabled=%s v_muzzle=%g firmware_latency_s=%g gimbal_lag_s=%g\n"
    "  target_timeout_s=%.2f\n  fire <= %.2fHz, spin-timed above %g rad/s, confidence >= %g\n"
    "  patrol: %s",
    target_state_topic_.c_str(), robot_pose_topic.c_str(), output_topic.c_str(),
    odom_frame_.c_str(), publish_rate_hz, lead_enabled_ ? "True" : "False", v_muzzle_,
    firmware_latency_s_, gimbal_lag_s_, target_timeout_s_, fire_rate_hz_, spin_enter_rad_s_,
    fire_confidence_threshold_,
    patrol_str.c_str());
}

void PointToCvTarget::on_target_state(TargetState::ConstSharedPtr msg)
{
  if (!latest_state_ || msg->robot_track_id != latest_state_->robot_track_id) {
    RCLCPP_INFO(get_logger(), "target_state: tracking robot %u", msg->robot_track_id);
  }
  latest_state_ = msg;
  target_active_ = true;
  const rclcpp::Time now = clock_now();
  const double latency_s = seconds(now - rclcpp::Time(msg->header.stamp, now.get_clock_type()));
  if (latency_s >= 0.0) {
    latency_stat_.add(latency_s);
    // Diagnostic only -- the lead solve uses each tick's own state age
    // instead, which is larger and varies. See README.md.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "target_state age on arrival: %.1f ms now, %.1f ms mean over %d samples",
      latency_s * 1e3, latency_stat_.mean() * 1e3, latency_stat_.count());
  }

  if (send_ack_) {
    send_ack_(msg->header);
  }
}

void PointToCvTarget::on_robot_pose(const RobotPose & msg)
{
  chassis_vel_root_ = Vector3d(msg.vel_x, msg.vel_y, 0.0);
}

void PointToCvTarget::on_ref_sys(const RefSysStatus & msg)
{
  const double delta = msg.delta_angle_got_hit_in;
  if (delta == NOT_HIT || !(std::abs(delta) <= M_PI + 1e-3)) {
    return;
  }
  const auto gun = gun_pose();
  if (!gun) {
    return;
  }
  const double hit_yaw = gun->second + hit_angle_sign_ * delta;
  patrol_->hit(hit_yaw, seconds(clock_now()));
  RCLCPP_INFO(
    get_logger(), "hit at %+.2f rad from the gun: facing odom yaw %+.2f", delta, hit_yaw);
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

std::optional<Vector3d> PointToCvTarget::patrol_point_now(const rclcpp::Time & now)
{
  const auto gun = gun_pose();
  if (!gun) {
    return std::nullopt;
  }
  const auto yaw = patrol_->step(seconds(now), gun->second);
  if (!yaw) {
    return std::nullopt;
  }
  return patrol_point(gun->first, *yaw, patrol_range_m_, patrol_pitch_down_rad_);
}

std::pair<bool, int> PointToCvTarget::fire_decision(
  std::optional<double> delay_s, const rclcpp::Time & now)
{
  if (!delay_s || fire_rate_hz_ <= 0.0) {
    return {false, 0};
  }
  if (latest_state_->confidence < fire_confidence_threshold_) {
    return {false, 0};
  }
  if (last_fire_time_ && seconds(now - *last_fire_time_) < 1.0 / fire_rate_hz_) {
    return {false, 0};
  }
  last_fire_time_ = now;
  // Python round() is half-to-even, as is nearbyint in the default mode.
  const double ms = std::nearbyint(*delay_s * 1000.0);
  return {true, static_cast<int>(std::max(0.0, std::min(65535.0, ms)))};
}

void PointToCvTarget::check_timeout()
{
  if (!target_active_) {
    return;
  }
  const rclcpp::Time now = clock_now();
  const double age_s = seconds(
    now - rclcpp::Time(latest_state_->header.stamp, now.get_clock_type()));
  if (age_s <= target_timeout_s_) {
    return;
  }

  target_active_ = false;
  RCLCPP_INFO(
    get_logger(), "Newest '%s' is %.2f s old - publishing no CVTarget until the next one arrives.",
    target_state_topic_.c_str(), age_s);
}

void PointToCvTarget::on_publish_tick()
{
  const rclcpp::Time now = clock_now();
  std::optional<Aim> aim;
  if (target_active_) {
    aim = compute_aim_point();
  }
  if (aim && patrol_) {
    patrol_->target_seen(seconds(now));
  } else if (!aim && patrol_) {
    const auto point = patrol_point_now(now);
    if (point) {
      aim = Aim{*point, std::nullopt};
    }
  }
  std::string out_frame;  // empty means no aim was published
  if (aim) {  // no target and no patrol: send nothing, the MCB holds still
    CVTarget out;
    out.header.stamp = now;
    out.header.frame_id = odom_frame_;
    out.x = static_cast<float>(aim->pos.x());
    out.y = static_cast<float>(aim->pos.y());
    out.z = static_cast<float>(aim->pos.z());
    const auto fire = fire_decision(aim->fire_delay_s, now);
    out.fire = fire.first;
    out.delay_ms = static_cast<uint16_t>(fire.second);
    out.type_c_based_patrol = type_c_based_patrol_;
    out.turn_to_hit = turn_to_hit_;
    out_frame = out.header.frame_id;
    send_target_(out);
  }
  if (send_tick_) {
    // Empty frame means no aim was published; otherwise name its output frame.
    std_msgs::msg::Header tick;
    tick.stamp = now;
    tick.frame_id = out_frame;
    send_tick_(tick);
  }
}

std::optional<PointToCvTarget::Aim> PointToCvTarget::compute_aim_point()
{
  const auto state = latest_state_;
  const rclcpp::Time now = clock_now();
  const rclcpp::Time state_stamp(state->header.stamp, now.get_clock_type());
  const double state_age_s = seconds(now - state_stamp);

  if (state_age_s > target_timeout_s_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Newest '%s' is %.2f s old (> target_timeout_s=%.2f) -- not aiming on it.",
      target_state_topic_.c_str(), state_age_s, target_timeout_s_);
    return std::nullopt;
  }

  if (!state->valid) {
    // Unconverged: aim at the measured panel, no lead, no fire.
    spinning_ = false;
    return Aim{Vector3d(state->panel.x, state->panel.y, state->panel.z), std::nullopt};
  }

  // Our pose at the state's stamp; if TF hasn't reached it yet, the newest one,
  // carried forward below.
  geometry_msgs::msg::TransformStamped tf_shooter;
  try {
    if (tf_buffer_->canTransform(odom_frame_, root_frame_, state_stamp)) {
      tf_shooter = tf_buffer_->lookupTransform(odom_frame_, root_frame_, state_stamp);
    } else {
      tf_shooter = tf_buffer_->lookupTransform(odom_frame_, root_frame_, tf2::TimePointZero);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "TF lookup %s<-%s failed: %s", odom_frame_.c_str(),
      root_frame_.c_str(), ex.what());
    return std::nullopt;
  }

  const auto & st = tf_shooter.transform.translation;
  const auto & sq = tf_shooter.transform.rotation;
  const Matrix3d shooter_R = quat_to_rot(sq.x, sq.y, sq.z, sq.w);
  const Vector3d shooter_vel_odom = shooter_R * chassis_vel_root_;
  // plan_shot wants us at the state's stamp, not the transform's.
  const double tf_to_state_s = seconds(
    state_stamp - rclcpp::Time(tf_shooter.header.stamp, now.get_clock_type()));
  const Vector3d shooter_pos_odom =
    Vector3d(st.x, st.y, st.z) + shooter_vel_odom * tf_to_state_s;

  const double threshold = spinning_ ? spin_exit_rad_s_ : spin_enter_rad_s_;
  spinning_ = std::abs(state->yaw_rate) > threshold;
  const ArmorState armor = {
    state->center.x, state->center.y, state->center.z, state->velocity.x, state->velocity.y,
    state->velocity.z, state->yaw, state->yaw_rate};
  ShotOptions options;
  options.gimbal_lag_s = gimbal_lag_s_;
  options.firmware_latency_s = firmware_latency_s_;
  options.lead = lead_enabled_;
  options.iterations = tof_iterations_;
  options.shooter_vel = shooter_vel_odom;
  options.chase_settle_s = chase_settle_s_;
  options.chase_margin_s = chase_margin_s_;
  options.accel = Vector3d(state->acceleration.x, state->acceleration.y, state->acceleration.z);
  // Age of THIS state at THIS tick, not latency_stat.mean: publishing runs on
  // its own timer over a cached state. See README.md.
  const Shot shot = plan_shot(
    armor, {state->radius[0], state->radius[1]}, {state->z_offset[0], state->z_offset[1]},
    state_age_s, shooter_pos_odom, v_muzzle_, spinning_, tick_s_, options);
  return Aim{shot.aim_pos, shot.delay_s};
}

}  // namespace thornbots_pkg
