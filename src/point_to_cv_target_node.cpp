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

#include "tf2/exceptions.h"

namespace thornbots_pkg
{

using dji_serial_bridge::msg::CVTarget;
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
  fire_max_state_age_s_ = declare_parameter("fire_max_state_age_s", 0.15);
  fire_confidence_threshold_ = declare_parameter("fire_confidence_threshold", 0.5);
  root_frame_ = declare_parameter("root_frame", std::string("root"));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
  lead_enabled_ = declare_parameter("lead_enabled", true);
  // MCB indexer request to projectile exit (timed on the robot), and the
  // shortest exit delay sent; the larger times the fire against the spin.
  firmware_latency_s_ = declare_parameter("firmware_latency_s", 0.020);
  min_fire_delay_s_ = declare_parameter("min_fire_delay_s", 0.025);
  // Setpoint to gimbal pointing there, on a moving setpoint; sets how far
  // ahead the aim point leads. See README.md.
  gimbal_lag_s_ = declare_parameter("gimbal_lag_s", 0.05);
  v_muzzle_ = declare_parameter("v_muzzle", 22.0);
  tof_iterations_ = static_cast<int>(declare_parameter("tof_iterations", 3));
  const double publish_rate_hz = declare_parameter("cv_target_publish_rate_hz", 40.0);
  tick_s_ = 1.0 / publish_rate_hz;
  // Spin mode (shotgating, or chase) above enter, back to panel aim below
  // exit, in |yaw_rate| rad/s.
  spin_enter_rad_s_ = declare_parameter("spin_enter_rad_s", 3.0);
  spin_exit_rad_s_ = declare_parameter("spin_exit_rad_s", 2.0);
  // Spin mode, >= 0: chase the facing panel, firing on any tick whose
  // panel has faced us this long and will for chase_margin_s more; both cover
  // the gimbal's jump between panels. < 0: shotgating, hold the center line
  // and time the fire. README.md.
  const double chase_settle_s = declare_parameter("chase_settle_s", -1.0);
  if (chase_settle_s >= 0.0) {
    chase_settle_s_ = chase_settle_s;
  }
  chase_margin_s_ = declare_parameter("chase_margin_s", 0.0);
  // Not spinning: keep leading a panel until another faces us this much more
  // squarely (rad). The 45-degree jitter bench selects this margin; see README.md.
  panel_switch_margin_rad_ = declare_parameter("panel_switch_margin_rad", 0.3);
  if (!std::isfinite(panel_switch_margin_rad_) || panel_switch_margin_rad_ < 0.0 ||
    panel_switch_margin_rad_ >= QUARTER_TURN)
  {
    throw std::invalid_argument("panel_switch_margin_rad must be finite and in [0, pi/2)");
  }
  // Firmware patrol permission is independent of the per-frame hit-turn decision.
  type_c_based_patrol_ = declare_parameter("type_c_based_patrol", false);
  idle_direction_ = declare_parameter("idle_direction", std::string("zone"));
  navigation_frame_ = declare_parameter("navigation_frame", std::string("nav_odom"));
  if (idle_direction_ != "zone" && idle_direction_ != "hold") {
    throw std::invalid_argument("idle_direction must be zone or hold");
  }
  // Field coordinates; transform into the continuous raw aim frame before use.
  zone_start_x_m_ = declare_parameter("zone_start_x_m", 4.625);
  zone_start_y_m_ = declare_parameter("zone_start_y_m", 0.0);
  idle_height_m_ = declare_parameter("idle_height_m", 0.15);
  idle_rate_rad_s_ = declare_parameter("idle_rate_rad_s", 0.5);
  idle_switch_rad_ = declare_parameter("idle_switch_rad", M_PI / 6.0);
  if (!std::isfinite(idle_rate_rad_s_) || idle_rate_rad_s_ <= 0.0 ||
    !std::isfinite(idle_switch_rad_) || idle_switch_rad_ < 0.0 ||
    !std::isfinite(fire_max_state_age_s_) || fire_max_state_age_s_ < 0.0)
  {
    throw std::invalid_argument("idle rate/switch and fire age must be finite and nonnegative");
  }
  muzzle_frame_ = declare_parameter("muzzle_frame", std::string("muzzle"));
  // Bench only: a Header per publish tick, sent or not, for lockstep.
  const auto tick_topic = declare_parameter("tick_topic", std::string(""));
  const auto state_ack_topic = declare_parameter("state_ack_topic", std::string(""));

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

  watchdog_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {check_timeout();});
  publish_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate_hz),
    [this]() {on_publish_tick();});

  const auto ref_sys_topic = declare_parameter(
    "ref_sys_topic", std::string("/dji_serial_bridge/ref_sys"));
  ref_sys_sub_ = create_subscription<dji_serial_bridge::msg::RefSysStatus>(
    ref_sys_topic, rclcpp::SensorDataQoS(),
    [this](dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr m) {on_ref_sys(*m);});
  RCLCPP_INFO(get_logger(), "point_to_cv_target ready: idle=%s, fire age <= %.3fs",
    idle_direction_.c_str(), fire_max_state_age_s_);
}

void PointToCvTarget::on_target_state(TargetState::ConstSharedPtr msg)
{
  const rclcpp::Time now = clock_now();
  const bool stale = latest_state_ && seconds(
    now - rclcpp::Time(latest_state_->detection_stamp, now.get_clock_type())) > target_timeout_s_;
  if (!latest_state_ || msg->robot_track_id != latest_state_->robot_track_id) {
    RCLCPP_INFO(get_logger(), "target_state: tracking robot %u", msg->robot_track_id);
    spinning_ = false;
    held_panel_yaw_.reset();
  }
  if (!msg->valid || stale) {
    spinning_ = false;
    held_panel_yaw_.reset();
  }
  if (std::abs(msg->yaw_rate) > (spinning_ ? spin_exit_rad_s_ : spin_enter_rad_s_)) {
    held_panel_yaw_.reset();
  }
  latest_state_ = msg;
  target_active_ = true;
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
      enemy_zone(blue_, zone_start_x_m_, zone_start_y_m_, idle_height_m_) +
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

std::pair<bool, int> PointToCvTarget::fire_decision(std::optional<double> delay_s)
{
  if (!delay_s) {
    return {false, 0};
  }
  if (latest_state_->confidence < fire_confidence_threshold_) {
    return {false, 0};
  }
  // Python round() is half-to-even, as is nearbyint in the default mode.
  return {true, wire_fire_delay_ms(*delay_s, fire_exit_s(firmware_latency_s_, min_fire_delay_s_))};
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
  spinning_ = false;
  held_panel_yaw_.reset();
  RCLCPP_INFO(
    get_logger(), "Newest '%s' is %.2f s old - no target estimate until the next one arrives.",
    target_state_topic_.c_str(), age_s);
}

void PointToCvTarget::on_publish_tick()
{
  check_timeout();
  const rclcpp::Time now = clock_now();
  std::optional<Aim> aim;
  if (target_active_) {
    aim = compute_aim_point();
  }
  const bool idle_frame = !aim;
  if (aim) {
    const double stamp = seconds(rclcpp::Time(latest_state_->detection_stamp, now.get_clock_type()));
    idle_.target_seen(stamp, aim->pos);
    const double age = seconds(now) - stamp;
    if (age < 0.0 || age > fire_max_state_age_s_) {aim->fire_delay_s.reset();}
  } else {
    aim = Aim{idle_point_now(now), std::nullopt};
  }
  std::string out_frame;  // empty means no aim was published
  if (aim) {
    CVTarget out;
    out.header.stamp = now;
    out.header.frame_id = odom_frame_;
    out.x = static_cast<float>(aim->pos.x());
    out.y = static_cast<float>(aim->pos.y());
    out.z = static_cast<float>(aim->pos.z());
    const auto fire = fire_decision(aim->fire_delay_s);
    out.fire = fire.first;
    out.delay_ms = static_cast<uint16_t>(fire.second);
    out.type_c_based_patrol = type_c_based_patrol_;
    out.turn_to_hit = idle_frame;
    out.patrol = idle_frame;
    last_sent_point_ = aim->pos;
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

  const double detection_age_s = seconds(now - rclcpp::Time(
      state->detection_stamp, now.get_clock_type()));
  if (detection_age_s > target_timeout_s_) {
    held_panel_yaw_.reset();
    spinning_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Newest '%s' is %.2f s old (> target_timeout_s=%.2f) -- not aiming on it.",
      target_state_topic_.c_str(), detection_age_s, target_timeout_s_);
    return std::nullopt;
  }

  if (!state->valid) {
    // Unconverged: aim at the measured panel, no lead, no fire.
    spinning_ = false;
    held_panel_yaw_.reset();
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
  options.min_fire_delay_s = min_fire_delay_s_;
  options.lead = lead_enabled_;
  options.iterations = tof_iterations_;
  options.shooter_vel = shooter_vel_odom;
  options.chase_settle_s = chase_settle_s_;
  options.chase_margin_s = chase_margin_s_;
  options.accel = Vector3d(state->acceleration.x, state->acceleration.y, state->acceleration.z);
  options.held_panel_yaw = spinning_ ? std::nullopt : held_panel_yaw_;
  options.panel_switch_margin_rad = panel_switch_margin_rad_;
  // Age of THIS state at THIS tick, not latency_stat.mean: publishing runs on
  // its own timer over a cached state. See README.md.
  const Shot shot = plan_shot(
    armor, {state->radius[0], state->radius[1]}, {state->z_offset[0], state->z_offset[1]},
    state_age_s, shooter_pos_odom, v_muzzle_, spinning_, tick_s_, options);
  held_panel_yaw_ = shot.panel_yaw;
  return Aim{shot.aim_pos, shot.delay_s};
}

}  // namespace thornbots_pkg
