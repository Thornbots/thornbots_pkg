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

// Pure intercept-solve, shot-planner and idle-look math for point_to_cv_target
// (no rclcpp), unit-tested standalone in test/test_point_to_cv_target_core.cpp.
// See README.md's ### point_to_cv_target Notes for the design rationale.

#ifndef THORNBOTS_PKG__POINT_TO_CV_TARGET_CORE_HPP_
#define THORNBOTS_PKG__POINT_TO_CV_TARGET_CORE_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <utility>

#include "Eigen/Dense"

namespace thornbots_pkg
{

using Eigen::Vector3d;

constexpr double QUARTER_TURN = 1.5707963267948966;  // pi / 2

// Same-track dwell on the node clock, including brief ineligible gaps.
// The caller supplies convergence/confidence/freshness; shot windows are separate.
class FireLock
{
public:
  bool update(double now_s, uint32_t track_id, bool eligible, bool spinning,
    const Vector3d & velocity, double lock_s, double stationary_speed_mps,
    double grace_s);
  void reset() {since_s_.reset(); ineligible_since_s_.reset();}

private:
  std::optional<double> since_s_;
  std::optional<double> ineligible_since_s_;
  double last_s_ = 0.0;
  uint32_t track_id_ = 0;
};

// Python-style floor modulo: the result takes the divisor's sign.
double py_mod(double a, double b);

struct Intercept
{
  Vector3d aim_pos;  // target_pos + target_vel * (tau + t_flight)
  double t_flight;  // sizes the prediction horizon only; Type-C does its own ballistics
};

// Fixed-point time-of-flight solve, no gravity/drag/elevation (Type-C owns
// those): t <- |p + v*(tau+t) - (shooter + shooter_vel*t)| / v_muzzle. All in
// one inertial-ish frame (odom). shooter_vel is the sentry's own chassis
// velocity; tau the pipeline+firmware latency already elapsed/expected (s).
Intercept solve_intercept(
  const Vector3d & target_pos, const Vector3d & target_vel, const Vector3d & shooter_pos,
  double tau, double v_muzzle, int iterations = 3,
  const Vector3d & shooter_vel = Vector3d::Zero());

// state: [xc, yc, zc, vx, vy, vz, yaw, w] in odom at its stamp.
using ArmorState = std::array<double, 8>;

struct ShotOptions
{
  double gimbal_lag_s = 0.05;
  double firmware_latency_s = 0.020;  // MCB indexer request to chamber exit
  double min_fire_delay_s = 0.040;    // allow the gimbal to acquire each aim point
  bool lead = true;
  int iterations = 3;
  Vector3d shooter_vel = Vector3d::Zero();
  std::optional<double> chase_settle_s;  // set: chase the facing panel while spinning
  double chase_margin_s = 0.0;
  Vector3d accel = Vector3d::Zero();  // the center's
  // Not spinning: Shot::panel_yaw from last tick, kept until another panel
  // faces us panel_switch_margin_rad more squarely (choose_panel).
  std::optional<double> held_panel_yaw;
  double panel_switch_margin_rad = 0.0;
};

struct Shot
{
  Vector3d aim_pos;  // odom point a gun leaving from shooter(aim horizon) points through
  std::optional<double> delay_s;  // fire after this long, or nullopt for no shot this tick
  // Not spinning: odom yaw of the led panel's normal near impact; else nullopt.
  std::optional<double> panel_yaw = std::nullopt;
};

// Panel 0-3 to lead on a robot that isn't spinning: the most face-on (normal
// yaw + k * pi/2 nearest bearing, the odom yaw from the target to us), except
// that the panel nearest held_yaw stays while no other faces us more than
// margin_rad more squarely.
int choose_panel(
  double yaw, double bearing, std::optional<double> held_yaw, double margin_rad);

// Choose an aim point and fire delay against a TargetState armor model, age_s
// old. radius, z_offset: per pair, as in TargetState. Each aim holds a tick_s,
// then the gimbal trails it by gimbal_lag_s; the aim targets the middle of
// that. The fire is timed over fire_exit_s.
// Not spinning: lead the choose_panel panel, fire now. Spinning (shotgating): aim on
// the center->shooter line at the arriving pair, fire after the delay that
// lands it there if under tick_s, else none. With chase_settle_s: chase the
// facing panel, fire mid-hold if the panel a shot meets has faced us
// chase_settle_s and will for chase_margin_s more. lead=false: aim at the
// current estimate, fire now. aim_pos is the intercept less our motion over
// the flight, since the shot carries our velocity.
Shot plan_shot(
  const ArmorState & state, const std::array<double, 2> & radius,
  const std::array<double, 2> & z_offset, double age_s, const Vector3d & shooter_pos,
  double v_muzzle, bool spinning, double tick_s, const ShotOptions & options = {});

// Fire decision to chamber exit: the MCB's latency, but never under the
// wire floor, so the MCB's request always waits min - latency first.
double fire_exit_s(double firmware_latency_s, double min_fire_delay_s);

// Wire delay names projectile exit; planner delay is the extra alignment wait.
int wire_fire_delay_ms(double alignment_s, double exit_s);

// Running mean/count of now-minus-detection-stamp latency samples (seconds).
class LatencyStat
{
public:
  void add(double sample_s);
  int count() const {return count_;}
  double mean() const {return mean_;}

private:
  int count_ = 0;
  double mean_ = 0.0;
};

// Wrap an angle to [-pi, pi).
double wrap_to_pi(double a);

// Idle look priority and bounded yaw changes. Times are capture/event seconds;
// gun yaw and points are in raw field-centred odom. Firmware owns hit turns.
class IdleLook
{
public:
  enum class Source {LastEnemy, Hit, Zone, Hold};
  Source source(double now_s, double target_timeout_s, bool hold) const;
  void target_seen(double stamp_s, const Vector3d & point);
  void hit(double stamp_s) {last_hit_s_ = stamp_s; reset();}
  void reset() {yaw_.reset(); desired_yaw_.reset();}
  Vector3d point(double now_s, const Vector3d & gun, double gun_yaw,
    const Vector3d & desired, double rate_rad_s, double switch_rad);
  const Vector3d & last_enemy() const {return last_enemy_;}

private:
  std::optional<double> last_target_s_, last_hit_s_, yaw_, desired_yaw_;
  Vector3d last_enemy_ = Vector3d::Zero();
  double last_step_s_ = 0.0;
};

Vector3d enemy_zone(bool blue, double start_x, double start_y, double height);

}  // namespace thornbots_pkg

#endif  // THORNBOTS_PKG__POINT_TO_CV_TARGET_CORE_HPP_
