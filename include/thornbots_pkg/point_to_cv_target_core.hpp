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

// Pure intercept-solve, shot-planner and patrol math for point_to_cv_target
// (no rclcpp), unit-tested standalone in test/test_point_to_cv_target_core.cpp.
// See README.md's ### point_to_cv_target Notes for the design rationale.

#ifndef THORNBOTS_PKG__POINT_TO_CV_TARGET_CORE_HPP_
#define THORNBOTS_PKG__POINT_TO_CV_TARGET_CORE_HPP_

#include <array>
#include <optional>
#include <utility>

#include "Eigen/Dense"

namespace thornbots_pkg
{

using Eigen::Vector3d;

constexpr double QUARTER_TURN = 1.5707963267948966;  // pi / 2

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
  double firmware_latency_s = 0.05;
  bool lead = true;
  int iterations = 3;
  Vector3d shooter_vel = Vector3d::Zero();
  std::optional<double> chase_settle_s;  // set: chase the facing panel while spinning
  double chase_margin_s = 0.0;
  Vector3d accel = Vector3d::Zero();  // the center's
};

struct Shot
{
  Vector3d aim_pos;  // odom point a gun leaving from shooter(aim horizon) points through
  std::optional<double> delay_s;  // fire after this long, or nullopt for no shot this tick
};

// Choose an aim point and fire delay against a TargetState armor model, age_s
// old. radius, z_offset: per pair, as in TargetState. Each aim holds a tick_s,
// then the gimbal trails it by gimbal_lag_s; the aim targets the middle of
// that. The fire is timed over firmware_latency_s.
// Not spinning: lead the facing panel, fire now. Spinning (shotgating): aim on
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

// Where the gun looks with no target: sweep, or face the last hit. Starts
// after_s after the last target, from the gun's own yaw, and turns at
// rate_rad_s. A hit holds its yaw for hit_turn_s, then the sweep goes on from
// there. Mirrors the MCB's own patrol (AutoAimAndFireCommand.cpp). Times in
// seconds, yaws in odom radians.
class Patrol
{
public:
  Patrol(double rate_rad_s, double after_s, double hit_turn_s)
  : rate_rad_s_(rate_rad_s), after_s_(after_s), hit_turn_s_(hit_turn_s) {}

  double rate_rad_s() const {return rate_rad_s_;}
  void target_seen(double now_s);
  void hit(double yaw, double now_s);
  // This tick's patrol yaw, or nullopt while a target is recent.
  std::optional<double> step(double now_s, double gun_yaw);

private:
  double rate_rad_s_, after_s_, hit_turn_s_;
  std::optional<double> last_target_s_;
  std::optional<double> yaw_;  // commanded yaw; nullopt until patrol starts
  double last_step_s_ = 0.0;
  std::optional<double> hit_yaw_;
  std::optional<double> hit_until_s_;
};

// Odom point range_m out from gun_pos along yaw, pitch_down_rad below level.
Vector3d patrol_point(
  const Vector3d & gun_pos, double yaw, double range_m, double pitch_down_rad);

}  // namespace thornbots_pkg

#endif  // THORNBOTS_PKG__POINT_TO_CV_TARGET_CORE_HPP_
