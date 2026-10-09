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

#include "thornbots_pkg/point_to_cv_target_core.hpp"

#include <algorithm>
#include <cmath>

namespace thornbots_pkg
{

double py_mod(double a, double b)
{
  double r = std::fmod(a, b);
  if (r != 0.0) {
    if ((b < 0.0) != (r < 0.0)) {
      r += b;
    }
  } else {
    r = std::copysign(0.0, b);
  }
  return r;
}

Intercept solve_intercept(
  const Vector3d & target_pos, const Vector3d & target_vel, const Vector3d & shooter_pos,
  double tau, double v_muzzle, int iterations, const Vector3d & shooter_vel)
{
  double t = 0.0;
  for (int i = 0; i < std::max(1, iterations); ++i) {
    const double dt = tau + t;
    const Vector3d a = target_pos + target_vel * dt;
    const Vector3d b = shooter_pos + shooter_vel * t;
    const double dist = (a - b).norm();
    t = v_muzzle > 0.0 ? dist / v_muzzle : 0.0;
  }
  const double dt = tau + t;
  return {target_pos + target_vel * dt, t};
}

namespace
{
// Python round(): half to even, then % 4 as a non-negative index.
int quarter_index(double turns)
{
  const auto k = std::lround(std::nearbyint(turns));
  return static_cast<int>(((k % 4) + 4) % 4);
}
}  // namespace

Shot plan_shot(
  const ArmorState & state, const std::array<double, 2> & radius,
  const std::array<double, 2> & z_offset, double age_s, const Vector3d & shooter_pos,
  double v_muzzle, bool spinning, double tick_s, const ShotOptions & options)
{
  const double xc = state[0], yc = state[1], zc = state[2];
  const Vector3d v(state[3], state[4], state[5]);
  const double yaw = state[6], w = state[7];
  const Vector3d acc = options.accel;
  const Vector3d shooter_vel = options.shooter_vel;
  const int iterations = options.iterations;
  const double aim_horizon_s = age_s + options.gimbal_lag_s + tick_s / 2.0;
  const double fire_horizon_s = age_s + options.firmware_latency_s;

  auto center = [&](double t) -> Vector3d {
      const double h = 0.5 * t * t;
      return Vector3d(
        xc + v.x() * t + acc.x() * h, yc + v.y() * t + acc.y() * h, zc + v.z() * t + acc.z() * h);
    };
  auto shooter = [&](double t) -> Vector3d {return shooter_pos + shooter_vel * t;};
  auto bearing = [&](double t) {
      const Vector3d c = center(t), s = shooter(t);
      return std::atan2(s.y() - c.y(), s.x() - c.x());
    };
  auto facing = [&](double t) {
      return quarter_index((bearing(t) - (yaw + w * t)) / QUARTER_TURN);
    };
  auto panel = [&](int k, double t) -> Vector3d {
      const Vector3d c = center(t);
      const double yaw_k = yaw + w * t + k * QUARTER_TURN, r = radius[k % 2];
      return Vector3d(
        c.x() + r * std::cos(yaw_k), c.y() + r * std::sin(yaw_k), c.z() + z_offset[k % 2]);
    };
  auto facing_panel = [&](double t) -> Vector3d {return panel(facing(t), t);};
  auto on_line = [&](double r, double dz) {
      return [&, r, dz](double t) -> Vector3d {
               const Vector3d c = center(t);
               const double b = bearing(t);
               return Vector3d(c.x() + r * std::cos(b), c.y() + r * std::sin(b), c.z() + dz);
             };
    };

  // Fixed point on the target's true path (curved by acceleration and spin),
  // not a straight-line extrapolation: t <- |path(h + t) - shooter(h + t)| /
  // v_muzzle, the muzzle leaving at h and the shot moving with it. Returns the
  // gun's world point for that shot, and t.
  auto intercept = [&](const auto & path) -> Intercept {
      double t = 0.0;
      for (int i = 0; i < std::max(1, iterations); ++i) {
        t = v_muzzle > 0.0 ?
          (path(aim_horizon_s + t) - shooter(aim_horizon_s + t)).norm() / v_muzzle : 0.0;
      }
      const Vector3d hit = path(aim_horizon_s + t);
      return {hit - shooter_vel * t, t};
    };

  if (!options.lead) {
    return {facing_panel(0.0), 0.0};
  }
  if (!spinning) {
    return {intercept(facing_panel).aim_pos, 0.0};
  }
  if (options.chase_settle_s) {
    const Intercept hit = intercept(facing_panel);
    // Leave mid-hold of whichever aim is current then: that aim was solved for
    // this exit, so it is on the panel this shot meets. Skip shots the gimbal
    // is still jumping for, either side of a switch.
    const double delay = py_mod(aim_horizon_s - fire_horizon_s, tick_s);
    const double t_impact = fire_horizon_s + delay + hit.t_flight;
    const int k = facing(t_impact);
    double theta = yaw + w * t_impact + k * QUARTER_TURN - bearing(t_impact);
    theta = py_mod(theta + QUARTER_TURN / 2.0, QUARTER_TURN);  // 0 as it starts facing, if w > 0
    const double since = w > 0.0 ? theta / std::abs(w) : (QUARTER_TURN - theta) / std::abs(w);
    const double until = QUARTER_TURN / std::abs(w) - since;
    const bool fire = since >= *options.chase_settle_s && until >= options.chase_margin_s;
    return {hit.aim_pos, fire ? std::optional<double>(delay) : std::nullopt};
  }

  const double t_flight =
    intercept(on_line(0.5 * (radius[0] + radius[1]), 0.0)).t_flight;
  if (std::abs(w) < 1e-6) {
    return {intercept(on_line(radius[0], z_offset[0])).aim_pos, std::nullopt};
  }

  auto to_alignment = [&](double t_impact) {
      // Time from t_impact until a panel normal points along the line to the shooter.
      const double phase = py_mod(yaw + w * t_impact - bearing(t_impact), QUARTER_TURN);
      return (w > 0.0 ? py_mod(QUARTER_TURN - phase, QUARTER_TURN) : phase) / std::abs(w);
    };

  // Hold the pair until the last shot at it has left the muzzle, then switch:
  // its height is a step the gimbal settles in well under a quarter turn, and
  // a switch any earlier moves the gun under that shot.
  const double t_pair = age_s + t_flight;
  const int k = facing(t_pair + to_alignment(t_pair));
  const Intercept hit = intercept(on_line(radius[k % 2], z_offset[k % 2]));
  const double delay = to_alignment(fire_horizon_s + t_flight);
  return {hit.aim_pos, delay < tick_s ? std::optional<double>(delay) : std::nullopt};
}

void LatencyStat::add(double sample_s)
{
  ++count_;
  mean_ += (sample_s - mean_) / count_;
}

double wrap_to_pi(double a)
{
  return py_mod(a + M_PI, 2.0 * M_PI) - M_PI;
}

void Patrol::target_seen(double now_s)
{
  last_target_s_ = now_s;
  yaw_.reset();
}

void Patrol::hit(double yaw, double now_s)
{
  hit_yaw_ = yaw;
  hit_until_s_ = now_s + hit_turn_s_;
}

std::optional<double> Patrol::step(double now_s, double gun_yaw)
{
  if (last_target_s_ && now_s - *last_target_s_ < after_s_) {
    return std::nullopt;
  }
  if (hit_until_s_ && now_s < *hit_until_s_) {
    yaw_ = hit_yaw_;
  } else if (!yaw_) {
    yaw_ = gun_yaw;
  } else {
    *yaw_ += rate_rad_s_ * (now_s - last_step_s_);
  }
  last_step_s_ = now_s;
  yaw_ = wrap_to_pi(*yaw_);
  return yaw_;
}

Vector3d patrol_point(const Vector3d & gun_pos, double yaw, double range_m, double pitch_down_rad)
{
  return Vector3d(
    gun_pos.x() + range_m * std::cos(yaw), gun_pos.y() + range_m * std::sin(yaw),
    gun_pos.z() - range_m * std::tan(pitch_down_rad));
}

}  // namespace thornbots_pkg
