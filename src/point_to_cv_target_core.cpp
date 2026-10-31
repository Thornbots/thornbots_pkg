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

bool FireLock::update(
  double now_s, uint32_t track_id, bool eligible, bool spinning,
  const Vector3d & velocity, double lock_s, double stationary_speed_mps,
  double grace_s)
{
  if (!std::isfinite(now_s)) {
    reset();
    return false;
  }
  if (track_id != track_id_ || now_s < last_s_ ||
    (ineligible_since_s_ && now_s - *ineligible_since_s_ > grace_s))
  {
    reset();
  }
  track_id_ = track_id;
  last_s_ = now_s;
  if (!eligible) {
    if (!ineligible_since_s_) {ineligible_since_s_ = now_s;}
    return false;
  }
  ineligible_since_s_.reset();
  if (!since_s_) {since_s_ = now_s;}
  return lock_s == 0.0 || (!spinning && velocity.norm() < stationary_speed_mps) ||
         now_s >= *since_s_ + lock_s;
}

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

int choose_panel(
  double yaw, double bearing, std::optional<double> held_yaw, double margin_rad)
{
  std::array<double, 4> off{};
  int best = 0;
  for (int k = 0; k < 4; ++k) {
    off[k] = std::abs(wrap_to_pi(yaw + k * QUARTER_TURN - bearing));
    if (off[k] < off[best]) {
      best = k;
    }
  }
  if (!held_yaw) {
    return best;
  }
  const int held = quarter_index((*held_yaw - yaw) / QUARTER_TURN);
  return off[held] <= off[best] + margin_rad ? held : best;
}

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
  const double fire_horizon_s =
    age_s + fire_exit_s(options.firmware_latency_s, options.min_fire_delay_s);

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

  if (spinning && !options.lead) {
    return {facing_panel(0.0), 0.0};
  }
  if (!spinning) {
    // Choose near impact, a flight past the aim horizon; held_panel_yaw comes
    // from the same point last tick.
    const double t_sel = options.lead ? aim_horizon_s +
      (center(aim_horizon_s) - shooter(aim_horizon_s)).norm() / std::max(v_muzzle, 1e-6) : 0.0;
    const int k = choose_panel(
      yaw + w * t_sel, bearing(t_sel), options.held_panel_yaw, options.panel_switch_margin_rad);
    const double panel_yaw = wrap_to_pi(yaw + w * t_sel + k * QUARTER_TURN);
    const auto chosen = [&](double t) -> Vector3d {return panel(k, t);};
    if (!options.lead) {
      return {chosen(0.0), 0.0, panel_yaw};
    }
    const auto hit = intercept(chosen);
    return {hit.aim_pos, 0.0,
      wrap_to_pi(yaw + w * (aim_horizon_s + hit.t_flight) + k * QUARTER_TURN)};
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

double fire_exit_s(double firmware_latency_s, double min_fire_delay_s)
{
  return std::max(firmware_latency_s, min_fire_delay_s);
}

int wire_fire_delay_ms(double alignment_s, double exit_s)
{
  const double ms = std::nearbyint((std::max(alignment_s, 0.0) + exit_s) * 1000.0);
  return static_cast<int>(std::clamp(ms, 0.0, 65535.0));
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

void IdleLook::target_seen(double stamp_s, const Vector3d & point)
{
  last_target_s_ = stamp_s;
  last_enemy_ = point;
  reset();
}

IdleLook::Source IdleLook::source(double now_s, double timeout_s, bool hold) const
{
  const bool enemy = last_target_s_ && now_s - *last_target_s_ <= timeout_s + 3.0;
  const bool hit = last_hit_s_ && now_s - *last_hit_s_ <= 2.0;
  if (hit && (!enemy || *last_hit_s_ > *last_target_s_)) {
    return Source::Hit;
  }
  if (enemy) {return Source::LastEnemy;}
  return hold ? Source::Hold : Source::Zone;
}

Vector3d IdleLook::point(double now_s, const Vector3d & gun, double gun_yaw,
  const Vector3d & desired, double rate_rad_s, double switch_rad)
{
  const double bearing = std::atan2(desired.y() - gun.y(), desired.x() - gun.x());
  if (!desired_yaw_ || std::abs(wrap_to_pi(bearing - *desired_yaw_)) >= switch_rad) {
    desired_yaw_ = bearing;
  }
  double dt = 0.0;
  if (!yaw_) {
    yaw_ = gun_yaw;
  } else {
    dt = std::max(0.0, now_s - last_step_s_);
  }
  const double step = rate_rad_s * dt;
  *yaw_ = wrap_to_pi(*yaw_ + std::clamp(wrap_to_pi(*desired_yaw_ - *yaw_), -step, step));
  last_step_s_ = now_s;
  const double range = std::max(1.0, (desired - gun).head<2>().norm());
  return {gun.x() + range * std::cos(*yaw_), gun.y() + range * std::sin(*yaw_), desired.z()};
}

Vector3d enemy_zone(bool blue, double start_x, double start_y, double height)
{
  return {blue ? -start_x : start_x, start_y, height};
}

}  // namespace thornbots_pkg
