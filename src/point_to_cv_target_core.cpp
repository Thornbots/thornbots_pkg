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

Vector3d enemy_zone(
  bool blue, double red_x, double red_y, double blue_x, double blue_y, double height)
{
  return {blue ? red_x : blue_x, blue ? red_y : blue_y, height};
}

}  // namespace thornbots_pkg
