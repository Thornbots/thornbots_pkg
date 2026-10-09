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

#include "thornbots_pkg/mcb_relay_core.hpp"

#include <algorithm>
#include <cmath>

namespace thornbots_pkg
{

void Relocalizer::add_odom(double t, double x, double y, double vx, double vy)
{
  if (!odom_.empty() && t < odom_.back().t) {
    odom_.clear();  // clock reset (sim restart)
  }
  odom_.push_back({t, x, y, vx, vy});
  while (!odom_.empty() && odom_.front().t < t - p_.history_s) {
    odom_.pop_front();
  }
}

std::optional<std::pair<double, double>> Relocalizer::odom_at(double t, double max_ahead_s) const
{
  if (odom_.empty() || t < odom_.front().t || t > odom_.back().t + max_ahead_s) {
    return std::nullopt;
  }
  if (t > odom_.back().t) {
    const auto & last = odom_.back();
    return std::make_pair(last.x + last.vx * (t - last.t), last.y + last.vy * (t - last.t));
  }
  // First sample with stamp >= t (bisect_left).
  const auto it = std::lower_bound(
    odom_.begin(), odom_.end(), t, [](const OdomSample & s, double v) {return s.t < v;});
  const OdomSample & b = *it;
  if (it == odom_.begin() || b.t == t) {
    return std::make_pair(b.x, b.y);
  }
  const OdomSample & a = *(it - 1);
  const double f = (t - a.t) / (b.t - a.t);
  return std::make_pair(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y));
}

std::optional<RelocalizeCommand> Relocalizer::decide(
  double loc_t, double loc_x, double loc_y, double loc_var_xy, double now)
{
  if (last_sent_t_ && now - *last_sent_t_ < p_.hold_off_s) {
    return std::nullopt;  // the last one may not have reached /odom yet
  }
  const auto at_loc = odom_at(loc_t);
  if (!at_loc) {
    return std::nullopt;
  }
  const double dx = loc_x - at_loc->first, dy = loc_y - at_loc->second;
  const double error = std::hypot(dx, dy);
  const OdomSample & last = odom_.back();
  const double speed = std::hypot(last.vx, last.vy);
  const double std_m =
    std::hypot(std::sqrt(std::max(loc_var_xy, 0.0)), speed * p_.latency_std_s);
  if (std_m > p_.max_std_m || error <= std::max(p_.error_threshold_m, p_.n_sigma * std_m)) {
    return std::nullopt;
  }
  // /odom is stamped on arrival, one UART leg after the MCB read it, and the
  // frame we send lands one leg plus a read delay from now.
  const double horizon = now - last.t + 2.0 * p_.uart_latency_s + p_.mcb_read_delay_s;
  last_sent_t_ = now;
  const double apply_t = now + p_.uart_latency_s + p_.mcb_read_delay_s;
  return RelocalizeCommand{
    last.x + last.vx * horizon + dx, last.y + last.vy * horizon + dy, apply_t, error, std_m};
}

}  // namespace thornbots_pkg
