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
  if (!odom_.empty() && (t < odom_.back().t ||
    std::hypot(x - odom_.back().x, y - odom_.back().y) > 1.0))
  {
    odom_.clear();  // clock reset or raw odometry discontinuity
    last_sent_t_.reset();
    has_prior_ = false;
    sent_offset_x_ = sent_offset_y_ = 0.0;
    resends_left_ = 0;
    last_loc_t_.reset();
    startup_t_.reset();
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

std::optional<std::pair<double, double>> Relocalizer::navigation_at(double t) const
{
  const auto raw = odom_at(t);
  if (!raw || !last_sent_t_) {
    return std::nullopt;
  }
  if (t < apply_t_) {
    if (!has_prior_) {return std::nullopt;}
    return std::make_pair(raw->first + prior_offset_x_, raw->second + prior_offset_y_);
  }
  return std::make_pair(raw->first + sent_offset_x_, raw->second + sent_offset_y_);
}

std::optional<RelocalizeCommand> Relocalizer::decide(
  double loc_t, double loc_x, double loc_y, double loc_var_xy, double now, double map_yaw, double yaw_var)
{
  if (!std::isfinite(loc_t) || !std::isfinite(loc_x) || !std::isfinite(loc_y) ||
    !std::isfinite(loc_var_xy) || loc_var_xy < 0.0 || !std::isfinite(now) ||
    !std::isfinite(map_yaw) || !std::isfinite(yaw_var) || yaw_var < 0.0 || now - loc_t > p_.max_pose_age_s || loc_t - now > 0.05 ||
    odom_.empty() || now - odom_.back().t > p_.max_pose_age_s)
  {
    startup_t_.reset();
    return std::nullopt;
  }
  if (last_sent_t_ && (now - *last_sent_t_ < p_.hold_off_s ||
    (last_loc_t_ && loc_t <= *last_loc_t_)))
  {
    return std::nullopt;
  }
  const auto at_loc = odom_at(loc_t);
  if (!at_loc) {return std::nullopt;}
  const OdomSample & last = odom_.back();
  const double speed = std::hypot(last.vx, last.vy);
  const double apply_t = now + p_.uart_latency_s + p_.mcb_read_delay_s;
  const double horizon = apply_t - last.t;
  const double raw_x = last.x + last.vx * horizon;
  const double raw_y = last.y + last.vy * horizon;
  const double rx = raw_x - at_loc->first, ry = raw_y - at_loc->second;
  const double std_m = std::hypot(std::sqrt(loc_var_xy + yaw_var * (rx*rx + ry*ry)), speed * p_.latency_std_s);
  if (std_m > p_.max_std_m) {
    startup_t_.reset();
    return std::nullopt;
  }
  const double c = std::cos(map_yaw), s = std::sin(map_yaw);
  // Rotate displacement into map, not the firmware's held offset or raw aim axes.
  double dx = loc_x + c * rx - s * ry - raw_x;
  double dy = loc_y + s * rx + c * ry - raw_y;
  double error = std::hypot(dx - sent_offset_x_, dy - sent_offset_y_);
  if (p_.absolute_policy && !last_sent_t_) {
    if (!startup_t_ || std::hypot(dx - candidate_x_, dy - candidate_y_) > p_.startup_spread_m) {
      startup_t_ = loc_t;
      candidate_x_ = dx; candidate_y_ = dy;
      return std::nullopt;
    }
    if (loc_t - *startup_t_ < p_.startup_span_s) {return std::nullopt;}
  }
  if (p_.absolute_policy && last_sent_t_ && error > p_.max_jump_m) {
    return std::nullopt;
  }
  const bool changed = error > std::max(p_.error_threshold_m, p_.n_sigma * std_m);
  const bool refresh = p_.absolute_policy && last_sent_t_ && now - *last_sent_t_ >= p_.refresh_s;
  if (last_sent_t_ && !changed && resends_left_ == 0 && !refresh) {return std::nullopt;}
  if (!p_.absolute_policy && !changed && resends_left_ == 0) {return std::nullopt;}
  if (changed || !last_sent_t_) {resends_left_ = p_.resend_count;}
  else if (resends_left_ > 0) {--resends_left_;}
  if (p_.absolute_policy && last_sent_t_ && error > p_.max_step_m) {
    const double scale = p_.max_step_m / error;
    dx = sent_offset_x_ + scale * (dx - sent_offset_x_);
    dy = sent_offset_y_ + scale * (dy - sent_offset_y_);
  }
  has_prior_ = last_sent_t_.has_value();
  prior_offset_x_ = sent_offset_x_; prior_offset_y_ = sent_offset_y_;
  last_sent_t_ = now;
  last_loc_t_ = loc_t;
  apply_t_ = apply_t;
  sent_offset_x_ = dx; sent_offset_y_ = dy;
  return RelocalizeCommand{raw_x + dx, raw_y + dy, apply_t, error, std_m};
}

}  // namespace thornbots_pkg
