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

#include "thornbots_pkg/target_manager_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "thornbots_pkg/target_confidence.hpp"

namespace thornbots_pkg
{
bool sane_model(const ManagedModel & m)
{
  if (!m.usable || !m.selected || !m.center.allFinite() || !m.velocity.allFinite() ||
    !m.acceleration.allFinite() || !m.panel.allFinite()) {return false;}
  for (double v : {m.state_s, m.detection_s, m.confidence, m.yaw, m.yaw_rate,
      m.yaw_rate_variance, m.radius[0], m.radius[1], m.z_offset[0], m.z_offset[1]})
  {
    if (!std::isfinite(v)) {return false;}
  }
  if (m.confidence < 0.0 || m.confidence > 1.0 || m.yaw_rate_variance < 0.0 ||
    m.radius[0] <= 0.0 || m.radius[1] <= 0.0) {return false;}
  return std::all_of(m.variance.begin(), m.variance.end(), [](double v) {
    return std::isfinite(v) && v >= 0.0;
  });
}

namespace
{
Eigen::Vector3d center_at(const ManagedModel & m, double t)
{
  const double dt = std::max(0.0, t - m.state_s);
  return m.center + m.velocity * dt + 0.5 * m.acceleration * dt * dt;
}
Eigen::Vector3d panel_at(const ManagedModel & m, int k, double t)
{
  const double yaw = m.yaw + m.yaw_rate * std::max(0.0, t - m.state_s) + k * M_PI / 2.0;
  return center_at(m, t) + Eigen::Vector3d(
    m.radius[k % 2] * std::cos(yaw), m.radius[k % 2] * std::sin(yaw), m.z_offset[k % 2]);
}
}  // namespace

double model_drift(const ManagedModel & a, const ManagedModel & b, double now_s, double horizon_s)
{
  // One cyclic correspondence for both times preserves physical panel identity.
  double best = std::numeric_limits<double>::infinity();
  for (int shift = 0; shift < 4; ++shift) {
    double distance = 0.0;
    for (double t : {now_s, now_s + horizon_s}) {
      distance = std::max(distance, (center_at(a, t) - center_at(b, t)).norm());
      for (int k = 0; k < 4; ++k) {
        distance = std::max(distance, (panel_at(a, k, t) - panel_at(b, (k + shift) % 4, t)).norm());
      }
    }
    best = std::min(best, distance);
  }
  return best;
}

TargetManager::TargetManager(const ManagerParams & params) : params_(params)
{
  decay_confidence(0.0, 0.0, params.tau_s);
  for (double v : {params.assign_confidence, params.clear_confidence, params.drift_m,
      params.horizon_s, params.refresh_s})
  {
    if (!std::isfinite(v)) {throw std::invalid_argument("manager parameters must be finite");}
  }
  if (params.clear_confidence < 0.0 || params.clear_confidence >= params.assign_confidence ||
    params.assign_confidence > 1.0 || params.drift_m <= 0.0 || params.horizon_s < 0.0 ||
    params.refresh_s <= 0.0)
  {throw std::invalid_argument("invalid target manager thresholds or periods");}
}

TargetEvent TargetManager::clear()
{
  if (!held_) {return TargetEvent::None;}
  released_track_ = held_->track_id;
  held_.reset();
  return TargetEvent::Clear;
}

TargetEvent TargetManager::consume(const ManagedModel & model, double now_s)
{
  clear_before_assign_ = false;
  candidate_ = model;
  return tick(now_s);
}

TargetEvent TargetManager::tick(double now_s)
{
  if (!std::isfinite(now_s) || (last_now_ && now_s < *last_now_)) {
    candidate_.reset();
    last_now_ = std::isfinite(now_s) ? std::optional<double>(now_s) : std::nullopt;
    return clear();
  }
  last_now_ = now_s;
  return evaluate(now_s);
}

TargetEvent TargetManager::evaluate(double now_s)
{
  if (!candidate_) {return TargetEvent::None;}
  const auto & m = *candidate_;
  if (!sane_model(m) || m.state_s > now_s || m.detection_s > m.state_s) {
    return clear();
  }
  const double confidence = decay_confidence(m.confidence, now_s - m.state_s, params_.tau_s);
  const bool replacement = held_ && (m.track_id != held_->track_id || m.reset);
  if (confidence <= params_.clear_confidence ||
    (replacement && confidence < params_.assign_confidence)) {return clear();}
  // A same-id filter reset must reset the receiver's planner as well.
  if (replacement && m.reset && held_->track_id == m.track_id) {
    clear();
    clear_before_assign_ = true;
  }
  TargetEvent event = TargetEvent::None;
  if (!held_ || replacement) {
    if (confidence >= params_.assign_confidence) {event = TargetEvent::Assign;}
  } else if (model_drift(*held_, m, now_s, params_.horizon_s) > params_.drift_m) {
    event = TargetEvent::Update;
  } else if (now_s - last_sent_s_ >= params_.refresh_s && m.detection_s > held_->detection_s) {
    event = TargetEvent::Refresh;
  }
  // Reset is an edge, including on suppressed inputs and subsequent timer ticks.
  candidate_->reset = false;
  if (event != TargetEvent::None) {
    held_ = m;
    last_sent_s_ = now_s;
  }
  return event;
}
}  // namespace thornbots_pkg
