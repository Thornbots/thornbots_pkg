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

#include "thornbots_pkg/mmm_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace thornbots_pkg::mmm
{
namespace
{
// Allow practical re-parking with independent translation/yaw hysteresis.
constexpr double kStartEntryDistanceM = 0.50;
constexpr double kStartEntryYawRad = 15.0 * M_PI / 180.0;
constexpr double kStartExitDistanceM = 0.60;
constexpr double kStartExitYawRad = 18.0 * M_PI / 180.0;
// Boot must remain on the mark; only save-time raw odometry gets more room.
constexpr double kBootSpawnDistanceM = 0.30;
constexpr double kSaveSpawnDistanceM = 0.50;

double yaw_distance(double a, double b)
{
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}
bool finite(Pose pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}
bool transaction(int number)
{
  return number == 2 || number == 4 || number == 6;
}
}  // namespace

bool near_raw_start(double x, double y, SpawnCheck check)
{
  const double limit = check == SpawnCheck::Boot ? kBootSpawnDistanceM : kSaveSpawnDistanceM;
  return std::isfinite(x) && std::isfinite(y) && std::hypot(x, y) <= limit;
}

StartGate::StartGate(Pose boot)
: boot_(boot) {}

void StartGate::reset(Pose boot)
{
  boot_ = boot;
  samples_.clear();
  at_start_ = false;
}

bool StartGate::update(std::optional<Pose> pose, double stamp, double now)
{
  if (!pose || !finite(*pose) || !finite(boot_) || !std::isfinite(stamp) ||
    !std::isfinite(now) || now - stamp < 0 || now - stamp > 0.5 ||
    (!samples_.empty() && stamp < samples_.back().first))
  {
    reset(boot_);
    return false;
  }
  const double distance = std::hypot(pose->x - boot_.x, pose->y - boot_.y);
  const double yaw = yaw_distance(pose->yaw, boot_.yaw);
  if (distance > kStartExitDistanceM || yaw > kStartExitYawRad) {
    reset(boot_);
    return false;
  }
  if (samples_.empty() || stamp > samples_.back().first) {
    samples_.emplace_back(stamp, *pose);
  }
  while (samples_.size() > 1 && stamp - samples_[1].first >= 2.0) {
    samples_.pop_front();
  }
  const bool parked = (at_start_ || stamp - samples_.front().first >= 2.0) &&
    std::all_of(samples_.begin(), samples_.end(), [&pose](const auto & sample) {
      return std::hypot(pose->x - sample.second.x, pose->y - sample.second.y) <= 0.03 + 1e-9 &&
             yaw_distance(pose->yaw, sample.second.yaw) <= 0.03 + 1e-9;
    });
  at_start_ = parked && (at_start_ ||
    (distance <= kStartEntryDistanceM && yaw <= kStartEntryYawRad));
  return at_start_;
}

bool ScanGate::update(const std::array<double, 4> & geometry, double stamp, double now)
{
  const bool well_formed = std::all_of(geometry.begin(), geometry.end(),
    [](double v) {return std::isfinite(v);}) && geometry[0] >= 10 && geometry[3] > 0 &&
    std::abs(geometry[2] - geometry[1] - (geometry[0] - 1) * geometry[3]) <= geometry[3] * 0.02;
  if (well_formed && geometry_) {
    for (size_t i = 0; i < geometry.size(); ++i) {
      const double tolerance = i == 0 ? 0.0 : 1e-7 + 1e-6 * std::abs((*geometry_)[i]);
      if (std::abs(geometry[i] - (*geometry_)[i]) > tolerance) {geometry_failed_ = true;}
    }
  }
  if (!well_formed || geometry_failed_ || !std::isfinite(stamp) || !std::isfinite(now) ||
    now - stamp < 0 || now - stamp > 0.5 || stamp <= stamp_)
  {
    fresh_scans_ = 0;
    return false;
  }
  if (!geometry_) {geometry_ = geometry;}
  stamp_ = stamp;
  ++fresh_scans_;
  return true;
}

bool ScanGate::ready(double now) const
{
  return geometry_ && !geometry_failed_ && fresh_scans_ >= 20 &&
         now >= stamp_ && now - stamp_ <= 0.5;
}

void Protocol::reset_session()
{
  pending_.reset();
  completed_even_.reset();
  failed_.reset();
  even_used_.fill(std::nullopt);
  // Keep odd receive windows: final legs must not dispatch another session.
}

Protocol::Protocol(std::function<void(int)> send, std::function<void(int)> dispatch,
  std::function<void(std::string)> warn)
: send_(std::move(send)), dispatch_(std::move(dispatch)), warn_(std::move(warn)) {}

bool Protocol::request(int number, double now)
{
  if (!transaction(number) || pending_ ||
    (even_used_[number] && now - *even_used_[number] < 5.0))
  {
    return false;
  }
  pending_ = number;
  last_attempt_ = now;
  attempts_ = 1;
  failed_.reset();
  send_(number);
  return true;
}

void Protocol::receive(int number, double now)
{
  if (number == 1 || number == 3) {
    const auto previous = odd_seen_[number];
    odd_seen_[number] = now;
    send_(number);
    if (!previous || now - *previous >= 5.0) {
      dispatch_(number);
    }
  } else if (transaction(number) && pending_ == number) {
    pending_.reset();
    even_used_[number] = now;
    send_(number);
    completed_even_ = number;
  }
}

void Protocol::tick(double now)
{
  if (now - last_ping_ >= 1.0) {
    send_(0);
    last_ping_ = now;
  }
  if (pending_ && now - last_attempt_ >= 0.5) {
    const int number = *pending_;
    if (attempts_ >= 10) {
      pending_.reset();
      even_used_[number] = now;
      failed_ = number;
      warn_("MMM " + std::to_string(number) +
        " ACK TIMEOUT after 10 attempts; retry after quiet interval");
    } else {
      last_attempt_ = now;
      ++attempts_;
      send_(number);
    }
  }
}

SaveDecision decide_save(bool session_ready, bool busy, bool at_start,
  std::optional<int> completed_even)
{
  if (!session_ready || busy) {return SaveDecision::Ignore;}
  return at_start && completed_even == 2 ? SaveDecision::Ready : SaveDecision::Incomplete;
}

Workflow::Workflow(Protocol & protocol)
: protocol_(protocol) {}

void Workflow::start_session()
{
  protocol_.reset_session();
  announced_.reset();
  configured_ = false;
  mapping_ready_ = false;
  recovering_ready_ = false;
}

void Workflow::setup_result(bool mapping_mode, bool success)
{
  configured_ = true;
  mapping_ready_ = mapping_mode && success;
  announced_.reset();
  recovering_ready_ = false;
}

void Workflow::save_result(bool ready_save, bool success)
{
  if (ready_save && !success) {
    recovering_ready_ = true;
    announced_.reset();
  }
}

void Workflow::tick(bool at_start, double now)
{
  if (!configured_) {return;}
  if (announced_ && protocol_.failed() == announced_) {
    announced_.reset();
  }
  if (recovering_ready_ && announced_ == 4 && !protocol_.pending() &&
    protocol_.completed_even() == 4)
  {
    recovering_ready_ = false;
  }
  const int desired = !mapping_ready_ ? 6 : (recovering_ready_ || !at_start ? 4 : 2);
  if (announced_ != desired && protocol_.request(desired, now)) {
    announced_ = desired;
  }
}
}  // namespace thornbots_pkg::mmm
