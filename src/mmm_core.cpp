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

StartGate::StartGate(Pose boot, double inward)
: boot_(boot), inward_(inward) {}

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
  if (distance > 0.15 || yaw > 0.15) {
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
  at_start_ = parked && (at_start_ || (distance <= inward_ && yaw <= inward_));
  return at_start_;
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
