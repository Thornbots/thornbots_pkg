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
#include "thornbots_pkg/aim_frames.hpp"

namespace thornbots_pkg
{

void AimFrames::add_raw(double stamp, const Eigen::Isometry3d & pose)
{
  if (!raw_.empty() && stamp < raw_.back().first) {
    raw_.clear();
  }
  if (!raw_.empty() && stamp == raw_.back().first) {
    raw_.back().second = pose;
  } else {
    raw_.emplace_back(stamp, pose);
  }
  while (raw_.size() > 2 && raw_[1].first < stamp - 5.0) {
    raw_.pop_front();
  }
}

bool AimFrames::too_old(double stamp) const
{
  return !raw_.empty() && stamp < raw_.front().first;
}

std::optional<Eigen::Isometry3d> AimFrames::navigation_to_aim(
  double stamp, const Eigen::Isometry3d & navigation_pose) const
{
  if (raw_.empty() || too_old(stamp) || stamp > raw_.back().first) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < raw_.size(); ++i) {
    if (stamp == raw_[i].first) {
      return navigation_pose * raw_[i].second.inverse();
    }
    if (raw_[i].first > stamp) {
      const auto & before = raw_[i - 1];
      const auto & after = raw_[i];
      const double fraction = (stamp - before.first) / (after.first - before.first);
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = before.second.translation() * (1.0 - fraction) +
        after.second.translation() * fraction;
      pose.linear() = Eigen::Quaterniond(before.second.linear()).slerp(
        fraction, Eigen::Quaterniond(after.second.linear())).toRotationMatrix();
      return navigation_pose * pose.inverse();
    }
  }
  return std::nullopt;
}

}  // namespace thornbots_pkg
