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
  // A >1 m single POSE step is a reanchor, not motion to interpolate through.
  if (!raw_.empty() && (stamp < raw_.back().first ||
    (pose.translation() - raw_.back().second.translation()).norm() > 1.0))
  {
    raw_.clear();
    pending_.clear();
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
      auto correction = Eigen::Isometry3d::Identity();
      correction.translation() = navigation_pose.translation() - raw_[i].second.translation();
      return correction;
    }
    if (raw_[i].first > stamp) {
      const auto & before = raw_[i - 1];
      const auto & after = raw_[i];
      const double fraction = (stamp - before.first) / (after.first - before.first);
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = before.second.translation() * (1.0 - fraction) +
        after.second.translation() * fraction;
      auto correction = Eigen::Isometry3d::Identity();
      correction.translation() = navigation_pose.translation() - pose.translation();
      return correction;
    }
  }
  return std::nullopt;
}

void AimFrames::add_navigation(int64_t stamp_ns, const Eigen::Isometry3d & pose)
{
  pending_.emplace_back(stamp_ns, pose);
  if (pending_.size() > 100) {
    pending_.pop_front();
  }
}

std::vector<std::pair<int64_t, Eigen::Isometry3d>> AimFrames::take_navigation()
{
  std::vector<std::pair<int64_t, Eigen::Isometry3d>> out;
  while (!pending_.empty()) {
    const auto & sample = pending_.front();
    const double stamp = sample.first * 1e-9;
    if (too_old(stamp)) {
      pending_.pop_front();
      continue;
    }
    const auto correction = navigation_to_aim(stamp, sample.second);
    if (!correction) {
      break;
    }
    out.emplace_back(sample.first, *correction);
    pending_.pop_front();
  }
  return out;
}

}  // namespace thornbots_pkg
