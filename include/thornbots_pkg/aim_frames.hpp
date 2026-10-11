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
#ifndef THORNBOTS_PKG__AIM_FRAMES_HPP_
#define THORNBOTS_PKG__AIM_FRAMES_HPP_

#include <cstddef>
#include <deque>
#include <optional>
#include <utility>

#include <Eigen/Geometry>

namespace thornbots_pkg
{

// Align navigation to raw aim at the same capture time; never extrapolate raw pose.
class AimFrames
{
public:
  void add_raw(double stamp, const Eigen::Isometry3d & pose);
  std::optional<Eigen::Isometry3d> navigation_to_aim(
    double stamp, const Eigen::Isometry3d & navigation_pose) const;
  bool too_old(double stamp) const;

private:
  std::deque<std::pair<double, Eigen::Isometry3d>> raw_;
};

}  // namespace thornbots_pkg
#endif  // THORNBOTS_PKG__AIM_FRAMES_HPP_
