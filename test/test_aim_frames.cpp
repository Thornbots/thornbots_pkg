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
#include <gtest/gtest.h>
#include "thornbots_pkg/aim_frames.hpp"

namespace
{
Eigen::Isometry3d pose(double x, double y, double yaw = 0.0)
{
  auto out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(x, y, 0.0);
  out.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return out;
}
}  // namespace

TEST(AimFrames, NavigationCorrectionPreservesRawAimAndCameraChain)
{
  thornbots_pkg::AimFrames frames;
  const auto raw = pose(4.0, 1.0);
  const auto camera = pose(0.2, 0.1, 0.8);
  frames.add_raw(1.0, raw);
  const auto navigation = pose(-2.0, 3.0, 0.1);
  const auto correction = frames.navigation_to_aim(1.0, navigation);
  ASSERT_TRUE(correction);
  EXPECT_TRUE((*correction * raw).matrix().isApprox(navigation.matrix()));
  EXPECT_TRUE((correction->inverse() * navigation * camera).matrix().isApprox(
      (raw * camera).matrix()));
  const auto next = pose(8.0, -4.0, -0.2);
  const auto next_correction = frames.navigation_to_aim(1.0, next);
  ASSERT_TRUE(next_correction);
  EXPECT_TRUE((next_correction->inverse() * next * camera).matrix().isApprox(
      (raw * camera).matrix()));
}

TEST(AimFrames, UsesInterpolatedRawPoseAtNavigationStamp)
{
  thornbots_pkg::AimFrames frames;
  frames.add_raw(1.0, pose(1.0, 2.0));
  frames.add_raw(2.0, pose(3.0, 4.0));
  const auto correction = frames.navigation_to_aim(1.5, pose(7.0, 8.0));
  ASSERT_TRUE(correction);
  EXPECT_TRUE(correction->matrix().isApprox(pose(5.0, 5.0).matrix()));
}

TEST(AimFrames, RejectsMissingStaleAndFutureRawPose)
{
  thornbots_pkg::AimFrames frames;
  EXPECT_FALSE(frames.navigation_to_aim(1.0, pose(0.0, 0.0)));
  frames.add_raw(1.0, pose(0.0, 0.0));
  EXPECT_FALSE(frames.navigation_to_aim(0.9, pose(0.0, 0.0)));
  EXPECT_FALSE(frames.navigation_to_aim(1.1, pose(0.0, 0.0)));
  frames.add_raw(2.0, pose(0.0, 0.0));
  frames.add_raw(8.0, pose(0.0, 0.0));
  EXPECT_TRUE(frames.too_old(1.0));
  frames.add_raw(0.0, pose(0.0, 0.0));
  EXPECT_FALSE(frames.navigation_to_aim(2.0, pose(0.0, 0.0)));
}
