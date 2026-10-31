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
  auto heading_fixed_navigation = navigation;
  heading_fixed_navigation.linear().setIdentity();
  EXPECT_TRUE(correction->linear().isIdentity());
  EXPECT_TRUE((*correction * raw).matrix().isApprox(heading_fixed_navigation.matrix()));
  EXPECT_TRUE((correction->inverse() * heading_fixed_navigation * camera).matrix().isApprox(
      (raw * camera).matrix()));
  const auto next = pose(8.0, -4.0, -0.2);
  const auto next_correction = frames.navigation_to_aim(1.0, next);
  ASSERT_TRUE(next_correction);
  auto heading_fixed_next = next;
  heading_fixed_next.linear().setIdentity();
  EXPECT_TRUE((next_correction->inverse() * heading_fixed_next * camera).matrix().isApprox(
      (raw * camera).matrix()));
}

TEST(AimFrames, UsesInterpolatedRawPoseAtNavigationStamp)
{
  thornbots_pkg::AimFrames frames;
  frames.add_raw(1.0, pose(1.0, 2.0));
  frames.add_raw(2.0, pose(1.2, 2.4));
  const auto correction = frames.navigation_to_aim(1.5, pose(7.0, 8.0));
  ASSERT_TRUE(correction);
  EXPECT_TRUE(correction->matrix().isApprox(pose(5.9, 5.8).matrix()));
}

TEST(AimFrames, NeverInterpolatesAcrossARawDiscontinuity)
{
  thornbots_pkg::AimFrames frames;
  frames.add_raw(1.0, pose(0.0, 0.0));
  frames.add_navigation(1010000000, pose(0.0, 0.0));
  frames.add_raw(1.02, pose(2.0, 0.0));
  EXPECT_FALSE(frames.navigation_to_aim(1.01, pose(0.0, 0.0)));
  EXPECT_TRUE(frames.take_navigation().empty());
  EXPECT_TRUE(frames.navigation_to_aim(1.02, pose(2.0, 0.0)));
}

TEST(AimFrames, ClockResetDropsPendingNavigationFromThePreviousEpoch)
{
  thornbots_pkg::AimFrames frames;
  frames.add_raw(10.0, pose(0.0, 0.0));
  frames.add_navigation(11000000000, pose(0.5, 0.0));
  EXPECT_TRUE(frames.take_navigation().empty());
  frames.add_raw(0.0, pose(0.0, 0.0));
  frames.add_raw(11.0, pose(0.0, 0.0));
  EXPECT_TRUE(frames.take_navigation().empty());
  frames.add_navigation(11000000000, pose(0.5, 0.0));
  const auto fresh = frames.take_navigation();
  ASSERT_EQ(fresh.size(), 1u);
  EXPECT_EQ(fresh[0].first, 11000000000);
  EXPECT_DOUBLE_EQ(fresh[0].second.translation().x(), 0.5);
}

TEST(AimFrames, LateOldEpochNavigationCannotBlockTheNewEpoch)
{
  thornbots_pkg::AimFrames frames;
  frames.add_raw(10.0, pose(0.0, 0.0));
  frames.add_raw(0.5, pose(0.0, 0.0));
  frames.add_navigation(11000000000, pose(0.5, 0.0));
  frames.add_navigation(1000000000, pose(0.2, 0.0));
  frames.add_raw(1.5, pose(0.0, 0.0));
  const auto fresh = frames.take_navigation();
  ASSERT_EQ(fresh.size(), 1u);
  EXPECT_EQ(fresh[0].first, 1000000000);
  frames.add_raw(11.0, pose(0.0, 0.0));
  EXPECT_TRUE(frames.take_navigation().empty());
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
