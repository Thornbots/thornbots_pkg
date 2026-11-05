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

// Unit tests for point_to_cv_target_core.hpp's idle looks. No rclcpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "thornbots_pkg/point_to_cv_target_core.hpp"

namespace tp = thornbots_pkg;
using Eigen::Vector3d;
using tp::IdleLook;

namespace
{
::testing::AssertionResult is_close(double a, double b, double rel_tol)
{
  if (std::abs(a - b) <= rel_tol * std::max(std::abs(a), std::abs(b))) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << a << " is not close to " << b;
}
}  // namespace

TEST(LatencyStat, RunningMean)
{
  tp::LatencyStat stat;
  for (double sample : {0.05, 0.07, 0.06}) {
    stat.add(sample);
  }
  EXPECT_EQ(stat.count(), 3);
  EXPECT_TRUE(is_close(stat.mean(), 0.06, 1e-6));
}

TEST(IdleLook, PriorityUsesNewerEnemyOrHitAndExpiresToZoneOrHold)
{
  IdleLook look;
  using Source = IdleLook::Source;
  EXPECT_EQ(look.source(1.0, 0.5, false), Source::Zone);
  EXPECT_EQ(look.source(1.0, 0.5, true), Source::Hold);
  look.target_seen(1.0, {2.0, 3.0, 0.15});
  EXPECT_EQ(look.source(4.4, 0.5, false), Source::LastEnemy);
  EXPECT_EQ(look.source(4.6, 0.5, false), Source::Zone);
  look.hit(2.0);
  EXPECT_EQ(look.source(2.1, 0.5, false), Source::Hit);
  look.target_seen(2.2, {3.0, 4.0, 0.15});
  EXPECT_EQ(look.source(2.3, 0.5, false), Source::LastEnemy);
  EXPECT_EQ(look.last_enemy().x(), 3.0);
  look.hit(5.5);
  EXPECT_EQ(look.source(6.0, 0.5, false), Source::Hit);
  EXPECT_EQ(look.source(7.6, 0.5, false), Source::Zone);
}

TEST(IdleLook, RateCapsWrapsAndIgnoresSmallDirectionChanges)
{
  IdleLook look;
  const Vector3d gun(0.0, 0.0, 0.3);
  look.point(0.0, gun, 0.0, {0.0, 3.0, 0.15}, 0.5, M_PI / 6);
  auto p = look.point(0.2, gun, 0.0, {0.0, 3.0, 0.15}, 0.5, M_PI / 6);
  EXPECT_NEAR(std::atan2(p.y(), p.x()), 0.1, 1e-9);
  p = look.point(4.0, gun, 0.0, {0.0, 3.0, 0.15}, 0.5, M_PI / 6);
  EXPECT_NEAR(std::atan2(p.y(), p.x()), M_PI / 2, 1e-9);
  p = look.point(5.0, gun, 0.0, {-0.3, 3.0, 0.15}, 0.5, M_PI / 6);
  EXPECT_NEAR(std::atan2(p.y(), p.x()), M_PI / 2, 1e-9);
  p = look.point(6.0, gun, 0.0, {-3.0, 0.0, 0.15}, 0.5, M_PI / 6);
  EXPECT_NEAR(std::atan2(p.y(), p.x()), M_PI / 2 + 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(p.z(), 0.15);
}

TEST(IdleLook, EnemyZoneMirrorsOnlyXForFirmwareTeam)
{
  EXPECT_EQ(tp::enemy_zone(false, -2.7, 0.6, 4.1, -1.2, 0.15), Vector3d(4.1, -1.2, 0.15));
  EXPECT_EQ(tp::enemy_zone(true, -2.7, 0.6, 4.1, -1.2, 0.15), Vector3d(-2.7, 0.6, 0.15));
}

TEST(WrapToPi, HalfOpenInterval)
{
  EXPECT_DOUBLE_EQ(tp::wrap_to_pi(0.0), 0.0);
  EXPECT_DOUBLE_EQ(tp::wrap_to_pi(M_PI), -M_PI);
  EXPECT_DOUBLE_EQ(tp::wrap_to_pi(-M_PI), -M_PI);
  EXPECT_NEAR(tp::wrap_to_pi(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(tp::wrap_to_pi(-5.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
}
