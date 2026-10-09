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

// Unit tests for mcb_relay_core.hpp's Relocalizer; no rclcpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "thornbots_pkg/mcb_relay_core.hpp"

namespace tp = thornbots_pkg;
using tp::Relocalizer;

namespace
{
// Feed /odom at 100 Hz for a chassis moving at vx along x from x0 at t=0.
void drive(Relocalizer & rel, double vx, double t_end, double dt = 0.01, double x0 = 0.0)
{
  double t = 0.0;
  while (t <= t_end + 1e-9) {
    rel.add_odom(t, x0 + vx * t, 0.0, vx, 0.0);
    t += dt;
  }
}

// pytest.approx default tolerance: rel 1e-6, abs 1e-12.
::testing::AssertionResult approx(double a, double b)
{
  const double tol = std::max(1e-6 * std::abs(b), 1e-12);
  if (std::abs(a - b) <= tol) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << a << " != approx(" << b << ")";
}
}  // namespace

TEST(Relocalizer, StillOffsetSendsLocalizationPose)
{
  Relocalizer rel;
  drive(rel, 0.0, 0.5);
  const auto out = rel.decide(0.4, 0.10, 0.0, 1e-6, 0.5);
  ASSERT_TRUE(out);
  EXPECT_TRUE(approx(out->x, 0.10));
  EXPECT_TRUE(approx(out->y, 0.0));
  EXPECT_TRUE(approx(out->error_m, 0.10));
  EXPECT_TRUE(approx(out->apply_t, 0.5 + 0.005 + 0.002));
}

TEST(Relocalizer, MovingOffsetLandsWhereTheMcbWillBe)
{
  Relocalizer::Params p;
  p.latency_std_s = 0.001;
  Relocalizer rel(p);
  drive(rel, 2.0, 0.5);
  // Localization at t=0.4 says we were 0.1 m further on than /odom did.
  const auto out = rel.decide(0.4, 2.0 * 0.4 + 0.1, 0.0, 1e-6, 0.52);
  ASSERT_TRUE(out);
  // The MCB reads its odometry at apply_t; /odom shows that one UART leg later.
  const double mcb_odom_at_apply = 2.0 * (out->apply_t + rel.params().uart_latency_s);
  EXPECT_TRUE(approx(out->x, mcb_odom_at_apply + 0.1));
}

TEST(Relocalizer, SmallOrUncertainOffsetsAreNotSent)
{
  Relocalizer rel;
  drive(rel, 0.0, 0.5);
  EXPECT_FALSE(rel.decide(0.4, 0.04, 0.0, 1e-6, 0.5));  // under 0.05 m
  EXPECT_FALSE(rel.decide(0.4, 0.10, 0.0, 0.03 * 0.03, 0.5));  // std over max_std_m
  // 0.055 m clears the threshold but not 3 sigma of 0.019 m.
  EXPECT_FALSE(rel.decide(0.4, 0.055, 0.0, 0.019 * 0.019, 0.5));
}

TEST(Relocalizer, SpeedAddsExtrapolationUncertainty)
{
  Relocalizer::Params p;
  p.latency_std_s = 0.003;
  p.max_std_m = 0.02;
  Relocalizer rel(p);
  drive(rel, 8.0, 0.5);  // 8 m/s x 3 ms = 0.024 m std, over max_std_m
  EXPECT_FALSE(rel.decide(0.4, 8.0 * 0.4 + 0.2, 0.0, 1e-6, 0.5));
}

TEST(Relocalizer, HoldOffAfterASend)
{
  Relocalizer::Params p;
  p.hold_off_s = 0.3;
  Relocalizer rel(p);
  drive(rel, 0.0, 1.0);
  EXPECT_TRUE(rel.decide(0.4, 0.1, 0.0, 1e-6, 0.5));
  EXPECT_FALSE(rel.decide(0.6, 0.1, 0.0, 1e-6, 0.7));
  EXPECT_TRUE(rel.decide(0.8, 0.1, 0.0, 1e-6, 0.85));
}

TEST(Relocalizer, LocalizationStampOutsideOdomHistory)
{
  Relocalizer::Params p;
  p.history_s = 1.0;
  Relocalizer rel(p);
  drive(rel, 0.0, 2.0);
  EXPECT_FALSE(rel.decide(0.5, 0.1, 0.0, 1e-6, 2.0));  // too old, dropped
  EXPECT_FALSE(rel.decide(2.3, 0.1, 0.0, 1e-6, 2.3));  // too far ahead
  EXPECT_TRUE(rel.decide(2.05, 0.1, 0.0, 1e-6, 2.05));  // just ahead
}

TEST(Relocalizer, InterpolatesBetweenOdomSamples)
{
  Relocalizer rel;
  rel.add_odom(0.0, 0.0, 0.0, 0.0, 0.0);
  rel.add_odom(0.1, 1.0, 2.0, 0.0, 0.0);
  auto at = rel.odom_at(0.025);
  ASSERT_TRUE(at);
  EXPECT_TRUE(approx(at->first, 0.25));
  EXPECT_TRUE(approx(at->second, 0.5));
  rel.add_odom(0.05, 0.0, 0.0, 0.0, 0.0);  // stamps went back: a sim restart
  at = rel.odom_at(0.05);
  ASSERT_TRUE(at);
  EXPECT_EQ(at->first, 0.0);
  EXPECT_EQ(at->second, 0.0);
  EXPECT_EQ(rel.odom_size(), 1u);
}
