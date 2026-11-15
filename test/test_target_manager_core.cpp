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
#include <cmath>
#include <limits>
#include <stdexcept>

#include "thornbots_pkg/target_confidence.hpp"
#include "thornbots_pkg/target_manager_core.hpp"

namespace tp = thornbots_pkg;
using Event = tp::TargetEvent;

namespace
{
tp::ManagerParams params() {return {0.30, 0.60, 0.40, 0.100, 0.200, 0.200};}
tp::ManagedModel model(double t = 1.0, double q = 0.8)
{
  tp::ManagedModel m;
  m.state_s = m.detection_s = t;
  m.confidence = q;
  m.track_id = 42;
  m.center = {3.0, 0.0, 0.2};
  m.radius = {0.2, 0.3};
  m.z_offset = {0.05, -0.05};
  return m;
}
}  // namespace

TEST(TargetConfidence, MaximumPerCaptureReplacesEvenWithLowerScore)
{
  tp::TargetConfidence c;
  EXPECT_EQ(c.at(1, 0.3), 0.0);
  c.accept(0.8, 1.0);
  c.accept(0.7, 1.0);
  c.accept(0.9, 1.0);
  EXPECT_DOUBLE_EQ(c.at(1.0, 0.3), 0.9);
  c.accept(0.6, 1.1);
  EXPECT_DOUBLE_EQ(c.at(1.1, 0.3), 0.6);
  c.accept(std::numeric_limits<double>::quiet_NaN(), 1.2);
  EXPECT_DOUBLE_EQ(*c.capture(), 1.1);
}

TEST(TargetConfidence, CaptureDecayContinuesWithoutDoubleCounting)
{
  tp::TargetConfidence c;
  c.accept(0.8, 1.0);
  EXPECT_NEAR(tp::decay_confidence(c.at(1.1, 0.3), 0.1, 0.3), c.at(1.2, 0.3), 1e-15);
  EXPECT_NEAR(c.at(1.0 + 0.3 * std::log(2.0), 0.3), 0.4, 1e-15);
  EXPECT_DOUBLE_EQ(c.at(0.9, 0.3), 0.8);
  c.reset();
  EXPECT_DOUBLE_EQ(c.at(1.0, 0.3), 0.0);
}

TEST(TargetConfidence, BoundedScoresAndBackwardCapture)
{
  tp::TargetConfidence c;
  c.accept(2, 2);
  EXPECT_EQ(c.at(2, 0.3), 1);
  c.accept(0.7, 1);
  EXPECT_EQ(c.at(1, 0.3), 0.7);
  c.accept(-1, 3);
  EXPECT_EQ(c.at(3, 0.3), 0);
  EXPECT_EQ(tp::decay_confidence(0.8, INFINITY, 0.3), 0);
}

TEST(TargetManager, RejectsInvalidConfiguration)
{
  for (double tau : {0.0, -1.0, static_cast<double>(INFINITY), static_cast<double>(NAN)}) {
    auto p = params(); p.tau_s = tau;
    EXPECT_THROW(tp::TargetManager{p}, std::invalid_argument);
  }
  for (auto thresholds : {std::pair<double, double>{0.4, 0.4}, {0.3, 0.4}, {1.1, 0.4}, {0.6, -0.1}}) {
    auto p = params();
    p.assign_confidence = thresholds.first; p.clear_confidence = thresholds.second;
    EXPECT_THROW(tp::TargetManager{p}, std::invalid_argument);
  }
}

TEST(TargetManager, HysteresisBoundaryHasNoDwellOrUpdateCount)
{
  tp::TargetManager manager(params());
  auto m = model(1, 0.59);
  EXPECT_EQ(manager.consume(m, 1), Event::None);
  m.confidence = 0.60;
  EXPECT_EQ(manager.consume(m, 1), Event::Assign);
  m.confidence = 0.5;
  EXPECT_EQ(manager.consume(m, 1), Event::None);
  EXPECT_TRUE(manager.assigned());
  m.confidence = 0.4;
  EXPECT_EQ(manager.consume(m, 1), Event::Clear);
  EXPECT_FALSE(manager.assigned());
  m.confidence = 0.5;
  EXPECT_EQ(manager.consume(m, 1), Event::None);
  m.confidence = 0.6;
  EXPECT_EQ(manager.consume(m, 1), Event::Assign);
}

TEST(TargetManager, StoppedInputDecaysAndNeverRefreshes)
{
  tp::TargetManager manager(params());
  EXPECT_EQ(manager.consume(model(), 1), Event::Assign);
  EXPECT_EQ(manager.tick(1.2), Event::None);
  EXPECT_EQ(manager.tick(1.205), Event::None);
  EXPECT_EQ(manager.tick(1.210), Event::Clear);
  EXPECT_EQ(manager.released_track(), 42u);
  EXPECT_EQ(manager.tick(1.8), Event::None);
}

TEST(TargetManager, RepeatedOldEvidenceCannotRestoreConfidence)
{
  tp::TargetManager manager(params());
  auto m = model();
  manager.consume(m, 1);
  EXPECT_EQ(manager.consume(m, 1.210), Event::Clear);
  EXPECT_EQ(manager.consume(m, 1.22), Event::None);
}

TEST(TargetManager, NewEvidenceAllowsFiveHzRefreshAtOriginalStateTime)
{
  tp::TargetManager manager(params());
  manager.consume(model(), 1);
  EXPECT_EQ(manager.consume(model(1.199), 1.199), Event::None);
  EXPECT_EQ(manager.consume(model(1.201), 1.201), Event::Refresh);
  EXPECT_EQ(manager.tick(1.402), Event::None);
}

TEST(TargetManager, SwitchClearsWeakReplacementOrImmediatelyAssignsStrongOne)
{
  for (bool strong : {false, true}) {
    tp::TargetManager manager(params());
    manager.consume(model(), 1);
    auto next = model(1.01, strong ? 0.8 : 0.5); next.track_id = 99;
    EXPECT_EQ(manager.consume(next, 1.01), strong ? Event::Assign : Event::Clear);
    EXPECT_EQ(manager.assigned(), strong);
    EXPECT_EQ(manager.tick(1.015), Event::None);
  }
}

TEST(TargetManager, ResetAndRetirementAreExplicitLossEvents)
{
  for (bool retired : {false, true}) {
    tp::TargetManager manager(params()); manager.consume(model(), 1);
    auto next = model(1.01, 0.5);
    next.reset = !retired; next.selected = !retired;
    EXPECT_EQ(manager.consume(next, 1.01), Event::Clear);
  }
}

TEST(TargetManager, ConfidentResetReassignsOnlyOnce)
{
  tp::TargetManager manager(params()); manager.consume(model(), 1);
  auto next = model(1.01); next.reset = true;
  EXPECT_EQ(manager.consume(next, 1.01), Event::Assign);
  EXPECT_TRUE(manager.clear_before_assign());
  EXPECT_EQ(manager.tick(1.015), Event::None);
}

TEST(TargetManager, InvalidNumbersClearButLargeFiniteVarianceIsAllowed)
{
  auto valid = model(); valid.variance.fill(1e12);
  tp::TargetManager manager(params());
  EXPECT_EQ(manager.consume(valid, 1), Event::Assign);
  for (int field = 0; field < 16; ++field) {
    manager.consume(model(), 1);
    auto bad = model();
    switch (field) {
      case 0: bad.center.x() = NAN; break;
      case 1: bad.velocity.y() = INFINITY; break;
      case 2: bad.acceleration.z() = NAN; break;
      case 3: bad.variance[5] = -1; break;
      case 4: bad.yaw_rate_variance = NAN; break;
      case 5: bad.confidence = NAN; break;
      case 6: bad.yaw = INFINITY; break;
      case 7: bad.panel.x() = NAN; break;
      case 8: bad.yaw_rate = INFINITY; break;
      case 9: bad.radius[1] = NAN; break;
      case 10: bad.z_offset[0] = INFINITY; break;
      case 11: bad.variance[0] = NAN; break;
      case 12: bad.yaw_rate_variance = -1; break;
      case 13: bad.state_s = NAN; break;
      case 14: bad.detection_s = INFINITY; break;
      default: bad.usable = false;
    }
    EXPECT_EQ(manager.consume(bad, 1), Event::Clear);
  }
}

TEST(TargetManager, CommonTimePredictionUsesConstantAccelerationAndRigidSpin)
{
  auto a = model(); a.velocity = {1, -2, 0}; a.acceleration = {2, 1, 0}; a.yaw_rate = 3;
  auto b = a; b.state_s += 0.1; b.detection_s = b.state_s;
  b.center += a.velocity * 0.1 + 0.5 * a.acceleration * 0.01;
  b.velocity += a.acceleration * 0.1; b.yaw += 0.3;
  EXPECT_NEAR(tp::model_drift(a, b, 1.2, 0.2), 0.0, 1e-12);
}

TEST(TargetManager, RelabeledPanelsPreserveIdentityAndAlternatingGeometry)
{
  const auto a = model(); auto b = a;
  b.yaw += M_PI / 2;
  std::swap(b.radius[0], b.radius[1]); std::swap(b.z_offset[0], b.z_offset[1]);
  EXPECT_NEAR(tp::model_drift(a, b, 1, 0.2), 0.0, 1e-12);
  tp::TargetManager manager(params()); manager.consume(a, 1);
  EXPECT_EQ(manager.consume(b, 1), Event::None);
}

TEST(TargetManager, DriftThresholdAndEmittedBaseline)
{
  tp::TargetManager manager(params()); auto a = model(); manager.consume(a, 1);
  a.center.z() += 0.08;
  EXPECT_EQ(manager.consume(a, 1.01), Event::None);
  a.center.z() += 0.03;
  EXPECT_EQ(manager.consume(a, 1.02), Event::Update);
  a.center.z() += 0.08;
  EXPECT_EQ(manager.consume(a, 1.03), Event::None);
}

TEST(TargetManager, HorizonCatchesVelocityAccelerationAndSpinDisagreement)
{
  for (int field = 0; field < 3; ++field) {
    auto a = model(); auto b = a;
    if (field == 0) {b.velocity.x() = 0.51;}
    if (field == 1) {b.acceleration.y() = 5.1;}
    if (field == 2) {b.yaw_rate = 2.6;}
    tp::TargetManager manager(params()); manager.consume(a, 1);
    EXPECT_EQ(manager.consume(b, 1), Event::Update);
  }
}

TEST(TargetManager, BackwardClockClearsAndFutureOrUnusableModelsCannotAssign)
{
  tp::TargetManager manager(params()); manager.consume(model(), 1);
  EXPECT_EQ(manager.tick(0.9), Event::Clear);
  EXPECT_EQ(manager.consume(model(), 0.95), Event::None);
  auto m = model(); m.usable = false;
  EXPECT_EQ(manager.consume(m, 1), Event::None);
}

TEST(TargetManager, ResetWithFutureModelOrBackwardClockStillDeliversClear)
{
  for (bool backward : {false, true}) {
    tp::TargetManager manager(params()); manager.consume(model(), 1);
    auto next = model(backward ? 0.9 : 1.1); next.reset = true;
    EXPECT_EQ(manager.consume(next, backward ? 0.9 : 1.0), Event::Clear);
    EXPECT_FALSE(manager.assigned());
  }
}
