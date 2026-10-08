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

// Unit tests for armor_tracker.hpp's armor-model EKF. Synthetic 4-panel
// targets only (radii 0.30/0.24, the emulator's layout), no rclcpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

#include "thornbots_pkg/armor_tracker.hpp"

namespace tp = thornbots_pkg;
using tp::ACC;
using tp::DZ;
using tp::POS;
using tp::R;
using tp::VEL;
using tp::W;
using tp::YAW;
using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace
{

const Vector3d CAMERA(0.0, 0.0, 0.4);
constexpr double RX = 0.30, RY = 0.24, NOISE_M = 0.03, DT = 1.0 / 60.0;
constexpr double STAGGER_M = 0.09;  // sim's staggered layout: 90% of a 0.1 m panel

Matrix3d iso(double var) {return var * Matrix3d::Identity();}

tp::EkfParams ekf_params(double radius = 0.27)
{
  tp::EkfParams p;
  p.radius = radius;
  p.q_accel = 2.0;
  p.q_yaw_accel = 5.0;
  p.q_radius = 0.02;
  return p;
}

tp::TrackerParams tracker_params() {return {ekf_params()};}

// Pair 0 (k = 0, 2) sits stagger/2 above the centre, pair 1 below, as in cv_target_emulator.
std::vector<std::pair<Vector3d, double>> true_panels(
  const Vector3d & centre, double yaw, double stagger = 0.0)
{
  std::vector<std::pair<Vector3d, double>> out;
  for (int k = 0; k < 4; ++k) {
    const double yk = yaw + k * M_PI / 2.0;
    const bool even = k % 2 == 0;
    out.emplace_back(
      centre + (even ? RX : RY) * Vector3d(std::cos(yk), std::sin(yk), 0.0) +
      Vector3d(0.0, 0.0, even ? stagger / 2.0 : -stagger / 2.0), yk);
  }
  return out;
}

// Most head-on panel within 75 degrees of the camera, like the emulator.
std::optional<Vector3d> seen_panel(const Vector3d & centre, double yaw, double stagger = 0.0)
{
  std::optional<Vector3d> best;
  double best_cos = 0.0;
  for (const auto & [pos, yk] : true_panels(centre, yaw, stagger)) {
    const Vector3d to_cam = CAMERA - pos;
    const double cos_view = (std::cos(yk) * to_cam.x() + std::sin(yk) * to_cam.y()) /
      to_cam.head<2>().norm();
    if (cos_view > std::cos(75.0 * M_PI / 180.0) && (!best || cos_view > best_cos)) {
      best = pos;
      best_cos = cos_view;
    }
  }
  return best;
}

Vector3d noise(std::mt19937 & rng, double std)
{
  std::normal_distribution<double> n(0.0, std);
  Vector3d sample;
  // Sequence draws explicitly: constructor argument order differs across platforms.
  for (int axis = 0; axis < 3; ++axis) {sample[axis] = n(rng);}
  return sample;
}

const tp::State & state_of(const tp::ArmorEKF & f) {return f.state;}
const tp::State & state_of(const tp::ArmorTracker & f) {return f.state();}
double other_r_of(const tp::ArmorEKF & f) {return f.other_r;}
double other_r_of(const tp::ArmorTracker & f) {return f.other_r();}
tp::ArmorEKF make(const Vector3d & m, double t, tp::ArmorEKF *)
{
  return tp::ArmorEKF(m, CAMERA, t, iso(0.05 * 0.05), ekf_params());
}
tp::ArmorTracker make(const Vector3d & m, double t, tp::ArmorTracker *)
{
  return tp::ArmorTracker(m, CAMERA, t, iso(0.05 * 0.05), tracker_params());
}

struct RunOpts
{
  unsigned seed = 0;
  double noise = NOISE_M;
  Vector3d start{3.0, 0.0, 0.3};
  double bad_start_s = 0.0;
  double yaw0 = 0.3;
  double stagger = 0.0;
};

template<typename Filter>
struct RunResult
{
  std::optional<Filter> filt;
  Vector3d centre;
  double yaw;
};

// Feed a noisy constant-velocity spinning target.
template<typename Filter>
RunResult<Filter> run(
  const Vector3d & velocity, double spin, double seconds, const RunOpts & o = {})
{
  std::mt19937 rng(o.seed);
  RunResult<Filter> res{std::nullopt, o.start, 0.3};
  for (int i = 0; i < static_cast<int>(seconds / DT); ++i) {
    const double t = i * DT;
    res.centre = o.start + velocity * t;
    res.yaw = o.yaw0 + spin * t;
    const auto seen = seen_panel(res.centre, res.yaw, o.stagger);
    if (!seen) {
      continue;
    }
    const Vector3d meas = *seen + noise(rng, t >= o.bad_start_s ? o.noise : 0.15);
    if (!res.filt) {
      res.filt.emplace(make(meas, t, static_cast<Filter *>(nullptr)));
    } else {
      res.filt->step(meas, CAMERA, t, iso(0.05 * 0.05));
    }
  }
  return res;
}

double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

}  // namespace

TEST(ArmorTracker, StationaryTargetEstimatesCentreAndNoSpin)
{
  auto r = run<tp::ArmorEKF>(Vector3d::Zero(), 0.0, 3.0);
  const auto & s = r.filt->state;
  // Only the seen panel is observable, so check it rather than the centre.
  const Vector3d panel = s.segment<3>(POS) +
    s[R] * Vector3d(std::cos(s[YAW]), std::sin(s[YAW]), 0.0);
  EXPECT_LT((panel - *seen_panel(r.centre, 0.3)).norm(), 0.03);
  EXPECT_LT(std::abs(s[W]), 1.0);
}

TEST(ArmorTracker, SpinInPlaceRecoversRateCentreAndBothRadii)
{
  const double w = 2.0 * M_PI * 1.5;
  auto r = run<tp::ArmorEKF>(Vector3d::Zero(), w, 4.0);
  const auto & s = r.filt->state;
  EXPECT_LT(std::abs(s[W] - w), 0.1 * w);
  EXPECT_LT((s.segment<2>(POS) - r.centre.head<2>()).norm(), 0.05);
  const double lo = std::min(s[R], r.filt->other_r), hi = std::max(s[R], r.filt->other_r);
  EXPECT_LT(std::abs(lo - RY), 0.04);
  EXPECT_LT(std::abs(hi - RX), 0.04);
}

TEST(ArmorTracker, SpinDirectionIsSigned)
{
  auto r = run<tp::ArmorEKF>(Vector3d::Zero(), -2.0 * M_PI, 4.0);
  EXPECT_LT(r.filt->state[W], -0.9 * 2.0 * M_PI);
}

TEST(ArmorTracker, SpinningWhileTranslatingRecoversVelocityAndRate)
{
  const double w = 2.0 * M_PI * 1.5;
  RunOpts o;
  o.start = Vector3d(3.0, -1.5, 0.3);
  auto r = run<tp::ArmorEKF>(Vector3d(0.0, 1.0, 0.0), w, 3.0, o);
  const auto & s = r.filt->state;
  EXPECT_LT(std::abs(s[W] - w), 0.15 * w);
  EXPECT_LT(std::abs(s[VEL + 1] - 1.0), 0.3);
  EXPECT_LT((s.segment<2>(POS) - r.centre.head<2>()).norm(), 0.1);
}

TEST(ArmorTracker, HandoffStepsYawAQuarterTurnAndSwapsRadius)
{
  // Panel at (2.7, 0) faces the camera at yaw pi; turn the chassis 50 deg
  // clockwise so the k=1 panel (yaw pi + 40 deg) is the more head-on one.
  tp::ArmorEKF ekf(Vector3d(2.7, 0.0, 0.3), CAMERA, 0.0, iso(1e-4), ekf_params(0.30));
  ekf.other_r = 0.24;
  ekf.state[YAW] -= 50.0 * M_PI / 180.0;
  const double yaw_before = ekf.state[YAW];
  const Vector3d k1 = tp::panel_positions(ekf.state, ekf.other_r)[1].pos;
  const auto [k, distance] = ekf.associate(k1, CAMERA);
  EXPECT_EQ(k, 1);
  EXPECT_LT(distance, 1e-9);
  EXPECT_NEAR(ekf.state[YAW], yaw_before + M_PI / 2.0, 1e-12);
  EXPECT_EQ(ekf.state[R], 0.24);
  EXPECT_EQ(ekf.other_r, 0.30);
}

TEST(ArmorTracker, BackPanelIsNeverAssociated)
{
  tp::ArmorEKF ekf(Vector3d(2.7, 0.0, 0.3), CAMERA, 0.0, iso(1e-4), ekf_params(0.30));
  const Vector3d back = tp::panel_positions(ekf.state, ekf.other_r)[2].pos;
  EXPECT_NE(ekf.associate(back, CAMERA).first, 2);
}

TEST(ArmorTracker, PredictedDoesNotMutate)
{
  auto r = run<tp::ArmorEKF>(Vector3d(0.0, 1.0, 0.0), 6.0, 1.0);
  auto & ekf = *r.filt;
  const tp::State before = ekf.state;
  const tp::Cov P_before = ekf.P;
  const double t_before = ekf.t_sec;
  const auto [s, P] = ekf.predicted(ekf.t_sec + 0.2);
  EXPECT_GT(s[POS + 1], before[POS + 1]);
  EXPECT_GT(s[YAW], before[YAW]);
  EXPECT_EQ(ekf.state, before);
  EXPECT_EQ(ekf.P, P_before);
  EXPECT_EQ(ekf.t_sec, t_before);
}

TEST(ArmorTracker, JinkReacquiresPositionAndKeepsSpin)
{
  const double w = 2.0 * M_PI * 1.5;
  auto r = run<tp::ArmorEKF>(Vector3d::Zero(), w, 3.0);
  auto & ekf = *r.filt;
  const double t = ekf.t_sec;
  const Vector3d jumped = Vector3d(3.0, 0.8, 0.3) + Vector3d(-RX, 0.0, 0.0);
  std::vector<tp::StepResult> results;
  for (int i = 0; i < 3; ++i) {
    results.push_back(ekf.step(jumped, CAMERA, t + (i + 1) * DT, iso(0.05 * 0.05)));
  }
  EXPECT_EQ(results, (std::vector<tp::StepResult>{
    tp::StepResult::Outlier, tp::StepResult::Outlier, tp::StepResult::Reacquire}));
  EXPECT_LT(std::abs(ekf.state[POS + 1] - 0.8), 0.1);
  EXPECT_LT(std::abs(ekf.state[W] - w), 0.1 * w);
}

TEST(ArmorTracker, RadiusIsClamped)
{
  tp::ArmorEKF ekf(Vector3d(2.7, 0.0, 0.3), CAMERA, 0.0, iso(1e-4), ekf_params());
  ekf.update(Vector3d(1.0, 0.0, 0.3), iso(1e-6));
  EXPECT_GE(ekf.state[R], ekf.p.r_min);
  EXPECT_LE(ekf.state[R], ekf.p.r_max);
}

TEST(ArmorTracker, BankRecoversTheSpinAfterABadFirstSecond)
{
  // A lone EKF fed 15cm noise for the first second (the head slewing in, in
  // sim) locks onto a wrong spin on most seeds; the bank must not.
  const double w = 2.0 * M_PI * 1.5;
  int single = 0, bank = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    RunOpts o;
    o.seed = seed;
    o.bad_start_s = 1.0;
    o.yaw0 = seed * 0.37;
    single += std::abs(run<tp::ArmorEKF>(Vector3d::Zero(), w, 6.0, o).filt->state[W] - w) <
      0.1 * w;
    bank += std::abs(run<tp::ArmorTracker>(Vector3d::Zero(), w, 6.0, o).filt->state()[W] - w) <
      0.1 * w;
  }
  EXPECT_GE(bank, 7);
  EXPECT_GT(bank, single);
}

TEST(ArmorTracker, BankLeadsWithTheRightSpinSign)
{
  auto r = run<tp::ArmorTracker>(Vector3d::Zero(), -2.0 * M_PI * 1.5, 4.0);
  EXPECT_LT(r.filt->state()[W], -0.9 * 2.0 * M_PI * 1.5);
}

TEST(ArmorTracker, RayCovarianceIsDepthAlongTheRayAndLateralAcross)
{
  const Matrix3d cov = tp::ray_covariance(Vector3d(3.0, 4.0, 0.4), CAMERA, 0.1, 0.02);
  const Vector3d ray = Vector3d(3.0, 4.0, 0.0) / 5.0;
  const Vector3d across = Vector3d(-4.0, 3.0, 0.0) / 5.0;
  EXPECT_NEAR(ray.dot(cov * ray), 0.01, 1e-11);
  EXPECT_NEAR(across.dot(cov * across), 0.0004, 1e-12);
  EXPECT_NEAR(cov(2, 2), 0.0004, 1e-12);
}

TEST(ArmorTracker, BankLocksSpinUnderHeavyDepthNoiseWithRayCovariance)
{
  // 12cm depth noise, 3cm lateral: the sim's range model at 3m. An isotropic
  // 12cm R let w=0 explain the sweep; the ray covariance must not.
  const double w = 2.0 * M_PI * 2.0;
  int locked = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> depth(0.0, 0.12);
    std::optional<tp::ArmorTracker> tracker;
    for (int i = 0; i < static_cast<int>(5.0 / DT); ++i) {
      const double t = i * DT;
      const auto seen = seen_panel(Vector3d(3.0, 0.0, 0.3), seed * 0.4 + w * t);
      if (!seen) {
        continue;
      }
      const Vector3d u = (*seen - CAMERA).normalized();
      const Vector3d meas = *seen + noise(rng, 0.03) + u * depth(rng);
      const Matrix3d cov = tp::ray_covariance(meas, CAMERA, 0.12, 0.04);
      if (!tracker) {
        tracker.emplace(meas, CAMERA, t, cov, tracker_params());
      } else {
        tracker->step(meas, CAMERA, t, cov);
      }
    }
    locked += std::abs(tracker->state()[W] - w) < 0.1 * w;
  }
  EXPECT_GE(locked, 7);
}

// Worst distance from a true panel to the nearest panel the state implies.
template<typename Filter>
double panel_error(const Filter & f, const Vector3d & centre, double yaw, double stagger)
{
  const auto implied = tp::panel_positions(state_of(f), other_r_of(f));
  double worst = 0.0;
  for (const auto & [pos, yk] : true_panels(centre, yaw, stagger)) {
    (void)yk;
    double nearest = 1e9;
    for (const auto & p : implied) {
      nearest = std::min(nearest, (p.pos - pos).norm());
    }
    worst = std::max(worst, nearest);
  }
  return worst;
}

// Pair heights, the centre height, and every implied panel within 0.08 m of
// a true one: a flipped dz puts a pair 0.09 m off. Noise alone reaches
// 0.074 m on 3 of 16 seeds (0.060 on 1 of 10 numpy seeds), hence 7 of 8.
template<typename Filter>
int staggered_seeds_ok()
{
  const double w = 2.0 * M_PI * 1.5;
  int ok = 0;
  for (unsigned seed = 0; seed < 8; ++seed) {
    RunOpts o;
    o.seed = seed;
    o.stagger = STAGGER_M;
    auto r = run<Filter>(Vector3d::Zero(), w, 4.0, o);
    const auto & s = state_of(*r.filt);
    ok += std::abs(std::abs(s[DZ]) - STAGGER_M / 2.0) < 0.015 &&
      std::abs(s[POS + 2] - r.centre.z()) < 0.015 &&
      panel_error(*r.filt, r.centre, r.yaw, STAGGER_M) < 0.08;
  }
  return ok;
}

TEST(ArmorTracker, StaggeredSpinRecoversBothPairHeights)
{
  EXPECT_GE(staggered_seeds_ok<tp::ArmorEKF>(), 7);
  EXPECT_GE(staggered_seeds_ok<tp::ArmorTracker>(), 7);
}

TEST(ArmorTracker, FlatSpinKeepsDzNearZero)
{
  auto r = run<tp::ArmorTracker>(Vector3d::Zero(), 2.0 * M_PI * 1.5, 4.0);
  EXPECT_LT(std::abs(r.filt->state()[DZ]), 0.01);
}

TEST(ArmorTracker, OddHandoffFlipsDzAndItsCovariance)
{
  tp::ArmorEKF ekf(Vector3d(2.7, 0.0, 0.3), CAMERA, 0.0, iso(1e-4), ekf_params(0.30));
  ekf.state[DZ] = 0.04;
  ekf.P(2, DZ) = ekf.P(DZ, 2) = -1e-4;
  ekf.state[YAW] -= 50.0 * M_PI / 180.0;
  const Vector3d k1 = tp::panel_positions(ekf.state, ekf.other_r)[1].pos;
  EXPECT_NEAR(k1.z(), 0.3 - 0.04, 1e-12);
  EXPECT_EQ(ekf.associate(k1, CAMERA).first, 1);
  EXPECT_NEAR(ekf.state[DZ], -0.04, 1e-15);
  EXPECT_EQ(ekf.P(2, DZ), 1e-4);
  EXPECT_EQ(ekf.P(DZ, 2), 1e-4);
  Eigen::SelfAdjointEigenSolver<tp::Cov> eig(ekf.P);
  EXPECT_GE(eig.eigenvalues().minCoeff(), -1e-12);
}

TEST(ArmorTracker, SinglePanelFacingUpdateHoldsAStillTargetsYaw)
{
  // One panel in view: without the facing update yaw random-walks and the
  // centre swings round the panel.
  const Vector3d seen = *seen_panel(Vector3d(3.0, 0.0, 0.3), M_PI);
  std::mt19937 rng(0);
  std::optional<tp::ArmorTracker> tracker;
  for (int i = 0; i < static_cast<int>(30.0 / DT); ++i) {
    const Vector3d meas = seen + noise(rng, 0.01);
    if (!tracker) {
      tracker.emplace(meas, CAMERA, i * DT, iso(0.05 * 0.05), tracker_params());
    } else {
      tracker->step(meas, CAMERA, i * DT, iso(0.05 * 0.05), 16.3, 3, 0.3);
    }
  }
  EXPECT_LT(std::abs(wrap(tracker->state()[YAW] - M_PI)), 0.15);
  EXPECT_LT((tracker->state().segment<2>(POS) - Eigen::Vector2d(3.0, 0.0)).norm(), 0.06);
}

TEST(ArmorTracker, AccelerationTracksBrakingUnderSpin)
{
  // 4 m/s braking at 6 m/s^2 while spinning 1 Hz, as target_driver's path end.
  const double w = 2.0 * M_PI, a = -6.0;
  std::mt19937 rng(1);
  std::optional<tp::ArmorEKF> ekf;
  double vy = 4.0;
  for (int i = 0; i < static_cast<int>(1.2 / DT); ++i) {
    const double t = i * DT;
    const double pre = std::max(0.0, t - 0.6);  // cruise 0.6 s, then brake
    vy = 4.0 + a * pre;
    const Vector3d centre(3.0, -2.0 + 4.0 * t + 0.5 * a * pre * pre, 0.3);
    const auto seen = seen_panel(centre, 0.3 + w * t);
    if (!seen) {
      continue;
    }
    const Vector3d meas = *seen + noise(rng, 0.01);
    if (!ekf) {
      tp::EkfParams p = ekf_params();
      p.spin_prior = w;
      p.spin_prior_std = 0.5;
      p.q_jerk = 3.0;
      p.accel_tau_s = 1.0;
      ekf.emplace(meas, CAMERA, t, iso(0.03 * 0.03), p);
    } else {
      ekf->step(meas, CAMERA, t, iso(0.03 * 0.03));
    }
  }
  EXPECT_LT(ekf->state[ACC + 1], 0.5 * a);
  EXPECT_LT(std::abs(ekf->state[VEL + 1] - vy), 0.5);
}

TEST(ArmorTracker, ZeroJerkIsConstantVelocity)
{
  tp::ArmorEKF ekf(Vector3d(2.7, 0.0, 0.3), CAMERA, 0.0, iso(1e-4), ekf_params(0.30));
  ekf.state.segment<3>(ACC) = Vector3d(5.0, 5.0, 5.0);
  const auto [s, P] = ekf.predicted(0.5);
  (void)P;
  EXPECT_LT((s.segment<3>(POS) - ekf.state.segment<3>(POS)).norm(), 1e-12);
  EXPECT_LT((s.segment<3>(VEL) - ekf.state.segment<3>(VEL)).norm(), 1e-12);
  EXPECT_TRUE((ekf.P.block<3, 3>(ACC, ACC).isZero(0.0)));
}

namespace
{

// Park a non-spinning target 3 s, then start motion(t); return (t, still lead, state).
std::vector<std::tuple<double, bool, tp::State>> parked_then(
  const std::function<std::pair<Vector3d, double>(double)> & motion, unsigned seed = 0)
{
  std::mt19937 rng(seed);
  std::optional<tp::ArmorTracker> tracker;
  std::vector<std::tuple<double, bool, tp::State>> log;
  for (int i = 0; i < static_cast<int>(4.0 / DT); ++i) {
    const double t = i * DT;
    const auto [centre, yaw] = motion(std::max(0.0, t - 3.0));
    const Vector3d meas = *seen_panel(centre, yaw) + noise(rng, NOISE_M);
    if (!tracker) {
      tp::TrackerParams p = tracker_params();
      p.ekf.q_jerk = 3.0;
      p.ekf.accel_tau_s = 1.0;
      tracker.emplace(meas, CAMERA, t, iso(NOISE_M * NOISE_M), p);
    } else {
      tracker->step(meas, CAMERA, t, iso(NOISE_M * NOISE_M), 16.3, 3, 0.3);
    }
    log.emplace_back(t, tracker->best().p.still, tracker->state());
  }
  return log;
}

}  // namespace

TEST(ArmorTracker, ParkedTargetIsLedByTheStillHypothesis)
{
  const auto log = parked_then([](double) {return std::make_pair(Vector3d(3.0, 0.0, 0.3), M_PI);});
  for (const auto & [t, still, s] : log) {
    if (t > 1.0 && t < 3.0) {
      EXPECT_TRUE(still) << "t=" << t;
      EXPECT_TRUE(s.segment<3>(VEL).isZero(0.0) && s.segment<3>(ACC).isZero(0.0) && s[W] == 0.0);
    }
  }
}

TEST(ArmorTracker, ParkedTargetThatDrivesOffLeavesTheStillHypothesisFast)
{
  // 6 m/s^2 across the view, the braking figure target_driver uses.
  const auto log = parked_then(
    [](double t) {return std::make_pair(Vector3d(3.0, 3.0 * t * t, 0.3), M_PI);});
  std::optional<double> left;
  for (const auto & [t, still, s] : log) {
    (void)s;
    if (t >= 3.0 && !still) {
      left = t;
      break;
    }
  }
  ASSERT_TRUE(left.has_value());
  EXPECT_LT(*left - 3.0, 0.3);
}

TEST(ArmorTracker, SpinningInPlaceIsNeverLedByTheStillHypothesis)
{
  auto r = run<tp::ArmorTracker>(Vector3d::Zero(), 2.0 * M_PI * 1.5, 4.0);
  EXPECT_FALSE(r.filt->best().p.still);
}
