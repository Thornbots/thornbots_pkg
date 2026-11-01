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

// Unit tests for point_to_cv_target_core.hpp's intercept solve, shot planner
// and idle looks. No rclcpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "thornbots_pkg/point_to_cv_target_core.hpp"

namespace tp = thornbots_pkg;
using Eigen::Vector3d;
using tp::ArmorState;
using tp::IdleLook;
using tp::ShotOptions;

namespace
{

constexpr double V_MUZZLE = 25.0;
// solve_intercept's default iteration count leaves a bounded residual; the
// figures come from GEOMETRIES below, measured, not guessed.
constexpr double ITER3_REL_TOL = 1e-2;
constexpr double CONVERGED_REL_TOL = 1e-9;
// Worst measured componentwise aim error at the default iteration count is
// 6.2mm over GEOMETRIES; 2cm leaves headroom and still sits well inside the
// 0.1m armor panel, so a real aiming error can't hide under it.
constexpr double ITER3_AIM_TOL_M = 0.02;

const Vector3d ORIGIN = Vector3d::Zero();

struct Geometry
{
  Vector3d target_pos, target_vel;
  double tau;
  Vector3d shooter, shooter_vel;
};

// Spanning crossing / receding / closing / oblique motion at ARCC ranges and
// speeds. The last four rows put the shooter off the origin, including behind
// and above the target, so a solve that ignored shooter_pos (or folded it in
// with the wrong sign) can't pass. Every row is a STATIONARY shooter;
// plan_shot's own-motion correction is checked by flying the shot:
// PlanShot.MovingShooter*.
const std::vector<Geometry> GEOMETRIES = {
  {{4.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, 0.0, ORIGIN, ORIGIN},
  {{4.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, 0.12, ORIGIN, ORIGIN},
  {{8.0, 0.0, 0.0}, {0.0, 4.0, 0.0}, 0.0, ORIGIN, ORIGIN},
  {{8.0, 0.0, 0.0}, {4.0, 0.0, 0.0}, 0.0, ORIGIN, ORIGIN},  // receding head-on
  {{8.0, 0.0, 0.0}, {-4.0, 0.0, 0.0}, 0.0, ORIGIN, ORIGIN},  // closing head-on
  {{2.0, 0.0, 1.5}, {0.0, 2.0, 0.0}, 0.08, ORIGIN, ORIGIN},  // elevated target
  {{3.0, 4.0, 0.0}, {1.0, -3.0, 0.5}, 0.12, ORIGIN, ORIGIN},  // oblique
  // Shooter off-origin: same crossing geometry as row 1, shifted whole.
  {{6.0, 1.0, 0.0}, {0.0, 2.0, 0.0}, 0.0, {2.0, 1.0, 0.0}, ORIGIN},
  // Shooter behind the target in +x: the range is 3m, not 11m, so a solve that
  // dropped shooter_pos would overshoot the flight time ~3.7x.
  {{8.0, 0.0, 0.0}, {0.0, 3.0, 0.0}, 0.1, {11.0, 0.0, 0.0}, ORIGIN},
  // Turret above a low target -- exercises the z offset on its own.
  {{5.0, 0.0, 0.2}, {0.0, 2.5, 0.0}, 0.06, {0.0, 0.0, 1.2}, ORIGIN},
  // Fully oblique: shooter off-axis in all three, 3-D target velocity.
  {{3.0, 4.0, 0.5}, {1.0, -3.0, 0.5}, 0.12, {-1.0, 1.5, 0.9}, ORIGIN},
};

// pytest.approx / math.isclose style comparisons.
::testing::AssertionResult is_close(double a, double b, double rel_tol, double abs_tol = 0.0)
{
  if (std::abs(a - b) <= std::max(rel_tol * std::max(std::abs(a), std::abs(b)), abs_tol)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << a << " is not close to " << b;
}

::testing::AssertionResult approx(double a, double b)
{
  return is_close(a, b, 1e-6, 1e-12);
}

// Solve the same intercept in closed form, as truth for the iterative solve.
// |p + v*(tau + t) - (s + sv*t)| = v_muzzle*t is a quadratic in t:
// (|w|^2 - v_muzzle^2) t^2 + 2 (d.w) t + |d|^2 = 0, with d = p + v*tau - s and
// w = v - sv the closing velocity. Returns the smallest positive root (the
// first time the shot can arrive); the linear branch covers a target closing
// at exactly v_muzzle.
double analytic_flight_time(
  const Vector3d & target_pos, const Vector3d & target_vel, double tau, double v_muzzle,
  const Vector3d & shooter_pos = Vector3d::Zero(),
  const Vector3d & shooter_vel = Vector3d::Zero())
{
  const Vector3d d = target_pos + target_vel * tau - shooter_pos;
  const Vector3d w = target_vel - shooter_vel;
  const double a = w.squaredNorm() - v_muzzle * v_muzzle;
  const double b = 2.0 * d.dot(w);
  const double c = d.squaredNorm();
  if (std::abs(a) < 1e-15) {
    return -c / b;
  }
  const double disc = b * b - 4.0 * a * c;
  EXPECT_GE(disc, 0.0) << "no real intercept for this geometry";
  std::vector<double> roots;
  for (double r : {(-b - std::sqrt(disc)) / (2.0 * a), (-b + std::sqrt(disc)) / (2.0 * a)}) {
    if (r > 0.0) {
      roots.push_back(r);
    }
  }
  EXPECT_FALSE(roots.empty()) << "no positive intercept time for this geometry";
  return *std::min_element(roots.begin(), roots.end());
}

}  // namespace

TEST(SolveIntercept, InterceptConditionHolds)
{
  // The invariant the solve exists to satisfy: the aim point must be exactly
  // v_muzzle*t_flight away from the shooter, so the projectile and the target
  // arrive together.
  for (const auto & g : GEOMETRIES) {
    const auto r = tp::solve_intercept(
      g.target_pos, g.target_vel, g.shooter, g.tau, V_MUZZLE, 3, g.shooter_vel);
    // The shooter has moved by shooter_vel*t by the time the shot lands.
    const Vector3d muzzle_at_impact = g.shooter + g.shooter_vel * r.t_flight;
    EXPECT_TRUE(
      is_close((r.aim_pos - muzzle_at_impact).norm(), V_MUZZLE * r.t_flight, ITER3_REL_TOL));
  }
}

TEST(SolveIntercept, FlightTimeMatchesClosedForm)
{
  for (const auto & g : GEOMETRIES) {
    const auto r = tp::solve_intercept(
      g.target_pos, g.target_vel, g.shooter, g.tau, V_MUZZLE, 3, g.shooter_vel);
    const double expected = analytic_flight_time(
      g.target_pos, g.target_vel, g.tau, V_MUZZLE, g.shooter, g.shooter_vel);
    EXPECT_TRUE(is_close(r.t_flight, expected, ITER3_REL_TOL));
  }
}

TEST(SolveIntercept, AimPointMatchesClosedFormComponentwise)
{
  // Per-axis, not just |aim|: dropping the z lead moves the norm by well under
  // ITER3_REL_TOL on a low target while putting the shot 15cm high.
  for (const auto & g : GEOMETRIES) {
    const auto r = tp::solve_intercept(
      g.target_pos, g.target_vel, g.shooter, g.tau, V_MUZZLE, 3, g.shooter_vel);
    const double t = analytic_flight_time(
      g.target_pos, g.target_vel, g.tau, V_MUZZLE, g.shooter, g.shooter_vel);
    for (int axis = 0; axis < 3; ++axis) {
      const double expected = g.target_pos[axis] + g.target_vel[axis] * (g.tau + t);
      EXPECT_TRUE(is_close(r.aim_pos[axis], expected, 0.0, ITER3_AIM_TOL_M));
    }
  }
}

TEST(SolveIntercept, IteratingToConvergenceReachesTheClosedForm)
{
  // Pins "2-3 converges in practice" as a measurement: the fixed point is the
  // closed-form root, and the default count is a truncation of it.
  for (const auto & g : GEOMETRIES) {
    const auto r = tp::solve_intercept(
      g.target_pos, g.target_vel, g.shooter, g.tau, V_MUZZLE, 40, g.shooter_vel);
    const double expected = analytic_flight_time(
      g.target_pos, g.target_vel, g.tau, V_MUZZLE, g.shooter, g.shooter_vel);
    EXPECT_TRUE(is_close(r.t_flight, expected, CONVERGED_REL_TOL));
  }
}

TEST(SolveIntercept, ShooterOffsetIsNotIgnored)
{
  // Same target and velocity, shooter moved 4m closer along the line of sight.
  // Guards shooter_pos silently dropped, or added instead of subtracted.
  const auto far = tp::solve_intercept({8.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, ORIGIN, 0.0, V_MUZZLE);
  const auto near = tp::solve_intercept(
    {8.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {4.0, 0.0, 0.0}, 0.0, V_MUZZLE);
  // Half the range, so half the flight time, so half the crossing lead.
  EXPECT_TRUE(is_close(near.aim_pos.y(), far.aim_pos.y() / 2.0, ITER3_REL_TOL));
  EXPECT_GT(near.aim_pos.y(), 0.0);
}

TEST(SolveIntercept, ShooterVelocityIsWiredIntoTheSolve)
{
  // A moving shooter must not produce the stationary answer -- that is what
  // deleting the parameter would look like. Asserts only that the two differ
  // and that the sign is the intuitive one (chasing the target shortens the
  // closing distance, so the shot arrives sooner). The magnitude is checked on
  // plan_shot, PlanShot.MovingShooter*.
  const Vector3d target_pos(8.0, 0.0, 0.0), target_vel(0.0, 2.0, 0.0);
  const double tau = 0.05;
  const auto stationary = tp::solve_intercept(target_pos, target_vel, ORIGIN, tau, V_MUZZLE);
  const auto chasing = tp::solve_intercept(
    target_pos, target_vel, ORIGIN, tau, V_MUZZLE, 3, {5.0, 0.0, 0.0});
  EXPECT_LT(chasing.t_flight, stationary.t_flight);
  EXPECT_LT(chasing.aim_pos.y(), stationary.aim_pos.y());
}

TEST(SolveIntercept, StationaryTargetNoLead)
{
  const auto r = tp::solve_intercept({4.0, 0.0, 0.0}, ORIGIN, ORIGIN, 0.0, 25.0);
  EXPECT_TRUE(is_close(r.aim_pos.x(), 4.0, 0.0, 1e-6));
  EXPECT_TRUE(is_close(r.aim_pos.y(), 0.0, 0.0, 1e-6));
  EXPECT_TRUE(is_close(r.t_flight, 4.0 / 25.0, 1e-3));
}

TEST(SolveIntercept, CrossingTargetLeadsByTheClosedFormDistance)
{
  // Target 4m ahead, moving 2 m/s sideways (+y). Asserted against the closed
  // form, not 2.0*t: aim[1] == target_vel[1]*(tau + t) is the returned aim
  // expression restated, so it holds for any t the solve produces.
  const auto r = tp::solve_intercept({4.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, ORIGIN, 0.0, V_MUZZLE);
  const double expected = analytic_flight_time({4.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, 0.0, V_MUZZLE);
  EXPECT_GT(r.aim_pos.y(), 0.0);
  EXPECT_TRUE(is_close(r.aim_pos.y(), 2.0 * expected, ITER3_REL_TOL));
}

TEST(SolveIntercept, NoVerticalLeadFromHorizontalMotion)
{
  // Elevated target (z=1.5) so a solve that zeroed or cross-mixed the z
  // channel fails here.
  const auto r = tp::solve_intercept({4.0, 0.0, 1.5}, {0.0, 2.0, 0.0}, ORIGIN, 0.1, V_MUZZLE);
  EXPECT_TRUE(is_close(r.aim_pos.z(), 1.5, 0.0, 1e-9));
}

TEST(SolveIntercept, LatencyExtendsLeadByTauTimesVelocity)
{
  // tau shifts the aim point by exactly v*tau beyond the no-latency lead, plus
  // whatever the longer reach adds to the flight time.
  const Vector3d target_pos(4.0, 0.0, 0.0), target_vel(0.0, 2.0, 0.0);
  const double tau = 0.1;
  const auto aim_no_latency = tp::solve_intercept(target_pos, target_vel, ORIGIN, 0.0, V_MUZZLE);
  const auto aim_latency = tp::solve_intercept(target_pos, target_vel, ORIGIN, tau, V_MUZZLE);
  const double t0 = analytic_flight_time(target_pos, target_vel, 0.0, V_MUZZLE);
  const double t_tau = analytic_flight_time(target_pos, target_vel, tau, V_MUZZLE);
  EXPECT_TRUE(is_close(aim_no_latency.aim_pos.y(), 2.0 * t0, ITER3_REL_TOL));
  EXPECT_TRUE(is_close(aim_latency.aim_pos.y(), 2.0 * (tau + t_tau), ITER3_REL_TOL));
}

TEST(SolveIntercept, ZeroVelocityConvergesImmediately)
{
  const auto r = tp::solve_intercept({10.0, 0.0, 0.0}, ORIGIN, ORIGIN, 0.0, 25.0, 1);
  EXPECT_TRUE(is_close(r.t_flight, 10.0 / 25.0, 1e-6));
}

TEST(LatencyStat, RunningMean)
{
  tp::LatencyStat stat;
  for (double sample : {0.05, 0.07, 0.06}) {
    stat.add(sample);
  }
  EXPECT_EQ(stat.count(), 3);
  EXPECT_TRUE(is_close(stat.mean(), 0.06, 1e-6));
}

// ── plan_shot ─────────────────────────────────────────────────────────────

namespace
{

const Vector3d SHOOTER(0.0, 0.0, 0.4);
const double TICK_S = 1.0 / 30.0;
const std::array<double, 2> RADII = {0.30, 0.24};
const std::array<double, 2> FLAT = {0.0, 0.0};
const std::array<double, 2> STAGGER = {0.045, -0.045};

struct ArmorArgs
{
  Vector3d center{3.0, 0.0, 0.3};
  Vector3d vel{0.0, 0.0, 0.0};
  double yaw = M_PI;
  double w = 0.0;
};

ArmorState armor(const ArmorArgs & a = {})
{
  return {a.center.x(), a.center.y(), a.center.z(), a.vel.x(), a.vel.y(), a.vel.z(), a.yaw, a.w};
}

// Zero age, and a gimbal lag that puts the aim horizon (lag + half a tick) on
// the fire horizon: horizon is both.
tp::Shot plan(
  const ArmorState & state, double horizon, bool spinning,
  const std::array<double, 2> & z_offset = FLAT, ShotOptions opt = {})
{
  opt.gimbal_lag_s = horizon - TICK_S / 2.0;
  opt.firmware_latency_s = horizon;
  opt.min_fire_delay_s = 0.0;
  return tp::plan_shot(state, RADII, z_offset, 0.0, SHOOTER, V_MUZZLE, spinning, TICK_S, opt);
}

double flight(const Vector3d & aim) {return (aim - SHOOTER).norm() / V_MUZZLE;}

double dist(const Vector3d & a, const Vector3d & b) {return (a - b).norm();}

ShotOptions iters(int n)
{
  ShotOptions o;
  o.iterations = n;
  return o;
}

}  // namespace

TEST(PlanShot, NonSpinningStationaryAimsAtTheFacingPanelNow)
{
  const auto s = plan(armor(), 0.1, false);
  ASSERT_TRUE(s.delay_s);
  EXPECT_EQ(*s.delay_s, 0.0);
  EXPECT_LT(dist(s.aim_pos, {2.7, 0.0, 0.3}), 1e-9);
}

TEST(PlanShot, NonSpinningPicksTheFacingPanelAndItsPair)
{
  // Panel 0 faces away; panel 2 (same pair) faces the shooter, panel 1 or 3
  // (other pair) would at a quarter turn.
  ArmorArgs a;
  a.yaw = 0.0;
  EXPECT_LT(dist(plan(armor(a), 0.1, false, STAGGER).aim_pos, {2.7, 0.0, 0.345}), 1e-9);
  a.yaw = M_PI / 2.0;
  EXPECT_LT(dist(plan(armor(a), 0.1, false, STAGGER).aim_pos, {2.76, 0.0, 0.255}), 1e-9);
}

TEST(ChoosePanel, PicksTheMostFaceOnPanel)
{
  // bearing pi: panel k's normal is at k * pi/2 + yaw.
  EXPECT_EQ(tp::choose_panel(0.0, M_PI, std::nullopt, 0.0), 2);
  EXPECT_EQ(tp::choose_panel(M_PI / 2.0 + 0.1, M_PI, std::nullopt, 0.0), 1);
}

TEST(ChoosePanel, HoldsThePanelUntilAnotherIsClearlyBetter)
{
  const double margin = 0.2, bearing = M_PI;
  // Panel 1 (normal at yaw + pi/2) is held while panel 2 faces us more squarely by
  // up to the margin, whichever way the tracker's yaw jitters across 45 deg.
  for (double d : {-0.09, 0.0, 0.09}) {
    const double yaw = M_PI / 4.0 + d;
    const double held = tp::wrap_to_pi(yaw + M_PI / 2.0);
    EXPECT_EQ(tp::choose_panel(yaw, bearing, held, margin), 1) << d;
  }
  // Past the margin, the better panel takes over.
  const double yaw = M_PI / 4.0 - 0.11;
  EXPECT_EQ(tp::choose_panel(yaw, bearing, tp::wrap_to_pi(yaw + M_PI / 2.0), margin), 2);
}

TEST(ChoosePanel, FollowsTheHeldPanelByWorldYawNotIndex)
{
  // The same physical panel, labelled a quarter turn apart by the state's yaw.
  const double held = M_PI / 2.0 + 0.6;
  EXPECT_EQ(tp::choose_panel(0.6, M_PI, held, 0.5), 1);
  EXPECT_EQ(tp::choose_panel(0.6 - M_PI / 2.0, M_PI, held, 0.5), 2);
}

TEST(PlanShot, NonSpinningHoldsAPanelNear45DegreesAcrossTicks)
{
  ShotOptions opt;
  opt.panel_switch_margin_rad = 0.2;
  ArmorArgs a;
  // Panel 1 faces us first; the yaw then jitters either side of 45 deg.
  a.yaw = M_PI / 4.0 + 0.15;
  auto s = plan(armor(a), 0.1, false, STAGGER, opt);
  ASSERT_TRUE(s.panel_yaw);
  const Vector3d first = s.aim_pos;
  for (double d : {-0.05, 0.05, -0.08, 0.08}) {
    a.yaw = M_PI / 4.0 + d;
    opt.held_panel_yaw = s.panel_yaw;
    s = plan(armor(a), 0.1, false, STAGGER, opt);
    EXPECT_NEAR(s.aim_pos.z(), first.z(), 1e-9) << d;  // the same pair's height
  }
  // Without the hold the pick flips with the jitter.
  a.yaw = M_PI / 4.0 - 0.05;
  EXPECT_NE(plan(armor(a), 0.1, false, STAGGER).aim_pos.z(), first.z());
}

TEST(PlanShot, SpinningReportsNoPanelYaw)
{
  ArmorArgs a;
  a.w = 9.0;
  EXPECT_FALSE(plan(armor(a), 0.1, true).panel_yaw);
}

TEST(PlanShot, SpinningWithoutLeadIgnoresAPreviousPanelHold)
{
  ArmorArgs a;
  a.yaw = M_PI / 4.0 - 0.1;
  ShotOptions opt;
  opt.lead = false;
  opt.held_panel_yaw = M_PI * 3.0 / 4.0;
  opt.panel_switch_margin_rad = 0.5;
  const auto spinning = plan(armor(a), 0.1, true, STAGGER, opt);
  EXPECT_FALSE(spinning.panel_yaw);
  opt.held_panel_yaw.reset();
  EXPECT_LT((spinning.aim_pos - plan(armor(a), 0.1, true, STAGGER, opt).aim_pos).norm(), 1e-9);
}

TEST(ChoosePanel, HeldNormalSurvivesThePiWrap)
{
  EXPECT_EQ(tp::choose_panel(-M_PI + 0.02, M_PI, M_PI - 0.02, 0.2), 0);
}

TEST(PlanShot, NonSpinningCrossingPanelMeetsTheInterceptCondition)
{
  const double horizon = 0.12;
  ArmorArgs a;
  a.vel = {0.0, 2.0, 0.0};
  const auto s = plan(armor(a), horizon, false, FLAT, iters(50));
  const double t = analytic_flight_time(
    {2.7, 0.0, 0.3}, {0.0, 2.0, 0.0}, horizon, V_MUZZLE, SHOOTER);
  EXPECT_TRUE(is_close(s.aim_pos.y(), 2.0 * (horizon + t), 1e-6));
  EXPECT_TRUE(is_close((s.aim_pos - SHOOTER).norm(), V_MUZZLE * t, 1e-6));
}

TEST(PlanShot, NonSpinningLeadsTheTangentialPanelVelocity)
{
  // Panel facing the shooter on a slowly turning chassis moves sideways at r*w
  // even with a still center.
  ArmorArgs a;
  a.w = 1.0;
  EXPECT_LT(plan(armor(a), 0.1, false).aim_pos.y(), -0.02);  // yaw pi, w > 0: sweeps toward -y
}

namespace
{
double alignment_error(const ArmorState & state, double horizon, double delay)
{
  const auto aim = plan(state, horizon, true).aim_pos;
  const double yc = state[1], xc = state[0], yaw = state[6], w = state[7];
  const double t_impact = horizon + flight(aim) + delay;
  const double bearing = std::atan2(SHOOTER.y() - yc, SHOOTER.x() - xc);
  const double phase = tp::py_mod(yaw + w * t_impact - bearing, M_PI / 2.0);
  return std::min(phase, M_PI / 2.0 - phase);
}
}  // namespace

TEST(PlanShot, SpinningDelayLandsAPanelSquareToTheShooter)
{
  const double w = 2.0 * M_PI * 1.5;
  int fired = 0;
  for (int i = 0; i < 40; ++i) {
    const double yaw = M_PI + i * (M_PI / 2.0) / 40.0;
    for (double spin : {w, -w}) {
      ArmorArgs a;
      a.yaw = yaw;
      a.w = spin;
      const auto state = armor(a);
      const auto s = plan(state, 0.08, true);
      if (!s.delay_s) {
        continue;
      }
      ++fired;
      EXPECT_GE(*s.delay_s, 0.0);
      EXPECT_LT(*s.delay_s, TICK_S);
      EXPECT_LT(alignment_error(state, 0.08, *s.delay_s), 0.02);  // rad; flight-time residual
    }
  }
  // A tick-long window catches tick*|w| of each quarter turn, both directions.
  const double expected = 2 * 40 * (TICK_S * w) / (M_PI / 2.0);
  EXPECT_LE(std::abs(fired - expected), 4);
}

TEST(PlanShot, SpinningAimsOnTheCenterToShooterLine)
{
  ArmorArgs a;
  a.center = {3.0, 1.0, 0.3};
  a.w = 9.0;
  const auto aim = plan(armor(a), 0.1, true).aim_pos;
  const double to_shooter = std::atan2(SHOOTER.y() - 1.0, SHOOTER.x() - 3.0);
  EXPECT_TRUE(is_close(std::atan2(aim.y() - 1.0, aim.x() - 3.0), to_shooter, 0.0, 1e-9));
  const double r = std::hypot(aim.x() - 3.0, aim.y() - 1.0);
  EXPECT_TRUE(approx(r, RADII[0]) || approx(r, RADII[1]));
}

TEST(PlanShot, SpinningAimsAtTheArrivingPairsRadiusAndHeight)
{
  // Sweep the phase: whichever pair the delay lands on, the aim uses its
  // radius and height, and both pairs come up.
  const double w = 9.0;
  std::set<int> seen;
  for (int i = 0; i < 160; ++i) {
    ArmorArgs a;
    a.yaw = M_PI + i * (2.0 * M_PI) / 160.0;
    a.w = w;
    const auto state = armor(a);
    const auto s = plan(state, 0.08, true, STAGGER);
    if (!s.delay_s) {
      continue;
    }
    const double t_impact = 0.08 + flight(s.aim_pos) + *s.delay_s;
    const int k = static_cast<int>(
      tp::py_mod(std::nearbyint((M_PI - (state[6] + w * t_impact)) / (M_PI / 2.0)), 4.0));
    EXPECT_TRUE(is_close(std::hypot(s.aim_pos.x() - 3.0, s.aim_pos.y()), RADII[k % 2], 0.0, 1e-9));
    EXPECT_TRUE(is_close(s.aim_pos.z(), 0.3 + STAGGER[k % 2], 0.0, 1e-9));
    seen.insert(k % 2);
  }
  EXPECT_EQ(seen, (std::set<int>{0, 1}));
}

TEST(PlanShot, AimHorizonSetsTheLeadAndFireHorizonTheTiming)
{
  ArmorArgs a;
  a.vel = {0.0, 2.0, 0.0};
  a.w = 9.0;
  const auto state = armor(a);
  // Equal radii, so the pair each horizon picks can't move the aim.
  ShotOptions opt;
  opt.gimbal_lag_s = 0.03 - TICK_S / 2.0;
  opt.firmware_latency_s = 0.08;
  const auto near = tp::plan_shot(
    state, {0.27, 0.27}, FLAT, 0.0, SHOOTER, V_MUZZLE, true, TICK_S, opt);
  opt.gimbal_lag_s = 0.08 - TICK_S / 2.0;
  const auto far = tp::plan_shot(
    state, {0.27, 0.27}, FLAT, 0.0, SHOOTER, V_MUZZLE, true, TICK_S, opt);
  // The line to the shooter turns a little as the center moves, hence 1 cm.
  EXPECT_TRUE(is_close(far.aim_pos.y() - near.aim_pos.y(), 2.0 * 0.05, 0.0, 0.01));
  EXPECT_EQ(near.delay_s.has_value(), far.delay_s.has_value());
  if (near.delay_s && far.delay_s) {
    EXPECT_EQ(*near.delay_s, *far.delay_s);
  }
}

TEST(PlanShot, ChaseAimsAtTheFacingPanelAndFiresOnceSettled)
{
  const double w = 9.0;
  const double quarter = (M_PI / 2.0) / w;
  int fired = 0;
  for (int i = 0; i < 160; ++i) {
    ArmorArgs a;
    a.yaw = M_PI + i * (2.0 * M_PI) / 160.0;
    a.w = w;
    const auto state = armor(a);
    ShotOptions opt;
    opt.gimbal_lag_s = 0.02 - TICK_S / 2.0;
    opt.firmware_latency_s = 0.05;
    opt.chase_settle_s = 0.3 * quarter;
    opt.chase_margin_s = 0.1 * quarter;
    const auto s = tp::plan_shot(state, RADII, STAGGER, 0.0, SHOOTER, V_MUZZLE, true, TICK_S, opt);
    if (!s.delay_s) {
      continue;
    }
    ++fired;
    // It leaves mid-hold: aim horizon minus fire horizon, mod a tick.
    EXPECT_TRUE(approx(*s.delay_s, tp::py_mod(0.02 - 0.05, TICK_S)));
    // A fired shot's aim is where a panel facing the shooter (within 45 deg) is
    // at impact, on its circle.
    const double t_impact = 0.02 + flight(s.aim_pos);  // the aim horizon; mid-hold exit meets it
    double best_miss = 1e9, best_off = 0.0;
    for (int k = 0; k < 4; ++k) {
      const double yaw_k = state[6] + w * t_impact + k * M_PI / 2.0;
      const double miss = dist(
        s.aim_pos, {3.0 + RADII[k % 2] * std::cos(yaw_k), RADII[k % 2] * std::sin(yaw_k),
          0.3 + STAGGER[k % 2]});
      if (miss < best_miss) {  // tuple min in the Python: ties on miss take the lowest off
        best_miss = miss;
        best_off = std::abs(tp::py_mod(yaw_k - M_PI + M_PI, 2.0 * M_PI) - M_PI);
      }
    }
    EXPECT_LT(best_miss, 0.01);
    EXPECT_LT(best_off, 45.0 * M_PI / 180.0);
  }
  // Fires on the middle 60% of each panel's facing window.
  EXPECT_LE(std::abs(fired - 0.6 * 160), 4);
}

TEST(PlanShot, ExtrapolatesWithAcceleration)
{
  // A target braking at 6 m/s^2 from 2 m/s: the aim is on the parabola at
  // impact, and the intercept condition holds on it.
  const double horizon = 0.1;
  ArmorArgs a;
  a.vel = {0.0, 2.0, 0.0};
  ShotOptions opt = iters(50);
  opt.accel = {0.0, -6.0, 0.0};
  const auto s = plan(armor(a), horizon, false, FLAT, opt);
  const double t = horizon + flight(s.aim_pos);
  EXPECT_TRUE(is_close(s.aim_pos.y(), 2.0 * t - 3.0 * t * t, 0.0, 1e-9));
  EXPECT_TRUE(is_close(s.aim_pos.x(), 2.7, 0.0, 1e-9));
  const auto straight = plan(armor(a), horizon, false, FLAT, iters(50));
  EXPECT_GT(straight.aim_pos.y() - s.aim_pos.y(), 0.05);
}

TEST(PlanShot, WithoutLeadAimsAtTheCurrentEstimateAndFiresNow)
{
  ArmorArgs a;
  a.vel = {0.0, 4.0, 0.0};
  a.w = 9.0;
  ShotOptions opt;
  opt.lead = false;
  const auto s = plan(armor(a), 0.3, true, FLAT, opt);
  ASSERT_TRUE(s.delay_s);
  EXPECT_EQ(*s.delay_s, 0.0);
  EXPECT_LT(std::abs(s.aim_pos.y()), 1e-9);
}

namespace
{
Vector3d exit_point(double horizon, const Vector3d & shooter_vel)
{
  return SHOOTER + shooter_vel * horizon;
}

// A shot leaving shooter(horizon) toward the odom point gun at V_MUZZLE,
// carrying our velocity, t seconds later: what the aiming bench's harness flies.
Vector3d fly(const Vector3d & gun, double horizon, const Vector3d & shooter_vel, double t)
{
  const Vector3d start = exit_point(horizon, shooter_vel);
  const Vector3d d = gun - start;
  return start + (shooter_vel + V_MUZZLE * d / d.norm()) * t;
}
}  // namespace

TEST(PlanShot, MovingShooterLandsOnThePanel)
{
  // Our shot leaves where we are at the aim horizon and carries our velocity,
  // so the gun must point off the intercept by our motion to impact.
  const std::vector<Vector3d> shooter_vels = {{0.0, 1.0, 0.0}, {0.0, -2.0, 0.0}, {1.5, 0.5, 0.0}};
  const std::vector<Vector3d> target_vels = {{0.0, 0.0, 0.0}, {0.0, 2.0, 0.0}};
  for (const auto & shooter_vel : shooter_vels) {
    for (const auto & target_vel : target_vels) {
      const double horizon = 0.1;
      ArmorArgs a;
      a.vel = target_vel;
      const auto state = armor(a);
      ShotOptions opt = iters(50);
      opt.shooter_vel = shooter_vel;
      const Vector3d gun = plan(state, horizon, false, FLAT, opt).aim_pos;
      const Vector3d still = plan(state, horizon, false, FLAT, iters(50)).aim_pos;
      const double t = dist(gun, exit_point(horizon, shooter_vel)) / V_MUZZLE;
      const Vector3d panel(
        2.7 + target_vel.x() * (horizon + t), target_vel.y() * (horizon + t), 0.3);
      EXPECT_LT(dist(fly(gun, horizon, shooter_vel, t), panel), 1e-6);
      // Aiming as if still misses by our motion to impact: many panel widths.
      const double t_still = dist(still, exit_point(horizon, shooter_vel)) / V_MUZZLE;
      EXPECT_GT(dist(fly(still, horizon, shooter_vel, t_still), panel), 0.1);
    }
  }
}

TEST(PlanShot, MovingShooterChaseLandsOnAFacingPanel)
{
  const double w = 9.0;
  const Vector3d sv(0.0, 1.0, 0.0);
  ArmorArgs a;
  a.yaw = M_PI + 0.3;
  a.w = w;
  a.vel = {0.0, 1.0, 0.0};
  const auto state = armor(a);
  ShotOptions opt = iters(50);
  opt.gimbal_lag_s = 0.02 - TICK_S / 2.0;
  opt.firmware_latency_s = 0.05;
  opt.shooter_vel = sv;
  opt.chase_settle_s = 0.0;
  const auto s = tp::plan_shot(state, RADII, FLAT, 0.0, SHOOTER, V_MUZZLE, true, TICK_S, opt);
  ASSERT_TRUE(s.delay_s);
  const double t = dist(s.aim_pos, exit_point(0.02, sv)) / V_MUZZLE;
  const double t_impact = 0.02 + t;
  const Vector3d center(3.0, t_impact, 0.3);
  const Vector3d shot = fly(s.aim_pos, 0.02, sv, t);
  double miss = 1e9;
  for (int k = 0; k < 4; ++k) {
    const double yaw_k = state[6] + w * t_impact + k * M_PI / 2.0;
    miss = std::min(
      miss, dist(
        shot, {center.x() + RADII[k % 2] * std::cos(yaw_k),
          center.y() + RADII[k % 2] * std::sin(yaw_k), 0.3}));
  }
  EXPECT_LT(miss, 1e-6);
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

TEST(FireTiming, ExitNeverSendsUnderTheFloor)
{
  EXPECT_DOUBLE_EQ(tp::fire_exit_s(0.020, 0.040), 0.040);
  EXPECT_DOUBLE_EQ(tp::fire_exit_s(0.094, 0.040), 0.094);
  ShotOptions defaults;
  EXPECT_DOUBLE_EQ(defaults.min_fire_delay_s, 0.040);
  EXPECT_DOUBLE_EQ(tp::fire_exit_s(defaults.firmware_latency_s, defaults.min_fire_delay_s), 0.040);
  EXPECT_EQ(tp::wire_fire_delay_ms(
      0.0, tp::fire_exit_s(defaults.firmware_latency_s, defaults.min_fire_delay_s)), 40);
  EXPECT_EQ(tp::wire_fire_delay_ms(-0.001, 0.040), 40);
}

TEST(FireLock, MovingTargetWaitsOneSecondAndIdentityChangesRestart)
{
  tp::FireLock lock;
  const Vector3d moving(0.2, 0.0, 0.0);
  EXPECT_FALSE(lock.update(2.0, 7, true, false, moving, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(2.999, 7, true, false, moving, 1.0, 0.1, 0.3));
  EXPECT_TRUE(lock.update(3.0, 7, true, false, moving, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(3.1, 8, true, false, moving, 1.0, 0.1, 0.3));
  EXPECT_TRUE(lock.update(4.1, 8, true, false, moving, 1.0, 0.1, 0.3));
}

TEST(FireLock, LongIneligibleGapAndClockRewindRestart)
{
  tp::FireLock lock;
  EXPECT_FALSE(lock.update(1.0, 1, true, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_TRUE(lock.update(2.0, 1, true, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(2.1, 1, false, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(2.5, 1, true, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_TRUE(lock.update(3.5, 1, true, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(3.0, 1, true, true, ORIGIN, 1.0, 0.1, 0.3));
}

TEST(FireLock, StationaryBypassRequiresLowThreeDimensionalSpeedAndNoSpin)
{
  tp::FireLock lock;
  EXPECT_TRUE(lock.update(1.0, 1, true, false, {0.03, 0.04, 0.0}, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(1.1, 1, true, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(1.2, 1, true, false, {0.0, 0.0, 0.1}, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(1.3, 1, false, false, ORIGIN, 1.0, 0.1, 0.3));
}

TEST(FireLock, ZeroDisablesOnlyTheDwell)
{
  tp::FireLock lock;
  EXPECT_TRUE(lock.update(1.0, 1, true, true, ORIGIN, 0.0, 0.0, 0.3));
  EXPECT_FALSE(lock.update(1.1, 1, false, true, ORIGIN, 0.0, 0.0, 0.3));
}

TEST(FireLock, BriefGapCountsTowardDwellButNeverAllowsFire)
{
  tp::FireLock lock;
  EXPECT_FALSE(lock.update(1.0, 7, true, true, ORIGIN, 1.0, 0.1, 0.3));
  for (int frame = 0; frame < 5; ++frame) {
    EXPECT_FALSE(lock.update(1.5 + frame / 30.0, 7, false, true, ORIGIN, 1.0, 0.1, 0.3));
  }
  EXPECT_FALSE(lock.update(1.7, 7, true, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_TRUE(lock.update(2.0, 7, true, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(2.1, 7, false, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(2.2, 8, false, true, ORIGIN, 1.0, 0.1, 0.3));
  EXPECT_FALSE(lock.update(2.3, 7, true, true, ORIGIN, 1.0, 0.1, 0.3));
}

TEST(FireTiming, WireDelayPreservesAlignmentAfterFirmwareLatencySubtraction)
{
  EXPECT_EQ(tp::wire_fire_delay_ms(0.0, 0.094), 94);
  EXPECT_EQ(tp::wire_fire_delay_ms(0.02, 0.094) - 94, 20);
  EXPECT_EQ(tp::wire_fire_delay_ms(0.05, 0.094) - 94, 50);
  EXPECT_EQ(tp::wire_fire_delay_ms(70.0, 0.08), 65535);
}

TEST(PlanShot, OneHzAlignmentGatingHoldsYawAndMeetsTheExitHorizon)
{
  for (double w : {2 * M_PI, -2 * M_PI}) {
    int fired = 0;
    for (int i = 0; i < 160; ++i) {
      ArmorArgs a;
      a.center.x() = 1.3;
      a.yaw = M_PI + i * 2 * M_PI / 160;
      a.w = w;
      ShotOptions opt;
      opt.gimbal_lag_s = 0;
      const double age = .013, tick = .025;
      const auto s = tp::plan_shot(
        armor(a), {.24, .24}, FLAT, age, SHOOTER, V_MUZZLE, true, tick, opt);
      EXPECT_NEAR(s.aim_pos.y(), 0, 1e-9);
      if (!s.delay_s) {
        continue;
      }
      ++fired;
      EXPECT_GE(*s.delay_s, 0);
      EXPECT_LT(*s.delay_s, tick);
      const double impact = age + tp::fire_exit_s(opt.firmware_latency_s, opt.min_fire_delay_s) +
        *s.delay_s + flight(s.aim_pos);
      const double phase = tp::py_mod(a.yaw + w * impact - M_PI, M_PI / 2);
      EXPECT_LT(std::min(phase, M_PI / 2 - phase), 1e-8);
    }
    EXPECT_GT(fired, 0);
  }
}

TEST(PlanShot, SpinFirePolicyDoesNotChangeNonspinningMovingPrediction)
{
  for (const Vector3d & velocity : {Vector3d(2, 0, 0), Vector3d(0, 2, 0), Vector3d(1, 1, 0)}) {
    ArmorArgs a;
    a.vel = velocity;
    ShotOptions gated, chase;
    gated.gimbal_lag_s = chase.gimbal_lag_s = 0;
    chase.chase_settle_s = .025;
    chase.chase_margin_s = .025;
    const auto gate_shot = tp::plan_shot(
      armor(a), RADII, STAGGER, .02, SHOOTER, V_MUZZLE, false, .025, gated);
    const auto chase_shot = tp::plan_shot(
      armor(a), RADII, STAGGER, .02, SHOOTER, V_MUZZLE, false, .025, chase);
    EXPECT_LT((gate_shot.aim_pos - chase_shot.aim_pos).norm(), 1e-12);
    EXPECT_EQ(gate_shot.delay_s, chase_shot.delay_s);
    EXPECT_EQ(gate_shot.panel_yaw, chase_shot.panel_yaw);
  }
}
