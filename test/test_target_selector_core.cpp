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

// Unit tests for target_selector_core.hpp's scoring/centrality/grouping/
// hysteresis. Synthetic inputs only; no rclcpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <vector>

#include "thornbots_pkg/target_selector_core.hpp"

namespace tp = thornbots_pkg;
using tp::ClusterInfo;
using tp::Point3;
using tp::RobotHysteresis;

namespace
{
const std::set<int> PRIORITY{2, 6};
const double MAX45 = 45.0 * M_PI / 180.0;

std::vector<int> sorted(std::vector<int> v)
{
  std::sort(v.begin(), v.end());
  return v;
}
}  // namespace

// ── team_from_ref_sys ─────────────────────────────────────────────────────

TEST(TeamFromRefSys, UnknownWithoutRobotIdEvenIfBlueBitReadsRed)
{
  EXPECT_FALSE(tp::team_from_ref_sys(0, false).has_value());
  EXPECT_FALSE(tp::team_from_ref_sys(0, true).has_value());
}

TEST(TeamFromRefSys, FromBlueBitOnceRobotIdIsSet)
{
  EXPECT_EQ(tp::team_from_ref_sys(7, true), std::optional<bool>(true));
  EXPECT_EQ(tp::team_from_ref_sys(7, false), std::optional<bool>(false));
}

// ── is_excluded_by_team ───────────────────────────────────────────────────

TEST(TeamFilter, UnknownPassesEverything)
{
  EXPECT_FALSE(tp::is_excluded_by_team(0, std::nullopt));
  EXPECT_FALSE(tp::is_excluded_by_team(7, std::nullopt));
}

TEST(TeamFilter, BlueExcludes0To3)
{
  for (int cid = 0; cid < 4; ++cid) {
    EXPECT_TRUE(tp::is_excluded_by_team(cid, true));
  }
  for (int cid = 4; cid < 8; ++cid) {
    EXPECT_FALSE(tp::is_excluded_by_team(cid, true));
  }
}

TEST(TeamFilter, RedExcludes4To7)
{
  for (int cid = 0; cid < 4; ++cid) {
    EXPECT_FALSE(tp::is_excluded_by_team(cid, false));
  }
  for (int cid = 4; cid < 8; ++cid) {
    EXPECT_TRUE(tp::is_excluded_by_team(cid, false));
  }
}

// ── centrality_3d ─────────────────────────────────────────────────────────

TEST(Centrality, BoresightIsOne)
{
  EXPECT_EQ(tp::centrality_3d(2.0, 0.0, 0.0, MAX45), 1.0);
}

TEST(Centrality, BehindCameraIsZero)
{
  EXPECT_EQ(tp::centrality_3d(-1.0, 0.0, 0.0, MAX45), 0.0);
  EXPECT_EQ(tp::centrality_3d(0.0, 0.0, 0.0, MAX45), 0.0);
}

TEST(Centrality, ClampedAtMaxAngle)
{
  const double x = 1.0;
  const double y = x * std::tan(MAX45);
  EXPECT_LT(std::abs(tp::centrality_3d(x, y, 0.0, MAX45)), 1e-9);
  // Beyond it: still clamped to 0, not negative.
  EXPECT_EQ(tp::centrality_3d(x, x * std::tan(MAX45 * 2.0), 0.0, MAX45), 0.0);
}

TEST(Centrality, MonotonicInAngle)
{
  const double c_near = tp::centrality_3d(2.0, 0.2, 0.0, MAX45);
  const double c_far = tp::centrality_3d(2.0, 1.0, 0.0, MAX45);
  EXPECT_GT(c_far, 0.0);
  EXPECT_LT(c_far, c_near);
  EXPECT_LT(c_near, 1.0);
}

// ── compute_score ─────────────────────────────────────────────────────────

TEST(ComputeScore, AdditiveNotMultiplicative)
{
  // The priority bonus is a conditional ADD, not conf*bonus or centrality*bonus.
  const double base = tp::compute_score(0.8, 0.5, 0, 1.0, 0.5, PRIORITY);
  const double with_bonus = tp::compute_score(0.8, 0.5, 2, 1.0, 0.5, PRIORITY);
  EXPECT_EQ(with_bonus, base + 0.5);
  EXPECT_EQ(base, 0.8 + 1.0 * 0.5);
}

TEST(ComputeScore, CenterWeightScalesOnlyTheCentralityTerm)
{
  const double s = tp::compute_score(0.8, 0.5, 0, 0.3, 0.5, PRIORITY);
  EXPECT_DOUBLE_EQ(s, 0.8 + 0.3 * 0.5);
  // A weight of 0 drops centrality entirely, not the confidence.
  const double s_zero = tp::compute_score(0.8, 0.5, 0, 0.0, 0.5, PRIORITY);
  EXPECT_DOUBLE_EQ(s_zero, 0.8);
  // The bonus is unweighted: it adds in full at any center_weight.
  const double s_bonus = tp::compute_score(0.8, 0.5, 2, 0.3, 0.5, PRIORITY);
  EXPECT_DOUBLE_EQ(s_bonus, s + 0.5);
}

TEST(ComputeScore, PriorityBonusOnlyForListedClasses)
{
  EXPECT_EQ(tp::compute_score(0.5, 0.5, 3, 1.0, 0.5, PRIORITY), 1.0);
}

// ── eligible ──────────────────────────────────────────────────────────────

TEST(Eligible, MinScoreGatesOnRawConfidenceNotComposite)
{
  EXPECT_FALSE(tp::eligible(0.1, 2, std::nullopt, 0.5));
  EXPECT_TRUE(tp::eligible(0.6, 2, std::nullopt, 0.5));
}

TEST(Eligible, RespectsTeamExclusionEvenAboveMinScore)
{
  EXPECT_FALSE(tp::eligible(0.99, 1, true, 0.0));
  EXPECT_TRUE(tp::eligible(0.99, 5, true, 0.0));
}

// ── group_panels (single-linkage clustering) ──────────────────────────────

TEST(GroupPanels, SingleRobotFourPanelsOneCluster)
{
  // Roughly a 0.30 x 0.24 chassis footprint, adjacent-pair spacing ~0.384 m.
  const std::vector<Point3> panels = {
    {2.0, 0.0, 0.0}, {1.7, 0.24, 0.0}, {1.4, 0.0, 0.0}, {1.7, -0.24, 0.0}};
  const auto clusters = tp::group_panels(panels, 0.4);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(sorted(clusters[0]), (std::vector<int>{0, 1, 2, 3}));
}

TEST(GroupPanels, TwoFarApartRobotsStaySeparate)
{
  const std::vector<Point3> panels = {{2.0, 0.0, 0.0}, {2.0, 3.0, 0.0}};
  EXPECT_EQ(tp::group_panels(panels, 0.4).size(), 2u);
}

TEST(GroupPanels, LinksAtExactlyTheRadiusAndNotBeyond)
{
  // 3-4-5 triple scaled by 0.08: the distance is exactly 0.4 in binary
  // floating point, so this pins the <= comparison.
  const std::vector<Point3> at_radius = {{0.0, 0.0, 0.0}, {0.24, 0.32, 0.0}};
  EXPECT_EQ(tp::group_panels(at_radius, 0.4).size(), 1u);

  const std::vector<Point3> beyond = {{0.0, 0.0, 0.0}, {0.25, 0.33, 0.0}};
  EXPECT_EQ(tp::group_panels(beyond, 0.4).size(), 2u);
}

TEST(GroupPanels, MergesTwoRobotsWithCloseNearestPanels)
{
  // Single-linkage's known cost: two robots 0.3 m apart at their nearest
  // panels come back as ONE cluster.
  std::vector<Point3> a_and_b = {
    {2.0, 0.0, 0.0}, {1.7, 0.24, 0.0}, {2.3, 0.0, 0.0}, {2.6, 0.24, 0.0}};
  auto clusters = tp::group_panels(a_and_b, 0.4);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(sorted(clusters[0]), (std::vector<int>{0, 1, 2, 3}));

  // Widen the bridge past the radius and they separate.
  std::vector<Point3> b_far = {
    {2.0, 0.0, 0.0}, {1.7, 0.24, 0.0}, {2.5, 0.0, 0.0}, {2.8, 0.24, 0.0}};
  EXPECT_EQ(tp::group_panels(b_far, 0.4).size(), 2u);
}

TEST(ClusterCentroid, IsMeanPosition)
{
  const std::vector<Point3> panels = {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  const Point3 c = tp::cluster_centroid(panels, {0, 1});
  EXPECT_EQ(c.x, 1.0);
  EXPECT_EQ(c.y, 0.0);
  EXPECT_EQ(c.z, 0.0);
}

// ── RobotHysteresis (cluster keys: a=1, b=2, c=3) ─────────────────────────

namespace
{
constexpr int A = 1, B = 2, C = 3;
}

TEST(Hysteresis, AcquiresImmediatelyWithNoIncumbent)
{
  RobotHysteresis h(0.3, 3);
  const auto winner = h.update({{A, {2.0, 0.0, 0.0}, 1.0}});
  EXPECT_EQ(winner, std::optional<int>(A));
  EXPECT_EQ(h.track_id(), 1u);
}

TEST(Hysteresis, HoldsIncumbentBelowSwitchMargin)
{
  RobotHysteresis h(0.3, 2);
  h.update({{A, {2.0, 0.0, 0.0}, 1.0}});
  const auto first_id = h.track_id();
  std::optional<int> winner;
  for (int i = 0; i < 5; ++i) {
    winner = h.update({{A, {2.0, 0.0, 0.0}, 1.0}, {B, {2.0, 5.0, 0.0}, 1.1}});
  }
  EXPECT_EQ(winner, std::optional<int>(A));
  EXPECT_EQ(h.track_id(), first_id);
}

TEST(Hysteresis, SwitchesAfterSustainedStrongerChallenger)
{
  RobotHysteresis h(0.3, 3);
  h.update({{A, {2.0, 0.0, 0.0}, 1.0}});
  const auto first_id = h.track_id();
  std::optional<int> winner;
  for (int i = 0; i < 3; ++i) {
    winner = h.update({{A, {2.0, 0.0, 0.0}, 1.0}, {B, {2.0, 5.0, 0.0}, 2.0}});
  }
  EXPECT_EQ(winner, std::optional<int>(B));
  EXPECT_EQ(h.track_id(), first_id + 1);
}

TEST(Hysteresis, StreakResetsWhenChallengerMarginLapses)
{
  // A challenger must clear switch_margin on CONSECUTIVE frames; one
  // sub-margin frame sends the streak back to 0.
  RobotHysteresis h(0.3, 3);
  h.update({{A, {2.0, 0.0, 0.0}, 1.0}});
  const auto first_id = h.track_id();
  auto frame = [&h](double challenger_score) {
      return h.update({{A, {2.0, 0.0, 0.0}, 1.0}, {B, {2.0, 5.0, 0.0}, challenger_score}});
    };
  EXPECT_EQ(frame(2.0), std::optional<int>(A));  // streak 1
  EXPECT_EQ(frame(1.1), std::optional<int>(A));  // ahead but under margin -> streak 0
  EXPECT_EQ(frame(2.0), std::optional<int>(A));  // streak 1 again
  EXPECT_EQ(frame(2.0), std::optional<int>(A));  // streak 2 -- would have been the switch
  EXPECT_EQ(h.track_id(), first_id);
  EXPECT_EQ(frame(2.0), std::optional<int>(B));  // streak 3 -> switch
  EXPECT_EQ(h.track_id(), first_id + 1);
}

TEST(Hysteresis, ReacquiresWhenIncumbentLost)
{
  RobotHysteresis h(0.3, 3, 1.0);
  h.update({{A, {2.0, 0.0, 0.0}, 1.0}});
  const auto first_id = h.track_id();
  // Only candidate now is far outside gate_radius_m of the old incumbent.
  const auto winner = h.update({{C, {2.0, 10.0, 0.0}, 0.5}});
  EXPECT_EQ(winner, std::optional<int>(C));
  EXPECT_EQ(h.track_id(), first_id + 1);
}

TEST(Hysteresis, EmptyClustersReturnsNone)
{
  RobotHysteresis h(0.3, 3);
  EXPECT_FALSE(h.update({}).has_value());
}
