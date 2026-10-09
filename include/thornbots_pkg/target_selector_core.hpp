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

// Pure scoring/centrality/grouping/hysteresis logic for target_selector (no
// rclcpp). See README.md's ### target_selector Notes for the clustering-rule
// tradeoff and scoring history.

#ifndef THORNBOTS_PKG__TARGET_SELECTOR_CORE_HPP_
#define THORNBOTS_PKG__TARGET_SELECTOR_CORE_HPP_

#include <cstdint>
#include <optional>
#include <set>
#include <vector>

namespace thornbots_pkg
{

struct Point3
{
  double x, y, z;
};

// Our team colour from a RefSysStatus: true blue, false red, nullopt unknown.
// robot_id 0 is taproot's RobotId::INVALID (sent before the referee assigns
// an ID); its blue bit reads 0, so without this check no colour would look red.
std::optional<bool> team_from_ref_sys(int robot_id, bool is_on_blue_team);

// True if class_id is our own team's (blue 0-3, red 4-7). nullopt team (no
// RefSysStatus yet) never excludes.
bool is_excluded_by_team(int class_id, std::optional<bool> is_blue_team);

// Bearing off the camera's forward (+x) axis, 1.0 at boresight, 0.0 at
// max_angle_rad or beyond; x <= 0 (behind the camera) scores 0.
double centrality_3d(double x, double y, double z, double max_angle_rad);

// score = confidence + center_weight * centrality, plus priority_class_bonus
// (added, not multiplied) when class_id is a priority class.
double compute_score(
  double confidence, double centrality, int class_id, double center_weight,
  double priority_class_bonus, const std::set<int> & priority_class_ids);

// Confidence gate on raw confidence only, after the team filter.
bool eligible(double confidence, int class_id, std::optional<bool> is_blue_team, double min_score);

// Single-linkage clusters of panels by 3D distance <= radius_m (inclusive),
// as index lists; clusters are ordered by their lowest index.
std::vector<std::vector<int>> group_panels(const std::vector<Point3> & panels, double radius_m);

Point3 cluster_centroid(const std::vector<Point3> & panels, const std::vector<int> & indices);

struct ClusterInfo
{
  int key;
  Point3 centroid;
  double score;
};

// Robot-level switch hysteresis: a challenger must beat the incumbent's score
// by switch_margin for switch_hold_frames consecutive frames. Continuity is by
// nearest centroid to the incumbent (within gate_radius_m, else re-acquire);
// acquiring from no incumbent is immediate.
class RobotHysteresis
{
public:
  RobotHysteresis(double switch_margin, int switch_hold_frames, double gate_radius_m = 1.0);

  // Winning cluster's key, or nullopt if clusters is empty.
  std::optional<int> update(const std::vector<ClusterInfo> & clusters);

  uint32_t track_id() const {return track_id_;}  // 0 = no incumbent yet

private:
  void acquire(const ClusterInfo & cluster);

  double switch_margin_;
  int switch_hold_frames_;
  double gate_radius_m_;
  uint32_t track_id_ = 0;
  std::optional<Point3> incumbent_centroid_;
  int challenge_streak_ = 0;
  uint32_t next_id_ = 1;
};

}  // namespace thornbots_pkg

#endif  // THORNBOTS_PKG__TARGET_SELECTOR_CORE_HPP_
