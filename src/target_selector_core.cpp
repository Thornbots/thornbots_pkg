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

#include "thornbots_pkg/target_selector_core.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>

namespace thornbots_pkg
{

std::optional<bool> team_from_ref_sys(int robot_id, bool is_on_blue_team)
{
  if (robot_id == 0) {
    return std::nullopt;
  }
  return is_on_blue_team;
}

bool is_excluded_by_team(int class_id, std::optional<bool> is_blue_team)
{
  if (!is_blue_team) {
    return false;
  }
  if (*is_blue_team) {
    return 0 <= class_id && class_id <= 3;
  }
  return 4 <= class_id && class_id <= 7;
}

double centrality_3d(double x, double y, double z, double max_angle_rad)
{
  if (x <= 0.0 || max_angle_rad <= 0.0) {
    return 0.0;
  }
  const double angle = std::atan2(std::hypot(y, z), x);
  const double c = 1.0 - angle / max_angle_rad;
  return std::max(0.0, std::min(1.0, c));
}

double compute_score(
  double confidence, double centrality, int class_id, double center_weight,
  double priority_class_bonus, const std::set<int> & priority_class_ids)
{
  double score = confidence + center_weight * centrality;
  if (priority_class_ids.count(class_id)) {
    score += priority_class_bonus;
  }
  return score;
}

bool eligible(double confidence, int class_id, std::optional<bool> is_blue_team, double min_score)
{
  if (is_excluded_by_team(class_id, is_blue_team)) {
    return false;
  }
  return confidence >= min_score;
}

std::vector<std::vector<int>> group_panels(const std::vector<Point3> & panels, double radius_m)
{
  const int n = static_cast<int>(panels.size());
  std::vector<int> parent(n);
  std::iota(parent.begin(), parent.end(), 0);
  auto find = [&parent](int i) {
      while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
      }
      return i;
    };
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const double dx = panels[i].x - panels[j].x;
      const double dy = panels[i].y - panels[j].y;
      const double dz = panels[i].z - panels[j].z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) <= radius_m) {
        const int ri = find(i), rj = find(j);
        if (ri != rj) {
          parent[ri] = rj;
        }
      }
    }
  }
  // Ordered by first member, like the insertion order of the root's first index.
  std::vector<std::vector<int>> clusters;
  std::map<int, size_t> slot;
  for (int i = 0; i < n; ++i) {
    const int root = find(i);
    auto it = slot.find(root);
    if (it == slot.end()) {
      it = slot.emplace(root, clusters.size()).first;
      clusters.emplace_back();
    }
    clusters[it->second].push_back(i);
  }
  return clusters;
}

Point3 cluster_centroid(const std::vector<Point3> & panels, const std::vector<int> & indices)
{
  Point3 sum{0.0, 0.0, 0.0};
  for (int i : indices) {
    sum.x += panels[i].x;
    sum.y += panels[i].y;
    sum.z += panels[i].z;
  }
  const double n = static_cast<double>(indices.size());
  return {sum.x / n, sum.y / n, sum.z / n};
}

RobotHysteresis::RobotHysteresis(double switch_margin, int switch_hold_frames, double gate_radius_m)
: switch_margin_(switch_margin), switch_hold_frames_(switch_hold_frames),
  gate_radius_m_(gate_radius_m) {}

std::optional<int> RobotHysteresis::update(const std::vector<ClusterInfo> & clusters)
{
  if (clusters.empty()) {
    return std::nullopt;
  }
  // First of equal maxima wins, as in the Python max().
  const ClusterInfo * best = &clusters[0];
  for (const auto & c : clusters) {
    if (c.score > best->score) {
      best = &c;
    }
  }

  if (!incumbent_centroid_) {
    acquire(*best);
    return best->key;
  }

  const Point3 inc = *incumbent_centroid_;
  auto dist = [&inc](const ClusterInfo & c) {
      return std::sqrt(
        (c.centroid.x - inc.x) * (c.centroid.x - inc.x) +
        (c.centroid.y - inc.y) * (c.centroid.y - inc.y) +
        (c.centroid.z - inc.z) * (c.centroid.z - inc.z));
    };
  const ClusterInfo * match = &clusters[0];
  double match_dist = dist(clusters[0]);
  for (const auto & c : clusters) {
    const double d = dist(c);
    if (d < match_dist) {
      match = &c;
      match_dist = d;
    }
  }
  if (match_dist > gate_radius_m_) {
    acquire(*best);  // incumbent lost track entirely
    return best->key;
  }

  if (best == match || best->score <= match->score) {
    challenge_streak_ = 0;
    incumbent_centroid_ = match->centroid;
    return match->key;
  }

  if (best->score >= match->score + switch_margin_) {
    ++challenge_streak_;
    if (challenge_streak_ >= switch_hold_frames_) {
      acquire(*best);
      return best->key;
    }
  } else {
    challenge_streak_ = 0;
  }

  incumbent_centroid_ = match->centroid;
  return match->key;
}

void RobotHysteresis::acquire(const ClusterInfo & cluster)
{
  track_id_ = next_id_++;
  challenge_streak_ = 0;
  incumbent_centroid_ = cluster.centroid;
}

}  // namespace thornbots_pkg
