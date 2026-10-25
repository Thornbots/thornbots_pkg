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

// Filter the robot's own head out of the lidar scan: blank a fixed angular
// sector of scan_raw where the head sits in the lidar's FOV (fixed in the
// lidar's frame; no joint-state sub needed), republishing on scan. Works for
// sim and real hardware.
//
// Current values: blind_angle_start=0.09, blind_angle_end=1.41 (5-81 deg CCW
// from the gun), from sentry_v2's CAD. See README.md for design rationale and
// retuning notes.

#include "thornbots_pkg/lidar_tracker_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace thornbots_pkg::lidar
{
namespace
{
double distance(Point a, Point b) {return std::hypot(a.x - b.x, a.y - b.y);}
Point predict(const Track & t, double stamp)
{
  return {t.position.x + t.velocity.x * (stamp - t.stamp),
    t.position.y + t.velocity.y * (stamp - t.stamp)};
}
}  // namespace
bool sweep_is_still(
  const std::vector<std::pair<double, double>> & head, double start, double end,
  double max_rate, double max_gap)
{
  if (head.size() < 2 || head.front().first > start || head.back().first < end) {return false;}
  for (size_t i = 1; i < head.size(); ++i) {
    const auto [a, angle_a] = head[i - 1];
    const auto [b, angle_b] = head[i];
    if (b < start || a > end) {continue;}
    if (b <= a || b - a > max_gap ||
      !std::isfinite(angle_a) || !std::isfinite(angle_b) ||
      std::abs(std::remainder(angle_b - angle_a, 2 * M_PI)) / (b - a) > max_rate)
    {return false;}
  }
  return true;
}
bool Grid::valid() const
{
  return width > 0 && height > 0 && std::isfinite(resolution) && resolution > 0 &&
         cells.size() == static_cast<size_t>(width) * height;
}
bool Grid::obstacle(Point p, double margin) const
{
  const double dx = p.x - x, dy = p.y - y;
  const double gx = (std::cos(yaw) * dx + std::sin(yaw) * dy) / resolution;
  const double gy = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / resolution;
  // Unmapped space is not evidence of a robot.
  if (gx < 0 || gy < 0 || gx >= width || gy >= height) {return true;}
  const int cx = static_cast<int>(gx), cy = static_cast<int>(gy);
  if (cells[cy * width + cx] < 0) {return true;}
  const int radius = static_cast<int>(std::ceil(margin / resolution)) + 1;
  for (int iy = std::max(0, cy - radius); iy <= std::min(height - 1, cy + radius); ++iy) {
    for (int ix = std::max(0, cx - radius); ix <= std::min(width - 1, cx + radius); ++ix) {
      if (cells[iy * width + ix] < 65) {continue;}
      const double ex = std::max({static_cast<double>(ix) - gx, 0.0, gx - (ix + 1)});
      const double ey = std::max({static_cast<double>(iy) - gy, 0.0, gy - (iy + 1)});
      if (std::hypot(ex, ey) * resolution <= margin) {return true;}
    }
  }
  return false;
}
void Tracker::clear() {tracks_.clear(); last_stamp_ = -1.0;}
std::vector<Track> Tracker::update(
  double stamp, const std::vector<std::optional<Point>> & rays, const Grid & map)
{
  if (!map.valid() || !std::isfinite(stamp)) {clear(); return {};}
  if (stamp < last_stamp_) {clear();}
  if (stamp == last_stamp_) {return snapshot(stamp);}
  last_stamp_ = stamp;
  std::vector<std::vector<Point>> clusters;
  std::vector<Point> cluster;
  auto finish = [&]() {
      if (!cluster.empty()) {clusters.push_back(std::move(cluster)); cluster.clear();}
    };
  for (const auto & p : rays) {
    if (!p || !std::isfinite(p->x) || !std::isfinite(p->y) ||
      map.obstacle(*p, parameters_.margin)) {finish(); continue;}
    if (!cluster.empty() && distance(cluster.back(), *p) > parameters_.cluster_gap) {finish();}
    cluster.push_back(*p);
  }
  finish();
  if (clusters.size() > 1 && rays.front() && rays.back() &&
    distance(clusters.back().back(), clusters.front().front()) <= parameters_.cluster_gap &&
    distance(*rays.front(), clusters.front().front()) < 1e-6 &&
    distance(*rays.back(), clusters.back().back()) < 1e-6)
  {
    clusters.back().insert(clusters.back().end(), clusters.front().begin(), clusters.front().end());
    clusters.erase(clusters.begin());
  }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & t) {
      return stamp - t.last_return > parameters_.keep_s;
    }), tracks_.end());
  std::vector<bool> used(tracks_.size(), false);
  for (const auto & points : clusters) {
    if (points.size() < 2) {continue;}
    double width = 0;
    Point center{};
    for (const auto & a : points) {
      center.x += a.x; center.y += a.y;
      for (const auto & b : points) {width = std::max(width, distance(a, b));}
    }
    if (width < parameters_.min_width || width > parameters_.max_width) {continue;}
    center.x /= points.size(); center.y /= points.size();
    size_t best = tracks_.size();
    double closest = parameters_.association_m;
    for (size_t i = 0; i < tracks_.size(); ++i) {
      const double d = distance(center, predict(tracks_[i], stamp));
      if (!used[i] && d < closest) {closest = d; best = i;}
    }
    if (best == tracks_.size()) {
      Track t;
      t.id = next_id_++; t.position = center; t.width = width;
      t.stamp = t.last_return = stamp;
      tracks_.push_back(t); used.push_back(true);
    } else {
      auto & t = tracks_[best];
      const Point predicted = predict(t, stamp);
      const double dt = stamp - t.stamp;
      t.velocity.x += 0.2 * (center.x - predicted.x) / dt;
      t.velocity.y += 0.2 * (center.y - predicted.y) / dt;
      t.position = {predicted.x + 0.7 * (center.x - predicted.x),
        predicted.y + 0.7 * (center.y - predicted.y)};
      t.stamp = t.last_return = stamp; t.width = width; used[best] = true;
    }
  }
  return snapshot(stamp);
}
std::vector<Track> Tracker::snapshot(double stamp) const
{
  auto result = tracks_;
  for (auto & t : result) {
    t.position = predict(t, stamp); t.stamp = stamp;
    if (stamp > t.label_until) {t.label = Label::Unchecked;}
  }
  return result;
}
void Tracker::camera(double stamp, Point origin, double yaw, std::optional<Point> enemy, bool allow_empty)
{
  size_t matched = tracks_.size();
  double nearest = parameters_.association_m;
  if (enemy) {
    for (size_t i = 0; i < tracks_.size(); ++i) {
      const double d = distance(predict(tracks_[i], stamp), *enemy);
      if (d < nearest) {nearest = d; matched = i;}
    }
  }
  for (size_t i = 0; i < tracks_.size(); ++i) {
    auto & t = tracks_[i];
    if (stamp > t.label_until) {t.label = Label::Unchecked;}
    if (i == matched) {
      t.label = Label::Enemy; t.label_until = stamp + parameters_.label_keep_s;
      t.looking_since = -1; continue;
    }
    const Point p = predict(t, stamp);
    const double bearing = std::atan2(p.y - origin.y, p.x - origin.x) - yaw;
    // Central 20 degrees only; "checked-empty" is a camera heuristic, not free space.
    const bool looking = allow_empty && std::abs(std::atan2(std::sin(bearing), std::cos(bearing))) < 0.175;
    if (!looking) {t.looking_since = -1; continue;}
    if (t.looking_since < 0) {t.looking_since = stamp;}
    if (stamp - t.looking_since >= parameters_.look_s && t.label != Label::Enemy) {
      t.label = Label::CheckedEmpty; t.label_until = stamp + parameters_.label_keep_s;
    }
  }
}
}  // namespace thornbots_pkg::lidar
