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

// ROS-free core of lidar_tracker: drop scan points near saved-map obstacles,
// cluster the rest into 0.10-0.40 m robot heads and track them in map with an
// alpha-beta filter; camera views label tracks. Design: README.md#lidar_tracker.

#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace thornbots_pkg::lidar
{
bool sweep_is_still(
  const std::vector<std::pair<double, double>> & head, double start, double end,
  double max_rate, double max_gap = 0.1);
struct Point {double x{}, y{};};
struct Grid
{
  int width{}, height{};
  double resolution{}, x{}, y{}, yaw{};
  std::vector<int8_t> cells;
  bool valid() const;
  bool obstacle(Point point, double margin) const;
};
enum class Label : uint8_t {Unchecked, Enemy, CheckedEmpty};
struct Track
{
  uint32_t id{};
  Point position, velocity;
  double stamp{}, last_return{}, width{}, label_until{}, looking_since{-1.0};
  Label label{Label::Unchecked};
};
struct Parameters
{
  double margin{0.25}, min_width{0.10}, max_width{0.40}, cluster_gap{0.18};
  double keep_s{3.0}, association_m{0.65}, label_keep_s{3.0}, look_s{0.6};
  double min_separation_m{0.4};  // closer clusters in one sweep are one robot
};
class Tracker
{
public:
  explicit Tracker(Parameters parameters = {})
  : parameters_(parameters) {}
  std::vector<Track> update(
    double stamp, const std::vector<std::optional<Point>> & rays, const Grid & map);
  void camera(
    double stamp, Point origin, double yaw, std::optional<Point> enemy,
    bool allow_empty = true);
  void clear();
  std::vector<Track> snapshot(double stamp) const;

private:
  Parameters parameters_;
  std::vector<Track> tracks_;
  uint32_t next_id_{1};
  double last_stamp_{-1.0};
};
}  // namespace thornbots_pkg::lidar
