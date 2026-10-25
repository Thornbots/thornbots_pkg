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

#include <cmath>
#include <optional>
#include <vector>
#include "gtest/gtest.h"

using thornbots_pkg::lidar::Grid;
using thornbots_pkg::lidar::Label;
using thornbots_pkg::lidar::Point;
using thornbots_pkg::lidar::Tracker;
namespace
{
Grid field()
{
  Grid map;
  map.width = map.height = 100; map.resolution = 0.1;
  map.cells.resize(10000, 0);
  return map;
}
std::vector<std::optional<Point>> head(double x, double y = 2)
{
  return {Point{x, y - 0.1}, Point{x, y}, Point{x, y + 0.1}, std::nullopt};
}
}  // namespace
TEST(LidarTracker, MapAndCellExtent)
{
  auto map = field(); map.cells[20 * 100 + 20] = 100;
  EXPECT_TRUE(map.obstacle({2.34, 2.05}, 0.25));
  EXPECT_FALSE(map.obstacle({2.36, 2.05}, 0.25));
  map.yaw = M_PI / 2; map.x = 5;
  EXPECT_TRUE(map.obstacle({2.95, 2.34}, 0.25));
  EXPECT_TRUE(map.obstacle({11, 11}, 0.25));
  EXPECT_TRUE(Tracker().update(0, head(3), {}).empty());
}
TEST(LidarTracker, MarginRejectsNearWallButKeepsRobot)
{
  auto map = field();
  for (int y = 0; y < 100; ++y) {map.cells[y * 100 + 20] = 100;}
  EXPECT_TRUE(Tracker().update(0, head(2.3), map).empty());
  EXPECT_EQ(Tracker().update(0, head(2.5), map).size(), 1U);
  thornbots_pkg::lidar::Parameters parameters; parameters.margin = 0.4;
  EXPECT_TRUE(Tracker(parameters).update(0, head(2.49), map).empty());
}
TEST(LidarTracker, VelocityDropoutsAndExpiry)
{
  Tracker tracker; auto map = field();
  uint32_t id = 0;
  for (int i = 0; i < 30; ++i) {
    const auto tracks = tracker.update(i * 0.1, head(3 + i * 0.05), map);
    ASSERT_EQ(tracks.size(), 1U);
    if (i == 0) {id = tracks[0].id;}
    EXPECT_EQ(tracks[0].id, id);
    if (i == 29) {EXPECT_NEAR(tracks[0].velocity.x, 0.5, 0.02);}
  }
  auto tracks = tracker.update(3.9, {}, map);
  ASSERT_EQ(tracks.size(), 1U); EXPECT_NEAR(tracks[0].position.x, 4.95, 0.03);
  EXPECT_DOUBLE_EQ(tracks[0].last_return, 2.9);
  EXPECT_TRUE(tracker.update(6.0, {}, map).empty());
}
TEST(LidarTracker, LabelsAndClockReset)
{
  Tracker tracker; auto map = field();
  tracker.update(0, head(3), map);
  tracker.camera(0, {0, 2}, 0, std::nullopt);
  tracker.camera(0.7, {0, 2}, 0, std::nullopt);
  EXPECT_EQ(tracker.update(0.7, head(3), map)[0].label, Label::CheckedEmpty);
  tracker.camera(0.8, {0, 2}, 0, Point{3, 2});
  EXPECT_EQ(tracker.update(0.8, head(3), map)[0].label, Label::Enemy);
  tracker.camera(4, {0, 2}, M_PI, std::nullopt);
  EXPECT_EQ(tracker.update(4, head(3), map)[0].label, Label::Unchecked);
  const auto id = tracker.update(0, head(3), map)[0].id;
  EXPECT_NE(id, 1U);
}
TEST(LidarTracker, NoiseAndOversizeDoNotBecomeTracks)
{
  Tracker tracker; auto map = field();
  EXPECT_TRUE(tracker.update(0, {Point{2, 2}}, map).empty());
  std::vector<std::optional<Point>> wall;
  for (int i = 0; i < 20; ++i) {wall.push_back(Point{3, 2 + i * 0.1});}
  EXPECT_TRUE(tracker.update(1, wall, map).empty());
}
TEST(LidarTracker, TwoObjectsAssociateOnlyOnceAndSeamJoins)
{
  Tracker tracker; auto map = field();
  auto rays = head(3); const auto other = head(4); rays.insert(rays.end(), other.begin(), other.end());
  EXPECT_EQ(tracker.update(0, rays, map).size(), 2U);
  EXPECT_EQ(tracker.update(0.1, rays, map).size(), 2U);
  Tracker seam;
  EXPECT_EQ(seam.update(0, {Point{3, 2}, Point{3, 2.1}, std::nullopt,
      Point{3, 1.9}}, map).size(), 1U);
}
TEST(LidarTracker, SmearedAndUnknownSweepsAreRejected)
{
  using thornbots_pkg::lidar::sweep_is_still;
  EXPECT_FALSE(sweep_is_still({}, 0, 0.09, 0.5));
  EXPECT_TRUE(sweep_is_still({{0, 0}, {0.05, 0.01}, {0.1, 0.02}}, 0.01, 0.09, 0.5));
  EXPECT_FALSE(sweep_is_still({{0, 0}, {0.05, 0.1}, {0.1, 0.2}}, 0.01, 0.09, 0.5));
  EXPECT_FALSE(sweep_is_still({{0, 0}, {0.2, 0}}, 0.01, 0.09, 0.5));
  EXPECT_FALSE(sweep_is_still({{0.05, 0}, {0.1, 0}}, 0.01, 0.09, 0.5));
  EXPECT_TRUE(sweep_is_still({{0, 3.14}, {0.05, -3.14}}, 0, 0.05, 0.5));
}
TEST(LidarTracker, MissingCameraActivityDoesNotMarkEmpty)
{
  Tracker tracker; auto map = field();
  tracker.update(0, head(3), map);
  tracker.camera(0, {0, 2}, 0, std::nullopt, false);
  tracker.camera(1, {0, 2}, 0, std::nullopt, false);
  EXPECT_EQ(tracker.snapshot(1)[0].label, Label::Unchecked);
}
