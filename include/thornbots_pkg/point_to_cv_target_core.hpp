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

// Pure idle-look math for point_to_cv_target (no rclcpp), unit-tested
// standalone in test/test_point_to_cv_target_core.cpp. Aim planning and firing
// belong to the MCB; see README.md's ### point_to_cv_target.

#ifndef THORNBOTS_PKG__POINT_TO_CV_TARGET_CORE_HPP_
#define THORNBOTS_PKG__POINT_TO_CV_TARGET_CORE_HPP_

#include <optional>

#include "Eigen/Dense"

namespace thornbots_pkg
{

using Eigen::Vector3d;

// Python-style floor modulo: the result takes the divisor's sign.
double py_mod(double a, double b);

// Running mean/count of now-minus-detection-stamp latency samples (seconds).
class LatencyStat
{
public:
  void add(double sample_s);
  int count() const {return count_;}
  double mean() const {return mean_;}

private:
  int count_ = 0;
  double mean_ = 0.0;
};

// Wrap an angle to [-pi, pi).
double wrap_to_pi(double a);

// Idle look priority and bounded yaw changes. Times are capture/event seconds;
// gun yaw and points are in raw start-relative odom. Firmware owns hit turns.
class IdleLook
{
public:
  enum class Source {LastEnemy, Hit, Zone, Hold};
  Source source(double now_s, double target_timeout_s, bool hold) const;
  void target_seen(double stamp_s, const Vector3d & point);
  void hit(double stamp_s) {last_hit_s_ = stamp_s; reset();}
  void reset() {yaw_.reset(); desired_yaw_.reset();}
  Vector3d point(double now_s, const Vector3d & gun, double gun_yaw,
    const Vector3d & desired, double rate_rad_s, double switch_rad);
  const Vector3d & last_enemy() const {return last_enemy_;}

private:
  std::optional<double> last_target_s_, last_hit_s_, yaw_, desired_yaw_;
  Vector3d last_enemy_ = Vector3d::Zero();
  double last_step_s_ = 0.0;
};

Vector3d enemy_zone(
  bool blue, double red_x, double red_y, double blue_x, double blue_y, double height);

}  // namespace thornbots_pkg

#endif  // THORNBOTS_PKG__POINT_TO_CV_TARGET_CORE_HPP_
