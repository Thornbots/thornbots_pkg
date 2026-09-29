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

// Armor-model EKF for target_tracker (no rclcpp). State [pos, vel, acc, yaw,
// w, r, dz]: chassis centre, velocity and acceleration in odom, the tracked
// panel's outward-normal yaw, spin rate, that panel's radius and its pair's
// height above the centre (the other pair at -dz). Measurement is one panel:
// h = pos + r (cos yaw, sin yaw, 0) + (0, 0, dz). See README.md's
// ### target_tracker.

#ifndef THORNBOTS_PKG__ARMOR_TRACKER_HPP_
#define THORNBOTS_PKG__ARMOR_TRACKER_HPP_

#include <array>
#include <optional>
#include <utility>
#include <vector>

#include "Eigen/Dense"

namespace thornbots_pkg
{

constexpr int POS = 0, VEL = 3, ACC = 6;  // 3-vector blocks
constexpr int YAW = 9, W = 10, R = 11, DZ = 12;
constexpr int N_STATE = 13;
constexpr double DZ_MAX = 0.15;  // m, clamp on the pair height offset
constexpr double BACK_FACING_COS = -0.3;  // association skips panels facing further away

using State = Eigen::Matrix<double, N_STATE, 1>;
using Cov = Eigen::Matrix<double, N_STATE, N_STATE>;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// depth_std along the camera->panel ray, lateral_std across it.
Matrix3d ray_covariance(
  const Vector3d & panel_pos, const Vector3d & camera_pos, double depth_std, double lateral_std);

struct Panel
{
  int k;
  double yaw;
  Vector3d pos;
};

// All 4 panels, k = 0 the tracked one.
std::array<Panel, 4> panel_positions(const State & state, double other_r);

// q_accel (m/s^2) drives the centre, q_jerk (m/s^3) a Singer acceleration
// (0 = constant velocity), q_yaw_accel (rad/s^2) the spin. still pins
// velocity, acceleration and spin at 0; the centre and yaw random-walk.
struct EkfParams
{
  double radius = 0.27;
  double q_accel = 2.0;
  double q_yaw_accel = 5.0;
  double q_radius = 0.02;
  double r_min = 0.18;
  double r_max = 0.45;
  double spin_prior = 0.0;
  double spin_prior_std = 8.0;
  double q_dz = 0.005;
  double dz_prior_std = 0.05;
  double q_jerk = 0.0;
  double accel_tau_s = 0.5;
  double accel_prior_std = 6.0;
  bool still = false;
  double q_still_pos = 0.02;
  double q_still_yaw = 0.05;
};

enum class StepResult { Update, Outlier, Reacquire };

class ArmorEKF
{
public:
  ArmorEKF(
    const Vector3d & panel_pos, const Vector3d & camera_pos, double t_sec,
    const Matrix3d & pos_var, const EkfParams & params);

  // Re-seed centre, velocity and yaw from one panel assumed to face the
  // camera; keep_spin keeps w, both radii and dz.
  void reacquire(
    const Vector3d & panel_pos, const Vector3d & camera_pos, double t_sec,
    const Matrix3d & pos_var, bool keep_spin = true);
  void pin_still();
  std::pair<State, Cov> predicted(double t_sec) const;
  void predict(double t_sec);
  // Re-label onto the nearest front-facing panel; returns (k, distance), k -1 if none.
  std::pair<int, double> associate(const Vector3d & panel_pos, const Vector3d & camera_pos);
  // facing_std > 0: the panel was seen alone (update_facing).
  StepResult step(
    const Vector3d & panel_pos, const Vector3d & camera_pos, double t_sec,
    const Matrix3d & pos_var, double gate_nis = 16.3, int max_outliers = 3,
    double facing_std = 0.0);
  void update_facing(const Vector3d & camera_pos, double std);
  void update(const Vector3d & panel_pos, const Matrix3d & pos_var);
  double nis(const Vector3d & panel_pos, const Matrix3d & pos_var) const;

  EkfParams p;
  State state;
  Cov P;
  double other_r;
  double t_sec;
  int n_outliers = 0;
  double last_nis = 0.0;
  double last_logdet = 0.0;  // log det of the last innovation covariance

private:
  std::pair<Cov, Cov> transition(double dt) const;
  // (NIS, log det S) of panel_pos against the current state.
  std::pair<double, double> innovation(const Vector3d & panel_pos, const Matrix3d & pos_var) const;
};

// Bank of ArmorEKFs seeded at different spin rates, plus a still one; the
// likeliest (EWMA of NIS + log det S) leads. See README.md.
struct TrackerParams
{
  EkfParams ekf;
  std::vector<double> spin_priors{0.0, 7.0, -7.0, 13.0, -13.0};
  double prior_std = 3.0;
  double alpha = 0.03;
  double reseed_margin = 3.0;
  double reseed_after_s = 1.0;
  double switch_margin = 1.0;
  double switch_after_s = 0.5;
  bool still = true;
  double still_margin = 0.25;
  double still_exit_llr = 15.0;
};

class ArmorTracker
{
public:
  ArmorTracker(
    const Vector3d & panel_pos, const Vector3d & camera_pos, double t_sec,
    const Matrix3d & pos_var, const TrackerParams & params);

  StepResult step(
    const Vector3d & panel_pos, const Vector3d & camera_pos, double t_sec,
    const Matrix3d & pos_var, double gate_nis = 16.3, int max_outliers = 3,
    double facing_std = 0.0);

  const ArmorEKF & best() const {return filters[lead];}
  const State & state() const {return best().state;}
  double other_r() const {return best().other_r;}
  std::pair<State, Cov> predicted(double t_sec) const {return best().predicted(t_sec);}

  TrackerParams p;
  std::vector<ArmorEKF> filters;
  std::vector<double> scores;  // empty until the first step
  std::vector<std::optional<double>> worse_since;
  size_t lead = 0;

private:
  double still_cusum_ = 0.0;
  std::optional<std::pair<size_t, double>> challenger_;  // (index, since t_sec)
};

}  // namespace thornbots_pkg

#endif  // THORNBOTS_PKG__ARMOR_TRACKER_HPP_
