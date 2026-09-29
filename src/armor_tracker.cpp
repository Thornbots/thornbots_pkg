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

#include "thornbots_pkg/armor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace thornbots_pkg
{

namespace
{

constexpr double QUARTER_TURN = M_PI / 2.0;

Vector3d offset(double yaw, double r, double dz)
{
  return Vector3d(r * std::cos(yaw), r * std::sin(yaw), dz);
}

// Zero row and column idx..idx+n of P.
void zero_rows_cols(Cov & P, int idx, int n)
{
  P.middleRows(idx, n).setZero();
  P.middleCols(idx, n).setZero();
}

// Index of the first minimum, as Python's min() and np.argmin pick.
template<typename Key>
size_t first_min(const std::vector<size_t> & idx, Key key)
{
  size_t best = idx.front();
  for (size_t i : idx) {
    if (key(i) < key(best)) {
      best = i;
    }
  }
  return best;
}

}  // namespace

Matrix3d ray_covariance(
  const Vector3d & panel_pos, const Vector3d & camera_pos, double depth_std, double lateral_std)
{
  const Vector3d ray = panel_pos - camera_pos;
  const Vector3d u = ray / (ray.norm() + 1e-9);
  return lateral_std * lateral_std * Matrix3d::Identity() +
         (depth_std * depth_std - lateral_std * lateral_std) * u * u.transpose();
}

std::array<Panel, 4> panel_positions(const State & state, double other_r)
{
  std::array<Panel, 4> out;
  for (int k = 0; k < 4; ++k) {
    const double yaw_k = state[YAW] + k * QUARTER_TURN;
    const bool even = k % 2 == 0;
    const double r_k = even ? state[R] : other_r;
    const double dz_k = even ? state[DZ] : -state[DZ];
    out[k] = {k, yaw_k, state.segment<3>(POS) + offset(yaw_k, r_k, dz_k)};
  }
  return out;
}

ArmorEKF::ArmorEKF(
  const Vector3d & panel_pos, const Vector3d & camera_pos, double t_sec_in,
  const Matrix3d & pos_var, const EkfParams & params)
: p(params), other_r(params.radius), t_sec(t_sec_in)
{
  if (p.q_jerk <= 0.0) {
    p.accel_prior_std = 0.0;
  }
  state.setZero();
  state[R] = p.radius;
  P.setIdentity();
  P(DZ, DZ) = p.dz_prior_std * p.dz_prior_std;
  reacquire(panel_pos, camera_pos, t_sec_in, pos_var, false);
}

void ArmorEKF::reacquire(
  const Vector3d & panel_pos, const Vector3d & camera_pos, double t, const Matrix3d & pos_var,
  bool keep_spin)
{
  const double r = state[R];
  double dz = state[DZ];
  const double yaw = std::atan2(camera_pos.y() - panel_pos.y(), camera_pos.x() - panel_pos.x());
  double w = state[W], w_var = P(W, W), r_var = P(R, R), dz_var = P(DZ, DZ);
  if (!keep_spin) {
    w = p.spin_prior;
    w_var = p.spin_prior_std * p.spin_prior_std;
    r_var = 0.05 * 0.05;
    dz = 0.0;
    dz_var = p.dz_prior_std * p.dz_prior_std;
  }
  state.setZero();
  state.segment<3>(POS) = panel_pos - offset(yaw, r, dz);
  state[YAW] = yaw;
  state[W] = w;
  state[R] = r;
  state[DZ] = dz;
  P.setZero();
  P.block<3, 3>(POS, POS) = pos_var + Vector3d(0.01, 0.01, 0.0).asDiagonal().toDenseMatrix();
  P.block<3, 3>(VEL, VEL) = Vector3d(4.0, 4.0, 0.25).asDiagonal();
  P.block<3, 3>(ACC, ACC) = p.accel_prior_std * p.accel_prior_std * Matrix3d::Identity();
  P(YAW, YAW) = 0.5 * 0.5;
  P(W, W) = w_var;
  P(R, R) = r_var;
  P(DZ, DZ) = dz_var;
  t_sec = t;
  n_outliers = 0;
  pin_still();
}

void ArmorEKF::pin_still()
{
  if (!p.still) {
    return;
  }
  state.segment<3>(VEL).setZero();
  state.segment<3>(ACC).setZero();
  state[W] = 0.0;
  zero_rows_cols(P, VEL, 3);
  zero_rows_cols(P, ACC, 3);
  zero_rows_cols(P, W, 1);
}

std::pair<Cov, Cov> ArmorEKF::transition(double dt) const
{
  const Matrix3d I3 = Matrix3d::Identity();
  Cov F = Cov::Identity();
  F.block<3, 3>(POS, VEL) = dt * I3;
  F(YAW, W) = dt;
  Cov Q = Cov::Zero();
  if (p.still) {
    Q.block<3, 3>(POS, POS) = dt * p.q_still_pos * p.q_still_pos * I3;
    Q(YAW, YAW) = dt * p.q_still_yaw * p.q_still_yaw;
    Q(R, R) = dt * p.q_radius * p.q_radius;
    Q(DZ, DZ) = dt * p.q_dz * p.q_dz;
    return {F, Q};
  }
  const double b00 = std::pow(dt, 4) / 4.0, b01 = std::pow(dt, 3) / 2.0, b11 = dt * dt;
  const double qa = p.q_accel * p.q_accel, qy = p.q_yaw_accel * p.q_yaw_accel;
  Q.block<3, 3>(POS, POS) = b00 * qa * I3;
  Q.block<3, 3>(POS, VEL) = b01 * qa * I3;
  Q.block<3, 3>(VEL, POS) = b01 * qa * I3;
  Q.block<3, 3>(VEL, VEL) = b11 * qa * I3;
  Q(YAW, YAW) = b00 * qy;
  Q(YAW, W) = Q(W, YAW) = b01 * qy;
  Q(W, W) = b11 * qy;
  Q(R, R) = dt * p.q_radius * p.q_radius;
  Q(DZ, DZ) = dt * p.q_dz * p.q_dz;
  if (p.q_jerk > 0.0) {
    // Singer: acc' = -acc / tau + jerk noise; exact F, white-jerk Q.
    const double alpha = 1.0 / p.accel_tau_s;
    const double decay = std::exp(-alpha * dt);
    F.block<3, 3>(POS, ACC) = (alpha * dt - 1.0 + decay) / (alpha * alpha) * I3;
    F.block<3, 3>(VEL, ACC) = (1.0 - decay) / alpha * I3;
    F.block<3, 3>(ACC, ACC) = decay * I3;
    Eigen::Matrix3d jerk;
    jerk << std::pow(dt, 5) / 20.0, std::pow(dt, 4) / 8.0, std::pow(dt, 3) / 6.0,
      std::pow(dt, 4) / 8.0, std::pow(dt, 3) / 3.0, dt * dt / 2.0,
      std::pow(dt, 3) / 6.0, dt * dt / 2.0, dt;
    jerk *= p.q_jerk * p.q_jerk;
    const int blocks[3] = {POS, VEL, ACC};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        Q.block<3, 3>(blocks[i], blocks[j]) += jerk(i, j) * I3;
      }
    }
  }
  return {F, Q};
}

std::pair<State, Cov> ArmorEKF::predicted(double t) const
{
  const double dt = t - t_sec;
  if (dt <= 0.0) {
    return {state, P};
  }
  const auto [F, Q] = transition(dt);
  return {F * state, F * P * F.transpose() + Q};
}

void ArmorEKF::predict(double t)
{
  std::tie(state, P) = predicted(t);
  t_sec = std::max(t_sec, t);
}

std::pair<int, double> ArmorEKF::associate(
  const Vector3d & panel_pos, const Vector3d & camera_pos)
{
  int best_k = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (const auto & panel : panel_positions(state, other_r)) {
    const double to_cam = std::atan2(
      camera_pos.y() - panel.pos.y(), camera_pos.x() - panel.pos.x());
    if (std::cos(panel.yaw - to_cam) <= BACK_FACING_COS) {
      continue;
    }
    const double d = (panel.pos - panel_pos).norm();
    if (best_k < 0 || d < best_d) {
      best_k = panel.k;
      best_d = d;
    }
  }
  if (best_k > 0) {
    state[YAW] += best_k * QUARTER_TURN;
    if (best_k % 2) {
      std::swap(state[R], other_r);
      state[DZ] = -state[DZ];
      P.row(DZ) *= -1.0;
      P.col(DZ) *= -1.0;  // P(DZ, DZ) flips twice, staying put
    }
  }
  return {best_k, best_d};
}

StepResult ArmorEKF::step(
  const Vector3d & panel_pos, const Vector3d & camera_pos, double t, const Matrix3d & pos_var,
  double gate_nis, int max_outliers, double facing_std)
{
  predict(t);
  associate(panel_pos, camera_pos);
  std::tie(last_nis, last_logdet) = innovation(panel_pos, pos_var);
  if (last_nis > gate_nis) {
    if (++n_outliers < max_outliers) {
      return StepResult::Outlier;
    }
    reacquire(panel_pos, camera_pos, t, pos_var);
    return StepResult::Reacquire;
  }
  n_outliers = 0;
  update(panel_pos, pos_var);
  if (facing_std > 0.0) {
    update_facing(camera_pos, facing_std);
  }
  return StepResult::Update;
}

void ArmorEKF::update_facing(const Vector3d & camera_pos, double std)
{
  // A panel seen alone faces the camera: pseudo-measure its yaw as the bearing.
  const double yaw = state[YAW];
  const Vector3d panel = state.segment<3>(POS) + offset(yaw, state[R], 0.0);
  const double bearing = std::atan2(camera_pos.y() - panel.y(), camera_pos.x() - panel.x());
  const double y = std::atan2(std::sin(bearing - yaw), std::cos(bearing - yaw));
  const double S = P(YAW, YAW) + std * std;
  const State K = P.col(YAW) / S;
  state += K * y;
  P -= K * P.row(YAW);
  P = 0.5 * (P + P.transpose());
}

namespace
{

// h(state) and its Jacobian.
std::pair<Vector3d, Eigen::Matrix<double, 3, N_STATE>> h_and_jacobian(const State & state)
{
  const double yaw = state[YAW], r = state[R];
  const double c = std::cos(yaw), s = std::sin(yaw);
  const Vector3d h = state.segment<3>(POS) + offset(yaw, r, state[DZ]);
  Eigen::Matrix<double, 3, N_STATE> H = Eigen::Matrix<double, 3, N_STATE>::Zero();
  H.block<3, 3>(0, POS).setIdentity();
  H(2, DZ) = 1.0;
  H(0, YAW) = -r * s;
  H(0, R) = c;
  H(1, YAW) = r * c;
  H(1, R) = s;
  return {h, H};
}

}  // namespace

std::pair<double, double> ArmorEKF::innovation(
  const Vector3d & panel_pos, const Matrix3d & pos_var) const
{
  const auto [h, H] = h_and_jacobian(state);
  const Vector3d y = panel_pos - h;
  const Matrix3d S = H * P * H.transpose() + pos_var;
  return {y.dot(S.partialPivLu().solve(y)), std::log(std::abs(S.determinant()))};
}

double ArmorEKF::nis(const Vector3d & panel_pos, const Matrix3d & pos_var) const
{
  return innovation(panel_pos, pos_var).first;
}

void ArmorEKF::update(const Vector3d & panel_pos, const Matrix3d & pos_var)
{
  const auto [h, H] = h_and_jacobian(state);
  const Matrix3d S = H * P * H.transpose() + pos_var;
  const Eigen::Matrix<double, N_STATE, 3> K = P * H.transpose() * S.inverse();
  state += K * (panel_pos - h);
  const Cov IKH = Cov::Identity() - K * H;
  P = IKH * P * IKH.transpose() + K * pos_var * K.transpose();  // Joseph form
  state[R] = std::clamp(state[R], p.r_min, p.r_max);
  state[DZ] = std::clamp(state[DZ], -DZ_MAX, DZ_MAX);
}

ArmorTracker::ArmorTracker(
  const Vector3d & panel_pos, const Vector3d & camera_pos, double t_sec,
  const Matrix3d & pos_var, const TrackerParams & params)
: p(params)
{
  for (double w : p.spin_priors) {
    EkfParams e = p.ekf;
    e.spin_prior = w;
    e.spin_prior_std = p.prior_std;
    e.still = false;
    filters.emplace_back(panel_pos, camera_pos, t_sec, pos_var, e);
  }
  if (p.still) {
    EkfParams e = p.ekf;
    e.spin_prior = 0.0;
    e.spin_prior_std = 0.0;
    e.still = true;
    filters.emplace_back(panel_pos, camera_pos, t_sec, pos_var, e);
  }
  worse_since.assign(filters.size(), std::nullopt);
}

StepResult ArmorTracker::step(
  const Vector3d & panel_pos, const Vector3d & camera_pos, double t_sec,
  const Matrix3d & pos_var, double gate_nis, int max_outliers, double facing_std)
{
  const size_t n = filters.size();
  std::vector<StepResult> statuses(n);
  std::vector<double> nlls(n);
  const bool first = scores.empty();
  if (first) {
    scores.assign(n, 0.0);
  }
  for (size_t i = 0; i < n; ++i) {
    auto & f = filters[i];
    statuses[i] = f.step(panel_pos, camera_pos, t_sec, pos_var, gate_nis, max_outliers,
        facing_std);
    nlls[i] = std::min(f.last_nis, gate_nis) + f.last_logdet;  // clamped: one wild sample
    scores[i] = first ? nlls[i] : scores[i] + p.alpha * (nlls[i] - scores[i]);
  }
  std::vector<size_t> moving, all;
  for (size_t i = 0; i < n; ++i) {
    all.push_back(i);
    if (!filters[i].p.still) {
      moving.push_back(i);
    }
  }
  if (best().p.still) {
    // A parked target that moves: CUSUM of the per-sample log-likelihood
    // ratio against the best moving filter hands over within a few samples.
    const size_t rival = first_min(moving, [&](size_t i) {return nlls[i];});
    still_cusum_ = std::max(0.0, still_cusum_ + nlls[lead] - nlls[rival]);
    if (still_cusum_ > p.still_exit_llr) {
      lead = first_min(moving, [&](size_t i) {return scores[i];});
      challenger_.reset();
      still_cusum_ = 0.0;
    }
  }
  // The lead changes only after a challenger beats it by switch_margin for
  // switch_after_s: a noise burst briefly favours a collapsed-radius
  // wrong-sign hypothesis, which is least sensitive to it.
  const size_t best_i = first_min(all, [&](size_t i) {return scores[i];});
  const double lead_score = scores[lead];
  const double margin = filters[best_i].p.still ? p.still_margin : p.switch_margin;
  if (best_i == lead || scores[best_i] > lead_score - margin) {
    challenger_.reset();
  } else if (!challenger_ || challenger_->first != best_i) {
    challenger_ = std::make_pair(best_i, t_sec);
  } else if (t_sec - challenger_->second >= p.switch_after_s) {
    lead = best_i;
    challenger_.reset();
  }
  const ArmorEKF & leader = filters[lead];
  const double best_score = scores[lead];
  for (size_t i = 0; i < n; ++i) {
    if (i == lead || (challenger_ && challenger_->first == i) ||
      scores[i] < best_score + p.reseed_margin)
    {
      worse_since[i].reset();
      continue;
    }
    if (!worse_since[i]) {
      worse_since[i] = t_sec;
    } else if (t_sec - *worse_since[i] >= p.reseed_after_s) {
      auto & f = filters[i];
      f.state = leader.state;
      f.state[W] = f.p.spin_prior;
      f.state[R] = f.other_r = f.p.radius;
      f.P = leader.P;
      zero_rows_cols(f.P, W, 1);
      f.P(W, W) = f.p.spin_prior_std * f.p.spin_prior_std;
      zero_rows_cols(f.P, R, 1);
      f.P(R, R) = 0.05 * 0.05;
      f.pin_still();
      f.t_sec = leader.t_sec;
      scores[i] = best_score + p.reseed_margin / 2.0;
      worse_since[i].reset();
    }
  }
  return statuses[lead];
}

}  // namespace thornbots_pkg
