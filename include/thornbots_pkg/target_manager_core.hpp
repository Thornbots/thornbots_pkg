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

#ifndef THORNBOTS_PKG__TARGET_MANAGER_CORE_HPP_
#define THORNBOTS_PKG__TARGET_MANAGER_CORE_HPP_

#include <array>
#include <cstdint>
#include <optional>

#include "Eigen/Dense"

namespace thornbots_pkg
{
struct ManagedModel
{
  double state_s = 0.0, detection_s = 0.0, confidence = 0.0;
  uint32_t track_id = 0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d panel = Eigen::Vector3d::Zero();
  double yaw = 0.0, yaw_rate = 0.0, yaw_rate_variance = 0.0;
  std::array<double, 2> radius{}, z_offset{};
  std::array<double, 6> variance{};
  bool selected = true, reset = false, usable = true;
};

struct ManagerParams
{
  double tau_s, assign_confidence, clear_confidence;
  double drift_m, horizon_s, refresh_s;
};

enum class TargetEvent {None, Assign, Update, Refresh, Clear};
bool sane_model(const ManagedModel & model);
double model_drift(const ManagedModel & held, const ManagedModel & current,
  double now_s, double horizon_s);

// The emitted baseline changes only on a send. Prediction for comparison uses
// the MCB's constant acceleration and rigid spin, never the EKF's Singer model.
class TargetManager
{
public:
  explicit TargetManager(const ManagerParams & params);
  TargetEvent consume(const ManagedModel & model, double now_s);
  TargetEvent tick(double now_s);
  bool assigned() const {return held_.has_value();}
  uint32_t released_track() const {return released_track_;}
  bool clear_before_assign() const {return clear_before_assign_;}

private:
  TargetEvent evaluate(double now_s);
  TargetEvent clear();
  ManagerParams params_;
  std::optional<ManagedModel> candidate_, held_;
  std::optional<double> last_now_;
  double last_sent_s_ = 0.0;
  uint32_t released_track_ = 0;
  bool clear_before_assign_ = false;
};
}  // namespace thornbots_pkg
#endif  // THORNBOTS_PKG__TARGET_MANAGER_CORE_HPP_
