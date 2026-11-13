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

#ifndef THORNBOTS_PKG__TARGET_CONFIDENCE_HPP_
#define THORNBOTS_PKG__TARGET_CONFIDENCE_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

namespace thornbots_pkg
{
inline double decay_confidence(double q, double dt, double tau)
{
  if (!std::isfinite(tau) || tau <= 0.0) {
    throw std::invalid_argument("confidence_decay_tau_s must be finite and positive");
  }
  if (!std::isfinite(q) || !std::isfinite(dt)) {return 0.0;}
  return std::clamp(q, 0.0, 1.0) * std::exp(-std::max(0.0, dt) / tau);
}

// One maximum over accepted selected-robot panels per capture; never accumulate frames.
class TargetConfidence
{
public:
  void reset() {anchor_.reset();}
  void accept(double q, double capture_s)
  {
    if (!std::isfinite(q) || !std::isfinite(capture_s)) {return;}
    if (anchor_ && capture_s < anchor_->second) {reset();}
    if (!anchor_ || capture_s > anchor_->second) {
      anchor_ = std::make_pair(std::clamp(q, 0.0, 1.0), capture_s);
    } else {
      anchor_->first = std::max(anchor_->first, std::clamp(q, 0.0, 1.0));
    }
  }
  double at(double state_s, double tau) const
  {return anchor_ ? decay_confidence(anchor_->first, state_s - anchor_->second, tau) : 0.0;}
  std::optional<double> capture() const
  {return anchor_ ? std::optional<double>(anchor_->second) : std::nullopt;}

private:
  std::optional<std::pair<double, double>> anchor_;
};
}  // namespace thornbots_pkg
#endif  // THORNBOTS_PKG__TARGET_CONFIDENCE_HPP_
