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

// Relocalize decision for mcb_relay (no rclcpp).
//
// The correction is the offset loc(t) - odom(t) at the localization stamp,
// /odom interpolated there, added to where the MCB's odometry will read when
// it applies the frame: latest /odom extrapolated past both UART legs and the
// MCB's read delay. Sent only when confident: localization's xy std and the
// extrapolation's (speed x latency_std_s) combined stay under max_std_m, and
// the offset clears error_threshold_m and n_sigma of that std. See
// README.md's ### mcb_relay.

#ifndef THORNBOTS_PKG__MCB_RELAY_CORE_HPP_
#define THORNBOTS_PKG__MCB_RELAY_CORE_HPP_

#include <deque>
#include <optional>
#include <utility>

namespace thornbots_pkg
{

struct OdomSample
{
  double t, x, y, vx, vy;
};

struct RelocalizeCommand
{
  double x, y;
  double apply_t;  // when the MCB should adopt (x, y), on the /odom stamps' clock
  double error_m, std_m;
};

// Buffers /odom and decides when and what to relocalize the MCB to.
class Relocalizer
{
public:
  struct Params
  {
    double error_threshold_m = 0.05;
    double n_sigma = 3.0;
    double max_std_m = 0.02;
    double uart_latency_s = 0.005;
    double mcb_read_delay_s = 0.002;
    double latency_std_s = 0.003;
    double hold_off_s = 0.3;
    double history_s = 1.0;
  };

  Relocalizer() = default;
  explicit Relocalizer(const Params & params)
  : p_(params) {}

  // Record one /odom sample; a stamp going backwards clears the history.
  void add_odom(double t, double x, double y, double vx, double vy);

  // (x, y) of /odom at t: interpolated, extrapolated up to max_ahead_s, else nullopt.
  std::optional<std::pair<double, double>> odom_at(double t, double max_ahead_s = 0.1) const;

  // loc_var_xy is the larger of localization's x and y variances (m^2); now is
  // the send time on the /odom stamps' clock. Returns what to send, or nullopt.
  std::optional<RelocalizeCommand> decide(
    double loc_t, double loc_x, double loc_y, double loc_var_xy, double now);

  const Params & params() const {return p_;}
  size_t odom_size() const {return odom_.size();}

private:
  Params p_;
  std::deque<OdomSample> odom_;
  std::optional<double> last_sent_t_;
};

}  // namespace thornbots_pkg

#endif  // THORNBOTS_PKG__MCB_RELAY_CORE_HPP_
