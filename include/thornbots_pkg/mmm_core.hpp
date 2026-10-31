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

#ifndef THORNBOTS_PKG__MMM_CORE_HPP_
#define THORNBOTS_PKG__MMM_CORE_HPP_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace thornbots_pkg::mmm
{
struct Pose
{
  double x, y, yaw;
};

enum class SpawnCheck {Boot, Save};
bool at_team_spawn(double x, double y, bool blue, SpawnCheck check);

class StartGate
{
public:
  explicit StartGate(Pose boot);
  bool update(std::optional<Pose> pose, double stamp, double now);
  bool at_start() const {return at_start_;}
  void reset(Pose boot);

private:
  Pose boot_;
  bool at_start_ = false;
  std::deque<std::pair<double, Pose>> samples_;
};

class ScanGate
{
public:
  bool update(const std::array<double, 4> & geometry, double stamp, double now);
  bool ready(double now) const;
  bool geometry_failed() const {return geometry_failed_;}
  const std::optional<std::array<double, 4>> & geometry() const {return geometry_;}
private:
  std::optional<std::array<double, 4>> geometry_;
  double stamp_ = -1;
  size_t fresh_scans_ = 0;
  bool geometry_failed_ = false;
};

class Protocol
{
public:
  Protocol(std::function<void(int)> send, std::function<void(int)> dispatch,
    std::function<void(std::string)> warn);
  // Only 2/4/6 are transactions; tick owns the independent zero heartbeat.
  void reset_session();
  bool request(int number, double now);
  void receive(int number, double now);
  void tick(double now);
  std::optional<int> pending() const {return pending_;}
  std::optional<int> completed_even() const {return completed_even_;}
  std::optional<int> failed() const {return failed_;}

private:
  std::function<void(int)> send_, dispatch_;
  std::function<void(std::string)> warn_;
  std::optional<int> pending_, completed_even_, failed_;
  std::array<std::optional<double>, 7> odd_seen_{}, even_used_{};
  double last_ping_ = -std::numeric_limits<double>::infinity();
  double last_attempt_ = 0;
  int attempts_ = 0;
};

enum class SaveDecision {Ignore, Incomplete, Ready};
SaveDecision decide_save(bool session_ready, bool busy, bool at_start,
  std::optional<int> completed_even);

class SaveGuard
{
public:
  void begin(bool ready_save) {ready_ = ready_save; valid_ = true;}
  void observe(bool parked_inputs) {valid_ = valid_ && parked_inputs;}
  bool accept(bool saved) const {return saved && (!ready_ || valid_);}
private:
  bool ready_ = false, valid_ = true;
};

class Workflow
{
public:
  explicit Workflow(Protocol & protocol);
  void start_session();
  void setup_result(bool mapping_mode, bool success);
  void save_result(bool ready_save, bool success);
  // Call after Protocol::tick; pose evidence selects 2/4 only after mapping setup.
  void tick(bool at_start, double now);

private:
  Protocol & protocol_;
  std::optional<int> announced_;
  bool configured_ = false;
  bool mapping_ready_ = false;
  bool recovering_ready_ = false;
};
}  // namespace thornbots_pkg::mmm
#endif  // THORNBOTS_PKG__MMM_CORE_HPP_
