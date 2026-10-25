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

#ifndef THORNBOTS_PKG__ROOM_CHECK_CORE_HPP_
#define THORNBOTS_PKG__ROOM_CHECK_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace thornbots_pkg::room_check
{
struct Result
{
  std::string name, status, detail;
};
inline Result verdict(const std::string & name, bool pass, const std::string & detail)
{
  return {name, pass ? "PASS" : "FAIL", detail};
}
inline std::string number(double value, int precision = 3)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}
struct Stream
{
  size_t count = 0, future = 0, stale = 0, invalid = 0, backwards = 0;
  double first = 0, last = 0, last_stamp = 0, max_gap = 0, max_wall_gap = 0;
  double last_wall = 0, max_age = 0, max_source_gap = 0;
  std::vector<double> gaps;
  void add(double receipt, double wall, double stamp, double stale_limit, double future_limit)
  {
    if (count) {
      gaps.push_back(receipt - last);
      max_gap = std::max(max_gap, receipt - last);
      max_source_gap = std::max(max_source_gap, stamp - last_stamp);
      max_wall_gap = std::max(max_wall_gap, wall - last_wall);
      if (stamp < last_stamp) {++backwards;}
    } else {
      first = receipt;
    }
    ++count;
    last = receipt;
    last_wall = wall;
    last_stamp = stamp;
    const double age = receipt - stamp;
    if (!std::isfinite(stamp) || stamp <= 0) {++invalid;}
    if (age < -future_limit) {++future;}
    if (age > stale_limit) {++stale;}
    max_age = std::max(max_age, age);
  }
  double gap(double start, double end) const
  {
    return count ? std::max({max_gap, first - start, end - last}) : end - start;
  }
  double rate(double start, double end) const
  {
    return end > start ? static_cast<double>(count) / (end - start) : 0;
  }
  Result availability(const std::string & name, bool required) const
  {
    if (!count) {return {name, required ? "FAIL" : "SKIP", "missing (0 messages)"};}
    return {name, "PASS", std::to_string(count) + " messages"};
  }
  Result cadence(const std::string & name, double start, double end,
    double expected, double tolerance, double gap_limit, bool required = true) const
  {
    if (!count) {return availability(name, required);}
    const double hz = rate(start, end), silence = gap(start, end);
    const auto large_gaps = std::count_if(gaps.begin(), gaps.end(),
      [gap_limit](double g) {return g > gap_limit;}) +
      (first - start > gap_limit ? 1 : 0) + (end - last > gap_limit ? 1 : 0);
    return verdict(name, count >= 2 && std::abs(hz / expected - 1) <= tolerance &&
      silence <= gap_limit, number(hz) + " Hz; max gap " + number(silence) +
      " s; gaps=" + std::to_string(large_gaps) +
      "; source max=" + number(max_source_gap) + " s; wall max=" + number(max_wall_gap) + " s");
  }
  bool stamps_ok() const {return !(future || stale || invalid || backwards);}
};
struct Geometry
{
  size_t count = 0, bad = 0, changed = 0, first_bins = 0, min_bins = 0, max_bins = 0;
  double first_increment = 0, min_increment = 0, max_increment = 0;
  double first_min = 0, first_max = 0;
  void add(size_t bins, double increment, double angle_min, double angle_max, size_t expected)
  {
    if (!count) {
      first_bins = min_bins = max_bins = bins;
      first_increment = min_increment = max_increment = increment;
      first_min = angle_min; first_max = angle_max;
    }
    ++count;
    min_bins = std::min(min_bins, bins);
    max_bins = std::max(max_bins, bins);
    min_increment = std::min(min_increment, increment);
    max_increment = std::max(max_increment, increment);
    if (bins != first_bins || increment != first_increment ||
      angle_min != first_min || angle_max != first_max) {++changed;}
    if (bins != expected || !std::isfinite(increment) || increment <= 0 ||
      !std::isfinite(angle_min) || !std::isfinite(angle_max) || bins < 2 ||
      std::abs(angle_max - angle_min - increment * static_cast<double>(bins - 1)) >
      1.5 * increment || std::abs(angle_max - angle_min - 2 * M_PI) >
      2 * increment) {++bad;}
  }
  Result score() const
  {
    if (!count) {return {"scan geometry", "FAIL", "missing scans"};}
    return verdict("scan geometry", !bad && !changed,
      "bins " + std::to_string(min_bins) + ".." + std::to_string(max_bins) +
      "; increment " + number(min_increment, 9) + ".." + number(max_increment, 9) +
      "; bad=" + std::to_string(bad) + " changed=" + std::to_string(changed));
  }
};
struct Drift
{
  size_t count = 0;
  double first_x = 0, first_y = 0, first_yaw = 0, first_time = 0, last_time = 0;
  double max_xy = 0, max_yaw = 0, path = 0, last_x = 0, last_y = 0;
  bool invalid = false;
  void add(double time, double x, double y, double yaw)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      invalid = true;
      return;
    }
    if (!count) {
      first_x = x; first_y = y; first_yaw = yaw; first_time = time;
    } else {
      path += std::hypot(x - last_x, y - last_y);
    }
    ++count;
    max_xy = std::max(max_xy, std::hypot(x - first_x, y - first_y));
    max_yaw = std::max(max_yaw, std::abs(std::remainder(yaw - first_yaw, 2 * M_PI)));
    last_x = x; last_y = y; last_time = time;
  }
  Result score(const std::string & name, double duration, double xy, double yaw,
    bool required) const
  {
    if (!count) {return {name, required ? "FAIL" : "SKIP", "missing pose"};}
    return verdict(name, !invalid && count >= 2 && last_time - first_time >= duration * 0.9 &&
      max_xy <= xy && max_yaw <= yaw, "excursion " + number(max_xy) + " m / " +
      number(max_yaw) + " rad; path " + number(path) + " m; span " +
      number(last_time - first_time) + " s");
  }
};
struct StaleCV
{
  size_t detections = 0, active_before = 0, decisions_after = 0, violations = 0;
  double last_detection = -1, max_delay = 0;
  void detection(double wall) {++detections; last_detection = wall;}
  void decision(double wall, bool active)
  {
    if (last_detection < 0) {return;}
    const double delay = wall - last_detection;
    if (delay <= 0.2 && active) {++active_before;}
    if (delay > 0.2) {
      ++decisions_after;
      if (active) {++violations; max_delay = std::max(max_delay, delay);}
    }
  }
  Result score(double end) const
  {
    const bool exercised = detections >= 2 && active_before && end - last_detection >= 1.0 &&
      decisions_after >= 2;
    return verdict("stale CV <=200ms", exercised && !violations,
      "post-cutoff active=" + std::to_string(violations) + "; last active delay=" +
      number(max_delay) + " s" + (exercised ? "" : "; NOT EXERCISED (need target, then stop YOLO)"));
  }
};
inline Result latency_score(std::vector<double> samples, size_t states,
  double future_limit, double latency_limit)
{
  std::sort(samples.begin(), samples.end());
  const size_t n = samples.size();
  const double p95 = n ? samples[static_cast<size_t>(0.95 * (n - 1))] : 0;
  return verdict("capture->state proxy", n >= 10 && n >= 0.8 * states &&
    samples.front() >= -future_limit && p95 <= latency_limit,
    "p95 " + number(p95) + " s; paired " + std::to_string(n) + "/" + std::to_string(states) +
    " (nearest measurement receipt <=3ms; not exact association)");
}
inline bool drop_report(const std::string & text)
{
  std::string lower = text;
  std::transform(lower.begin(), lower.end(), lower.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return lower.find("drop") != std::string::npos || lower.find("skip") != std::string::npos ||
         lower.find("mismatch") != std::string::npos || lower.find("not publishing") != std::string::npos;
}
}  // namespace thornbots_pkg::room_check
#endif  // THORNBOTS_PKG__ROOM_CHECK_CORE_HPP_
