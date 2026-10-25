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

#include "thornbots_pkg/room_check_core.hpp"

#include <chrono>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <stdexcept>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "dji_serial_bridge/msg/robot_pose.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rc = thornbots_pkg::room_check;
using Steady = std::chrono::steady_clock;
static double wall_now()
{
  return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
}
static double stamp(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + t.nanosec * 1e-9;
}
static std::string csv(const std::string & value)
{
  std::string out = "\"";
  for (const char c : value) {out += c == '"' ? "\"\"" : std::string(1, c);}
  return out + "\"";
}
static double yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

class RoomCheck : public rclcpp::Node
{
public:
  RoomCheck() : Node("room_check")
  {
    duration_ = positive("duration_s", 30.0);
    warmup_ = positive("warmup_s", 3.0);
    stale_limit_ = positive("max_stamp_age_s", 0.5);
    future_limit_ = positive("max_future_s", 0.05);
    raw_xy_ = positive("raw_drift_m", 0.03);
    loc_xy_ = positive("localization_drift_m", 0.15);
    yaw_limit_ = positive("drift_rad", 0.03);
    pose_hz_ = positive("pose_hz", 90.0);
    camera_hz_ = positive("camera_hz", 60.0);
    yolo_hz_ = positive("yolo_hz", 60.0);
    latency_limit_ = positive("latency_s", 0.2);
    bins_ = declare_parameter("scan_bins", 720);
    if (bins_ < 2) {throw std::runtime_error("scan_bins must be >=2");}
    camera_ = declare_parameter("camera", false);
    stale_cv_ = declare_parameter("stale_cv", false);
    require_ref_ = declare_parameter("require_referee", false);
    require_rf2o_ = declare_parameter("require_rf2o", false);
    require_loc_ = declare_parameter("require_localization", false);
    const std::string parent = declare_parameter("output_dir", std::string("room-evidence"));
    auto time = std::time(nullptr);
    std::ostringstream name;
    name << std::put_time(std::gmtime(&time), "%Y%m%dT%H%M%SZ") << "-" <<
      std::chrono::duration_cast<std::chrono::microseconds>(Steady::now().time_since_epoch()).count();
    directory_ = std::filesystem::path(parent) / name.str();
    if (!std::filesystem::create_directories(directory_)) {
      throw std::runtime_error("cannot create evidence directory");
    }
    events_.open(directory_ / "observations.csv");
    logs_.open(directory_ / "rosout.csv");
    if (!events_ || !logs_) {throw std::runtime_error("cannot open evidence files");}
    events_ << "event,steady_s,ros_s,stamp_s,age_s,value1,value2,value3\n";
    logs_ << "steady_s,ros_s,node,level,message\n";
    events_ << std::setprecision(17);
    logs_ << std::setprecision(17);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    subscribe<dji_serial_bridge::msg::RobotPose>("/dji_serial_bridge/pose", "POSE",
      [this](const auto & m) {event("POSE position/status", stamp(m.header.stamp), m.x, m.y, m.odom_status);});
    subscribe<dji_serial_bridge::msg::RefSysStatus>("/dji_serial_bridge/ref_sys", "REF_SYS",
      [this](const auto & m) {
        if (m.robot_id) {++ref_assigned_;}
        event("REF_SYS assignment", stamp(m.header.stamp), m.robot_id, m.is_on_blue_team);
      });
    subscribe<sensor_msgs::msg::LaserScan>("/scan", "scan", [this](const auto & m) {
        geometry_.add(m.ranges.size(), m.angle_increment, m.angle_min, m.angle_max, bins_);
        event("scan_geometry", stamp(m.header.stamp), m.ranges.size(), m.angle_increment, m.angle_min);
      });
    odometry("/odom", "raw odom");
    odometry("/scan_odom", "rf2o");
    odometry("/localization/odom", "navigation");
    subscribe<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", "AMCL",
      [this](const auto & m) {pose("AMCL", m.pose.pose);});
    auto tf = create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SensorDataQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
        if (!recording_) {return;}
        for (const auto & t : m->transforms) {
          const auto & child = t.child_frame_id;
          if (child == "head") {observe("head TF", t.header);}
          if (t.header.frame_id == "map" && child == "nav_odom") {
            observe("map TF", t.header, 1.1);
          }
          if (t.header.frame_id == "nav_odom" && child == "odom") {
            observe("nav TF", t.header);
          }
        }
      });
    subscriptions_.push_back(tf);
    subscribe<diagnostic_msgs::msg::DiagnosticArray>("/scan_odom/quality", "rf2o quality",
      [this](const auto & m) {
        for (const auto & status : m.status) {
          if (status.level == 2) {++rf2o_failed_;}
          for (const auto & kv : status.values) {
            if (kv.key == "increment" && kv.value == "none") {++rf2o_no_increment_;}
          }
        }
      });
    auto log = create_subscription<rcl_interfaces::msg::Log>("/rosout", rclcpp::QoS(1000).best_effort(),
      [this](rcl_interfaces::msg::Log::ConstSharedPtr m) {
        if (!recording_) {return;}
        logs_ << wall_now() << ',' << now().seconds() << ',' << csv(m->name) << ',' <<
          static_cast<int>(m->level) << ',' << csv(m->msg) << '\n';
        if (!rc::drop_report(m->msg)) {return;}
        if (m->name.find("rf2o") != std::string::npos) {++rf2o_reports_;}
        if (m->name.find("slam") != std::string::npos) {++slam_reports_;}
        if (m->name.find("amcl") != std::string::npos) {++amcl_reports_;}
      });
    subscriptions_.push_back(log);
    if (camera_ || stale_cv_) {
      const auto yolo_topic = declare_parameter("yolo_topic", std::string("/detections_output"));
      subscribe<vision_msgs::msg::Detection2DArray>(yolo_topic, "YOLO",
        [this](const auto &) {stale_.detection(wall_now());});
      subscribe<dji_serial_bridge::msg::CVTarget>("/cv/target", "CV decisions",
        [this](const auto & m) {stale_.decision(wall_now(), !m.patrol || m.fire);});
    }
    if (camera_) {
      subscribe<sensor_msgs::msg::Image>(
        declare_parameter("camera_topic", std::string("/color/image_raw")), "camera");
      subscribe<std_msgs::msg::Header>("/cv/tracker/measurement", "measurement",
        [this](const auto & m) {measurements_.push_back({wall_now(), stamp(m.stamp)});});
      subscribe<dji_serial_bridge::msg::TargetState>("/cv/target_state", "TargetState",
        [this](const auto &) {
          pending_states_.push_back({wall_now(), now().seconds()});
        });
    }
    std::ofstream config(directory_ / "config.txt");
    for (const auto & p : list_parameters({}, 10).names) {
      config << p << "=" << get_parameter(p).value_to_string() << "\n";
    }
    for (const auto * key : {"ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE"}) {
      const char * value = std::getenv(key);
      config << key << "=" << (value ? value : "(unset)") << "\n";
    }
    config.flush();
    if (!config) {throw std::runtime_error("configuration write failed");}
    std::cout << "Park chassis and head for " << duration_ << " wall seconds after " << warmup_ <<
      " s discovery. Evidence: " << directory_.string() << '\n';
    if (stale_cv_) {
      std::cout << "STALE CV: show a tracked target, then stop YOLO in its terminal; leave consumers running.\n";
    }
  }
  int run()
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(shared_from_this());
    const double begin = wall_now();
    const bool simulated = get_parameter("use_sim_time").as_bool();
    while (rclcpp::ok() && (wall_now() - begin < warmup_ ||
      (simulated && now().nanoseconds() == 0 && wall_now() - begin < warmup_ + 10))) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    graph("graph-start.txt");
    start_ = now().seconds(); wall_start_ = wall_now(); recording_ = true;
    double prev_ros = start_, prev_wall = wall_start_;
    while (rclcpp::ok() && wall_now() - wall_start_ < duration_) {
      executor.spin_some();
      const double w = wall_now(), t = now().seconds();
      if (t < prev_ros || (std::abs((t - prev_ros) - (w - prev_wall)) > 0.1 &&
        !get_parameter("use_sim_time").as_bool())) {++clock_jumps_;}
      if (t != prev_ros) {last_clock_progress_ = w;}
      prev_ros = t; prev_wall = w;
      pair_latency(w);
      map_pose(w);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    end_ = now().seconds(); recording_ = false;
    if (rclcpp::ok()) {graph("graph-end.txt");}
    std::vector<rc::Result> results;
    results.push_back(rc::verdict("clock", end_ > start_ && start_ > 0 && !clock_jumps_ &&
      wall_now() - last_clock_progress_ < 0.5 && rclcpp::ok(),
      "ROS span " + rc::number(end_ - start_) + " s; wall " + rc::number(wall_now() - wall_start_) +
      " s; jumps=" + std::to_string(clock_jumps_)));
    for (const auto & key : {"POSE", "head TF"}) {
      results.push_back(streams_[key].cadence(key, start_, end_, pose_hz_, 0.2, 0.1));
    }
    results.push_back(streams_["REF_SYS"].cadence("REF_SYS", start_, end_, 10, 0.3, 0.35, require_ref_));
    if (streams_["REF_SYS"].count && !ref_assigned_) {
      results.push_back({"referee assignment", require_ref_ ? "FAIL" : "SKIP", "robot_id=0 (no referee)"});
    }
    results.push_back(geometry_.score());
    results.push_back(streams_["scan"].cadence("scan delivery", start_, end_, 10, 0.5, 0.5));
    const auto scan_count = streams_["scan"].count, odom_count = streams_["rf2o"].count;
    const bool rf2o_present = require_rf2o_ || publishers_["/scan_odom"];
    auto rf = streams_["rf2o"].availability("rf2o accepts scans", rf2o_present);
    if (odom_count) {
      rf = rc::verdict("rf2o accepts scans", scan_count > 0 &&
        odom_count >= 0.8 * static_cast<double>(scan_count) &&
        streams_["rf2o"].gap(start_, end_) <= 0.5 && !rf2o_no_increment_ && !rf2o_reports_,
        std::to_string(odom_count) + "/" + std::to_string(scan_count) +
        " outputs/scans; failed-quality=" + std::to_string(rf2o_failed_) +
        " no-increment=" + std::to_string(rf2o_no_increment_) +
        " skip/drop reports=" + std::to_string(rf2o_reports_));
    }
    results.push_back(rf);
    if (slam_running_ || amcl_running_) {
      results.push_back(rc::verdict("SLAM/AMCL drop reports", !slam_reports_ && !amcl_reports_,
        "SLAM=" + std::to_string(slam_reports_) + " AMCL=" + std::to_string(amcl_reports_) +
        " (log reports; throttled, lower bound)"));
    } else {
      results.push_back({"SLAM/AMCL drop reports", "SKIP", "neither backend discovered"});
    }
    size_t stamp_errors = 0, stamped = 0;
    std::ofstream metrics(directory_ / "metrics.csv");
    metrics << "stream,count,hz,receipt_max_gap_s,source_max_gap_s,wall_max_gap_s,future,stale,zero,backwards,max_age_s\n";
    metrics << std::setprecision(17);
    for (const auto & item : streams_) {
      const auto & m = item.second;
      metrics << csv(item.first) << ',' << m.count << ',' << m.rate(start_, end_) << ',' <<
        m.gap(start_, end_) << ',' << m.max_source_gap << ',' << m.max_wall_gap << ',' <<
        m.future << ',' << m.stale << ',' << m.invalid << ',' << m.backwards << ',' << m.max_age << '\n';
      const auto & s = item.second;
      stamped += s.count;
      stamp_errors += s.future + s.stale + s.invalid + s.backwards;
      events_ << csv("stamp totals " + item.first) << ",0,0,0,0," << s.future << ',' <<
        s.stale << ',' << s.invalid + s.backwards << '\n';
    }
    results.push_back(rc::verdict("stamps", stamped && !stamp_errors,
      stamped ? std::to_string(stamp_errors) + " future/stale/zero/backwards events (metrics.csv)" :
      "missing stamped messages"));
    results.push_back(drifts_["raw odom"].score("raw odom parked", end_ - start_, raw_xy_, yaw_limit_, true));
    results.push_back(drifts_["navigation"].score("navigation parked", end_ - start_, loc_xy_, yaw_limit_,
      require_loc_ || publishers_["/localization/odom"]));
    results.push_back(drifts_["map root"].score("map localization parked", end_ - start_, loc_xy_, yaw_limit_,
      require_loc_ || streams_["map TF"].count));
    if (camera_) {camera_results(results);}
    if (stale_cv_) {results.push_back(stale_.score(wall_now()));}
    std::ofstream summary(directory_ / "summary.txt");
    bool fail = false;
    for (const auto & r : results) {
      const auto line = r.status + " " + r.name + ": " + r.detail + "\n";
      std::cout << line; summary << line;
      fail = fail || r.status == "FAIL";
    }
    const std::string final = std::string(fail ? "FAIL" : "PASS") +
      " observed checks (SKIP is unverified); evidence: " + directory_.string() + "\n";
    std::cout << final; summary << final;
    events_.flush(); logs_.flush(); summary.flush(); metrics.flush();
    if (!events_ || !logs_ || !summary || !metrics) {throw std::runtime_error("evidence write failed");}
    return fail ? 1 : 0;
  }

private:
  double positive(const std::string & key, double value)
  {
    const double result = declare_parameter(key, value);
    if (!std::isfinite(result) || result <= 0) {throw std::runtime_error(key + " must be positive");}
    return result;
  }
  template<class Message, class Callback>
  void subscribe(const std::string & topic, const std::string & key, Callback callback)
  {
    subscriptions_.push_back(create_subscription<Message>(topic, rclcpp::SensorDataQoS().keep_last(
      std::is_same_v<Message, sensor_msgs::msg::Image> ? 5 : 1000),
      [this, key, callback](const typename Message::ConstSharedPtr m) {
        if (!recording_) {return;}
        if constexpr (std::is_same_v<Message, std_msgs::msg::Header>) {observe(key, *m);} else {
          observe(key, m->header);
        }
        callback(*m);
      }));
  }
  template<class Message>
  void subscribe(const std::string & topic, const std::string & key)
  {
    subscribe<Message>(topic, key, [](const auto &) {});
  }
  void observe(const std::string & key, const std_msgs::msg::Header & header, double future = -1)
  {
    const double t = stamp(header.stamp);
    streams_[key].add(now().seconds(), wall_now(), t, stale_limit_, future < 0 ? future_limit_ : future);
    event(key, t);
  }
  void event(const std::string & key, double t, double a = 0, double b = 0, double c = 0)
  {
    events_ << csv(key) << ',' << wall_now() << ',' << now().seconds() << ',' << t << ',' <<
      now().seconds() - t << ',' << a << ',' << b << ',' << c << '\n';
  }
  void odometry(const std::string & topic, const std::string & key)
  {
    subscribe<nav_msgs::msg::Odometry>(topic, key, [this, key](const auto & m) {
        pose(key, m.pose.pose);
      });
  }
  void pose(const std::string & key, const geometry_msgs::msg::Pose & p)
  {
    const auto & q = p.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm) || std::abs(norm - 1) > 0.01) {drifts_[key].invalid = true;}
    drifts_[key].add(now().seconds(), p.position.x, p.position.y, yaw(p.orientation));
    event(key + " position", now().seconds(), p.position.x, p.position.y, yaw(p.orientation));
  }
  void map_pose(double wall)
  {
    if (wall - last_map_poll_ < 0.05) {return;}
    last_map_poll_ = wall;
    try {
      const auto t = tf_buffer_->lookupTransform("map", "root", tf2::TimePointZero);
      const double age = now().seconds() - stamp(t.header.stamp);
      if (age < -future_limit_ || age > stale_limit_) {return;}
      const auto & p = t.transform.translation;
      drifts_["map root"].add(now().seconds(), p.x, p.y, yaw(t.transform.rotation));
      event("map root position", stamp(t.header.stamp), p.x, p.y, yaw(t.transform.rotation));
    } catch (const tf2::TransformException &) {
      // Missing/short map-root coverage is scored at the end; subsets still run.
    }
  }
  void graph(const std::string & file)
  {
    std::ofstream out(directory_ / file);
    for (const auto & entry : get_topic_names_and_types()) {
      const auto n = count_publishers(entry.first);
      publishers_[entry.first] = std::max(publishers_[entry.first], n);
      out << entry.first << " publishers=" << n;
      for (const auto & type : entry.second) {out << ' ' << type;}
      out << '\n';
    }
    for (const auto & node : get_node_names()) {
      out << "node " << node << '\n';
      slam_running_ = slam_running_ || node.find("slam") != std::string::npos;
      amcl_running_ = amcl_running_ || node.find("amcl") != std::string::npos;
    }
    out.flush();
    if (!out) {throw std::runtime_error("graph write failed");}
  }
  void pair_latency(double wall)
  {
    auto it = pending_states_.begin();
    while (it != pending_states_.end()) {
      if (wall - it->first < 0.01) {++it; continue;}
      double distance = 0.003, capture = 0;
      for (const auto & measurement : measurements_) {
        const double separation = std::abs(it->first - measurement.first);
        if (separation <= distance) {distance = separation; capture = measurement.second;}
      }
      if (capture > 0) {
        latencies_.push_back(it->second - capture);
        event("capture-state latency proxy", capture, it->second - capture);
      }
      it = pending_states_.erase(it);
    }
    measurements_.erase(std::remove_if(measurements_.begin(), measurements_.end(),
      [wall](const auto & m) {return wall - m.first > 1;}), measurements_.end());
  }
  void camera_results(std::vector<rc::Result> & results)
  {
    results.push_back(streams_["camera"].cadence("camera delivered fps", start_, end_, camera_hz_, 0.2, 0.2));
    results.push_back(streams_["YOLO"].cadence("YOLO delivered fps", start_, end_, yolo_hz_, 0.2, 0.2));
    results.push_back(rc::latency_score(latencies_, streams_["TargetState"].count,
      future_limit_, latency_limit_));
  }
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, rc::Stream> streams_;
  std::map<std::string, rc::Drift> drifts_;
  std::map<std::string, size_t> publishers_;
  rc::Geometry geometry_;
  rc::StaleCV stale_;
  std::filesystem::path directory_;
  std::ofstream events_, logs_;
  std::vector<std::pair<double, double>> pending_states_, measurements_;
  std::vector<double> latencies_;
  double duration_, warmup_, stale_limit_, future_limit_, raw_xy_, loc_xy_, yaw_limit_;
  double pose_hz_, camera_hz_, yolo_hz_, latency_limit_, start_ = 0, end_ = 0, wall_start_ = 0;
  double last_clock_progress_ = 0, last_map_poll_ = 0;
  int bins_;
  bool slam_running_ = false, amcl_running_ = false;
  bool recording_ = false, camera_, stale_cv_, require_ref_, require_rf2o_, require_loc_;
  size_t clock_jumps_ = 0, ref_assigned_ = 0, rf2o_failed_ = 0, rf2o_no_increment_ = 0;
  size_t rf2o_reports_ = 0, slam_reports_ = 0, amcl_reports_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 2;
  try {
    result = std::make_shared<RoomCheck>()->run();
  } catch (const std::exception & e) {
    std::cerr << "FAIL room check: " << e.what() << '\n';
  }
  rclcpp::shutdown();
  return result;
}
