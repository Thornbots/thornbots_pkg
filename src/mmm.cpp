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

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "dji_serial_bridge/msg/ping.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sentry_localization/srv/save_map.hpp"
#include "slam_toolbox/srv/reset.hpp"
#include "thornbots_pkg/mmm_core.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace fs = std::filesystem;
namespace core = thornbots_pkg::mmm;
using Reset = slam_toolbox::srv::Reset;
using SaveMap = sentry_localization::srv::SaveMap;
using Steady = std::chrono::steady_clock;

class Mmm : public rclcpp::Node
{
public:
  Mmm()
  : Node("mmm"), buffer_(get_clock()), listener_(buffer_),
    protocol_([this](int n) {send(n);}, [this](int n) {dispatch(n);},
      [this](const std::string & message) {RCLCPP_ERROR(get_logger(), "%s", message.c_str());}),
    workflow_(protocol_)
  {
    mode_ = declare_parameter<std::string>("localization_mode", "mapping");
    save_dir_ = declare_parameter<std::string>("map_save_dir", "/workspaces/isaac_ros-dev/maps");
    room_ = declare_parameter<bool>("room", false);
    explicit_team_ = declare_parameter<std::string>("explicit_team", "");
    shutdown_ = declare_parameter<bool>("shutdown_after_save", false);
    request_file_ = declare_parameter<std::string>(
      "shutdown_request_file", "/run/thornbots-mmm/poweroff.request");
    timeout_ = declare_parameter<double>("timeout_s", 60.0);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || (!explicit_team_.empty() &&
      explicit_team_ != "red" && explicit_team_ != "blue"))
    {
      throw std::invalid_argument("MMM timeout/team parameters invalid");
    }
    ping_pub_ = create_publisher<dji_serial_bridge::msg::Ping>(
      "/dji_serial_bridge/ping_to_mcb", 10);
    ping_sub_ = create_subscription<dji_serial_bridge::msg::Ping>(
      "/dji_serial_bridge/ping_from_mcb", 10,
      [this](dji_serial_bridge::msg::Ping::ConstSharedPtr m) {
        protocol_.receive(m->number, wall_now());
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {odom_ = m;});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {joints_ = m;});
    ref_sub_ = create_subscription<dji_serial_bridge::msg::RefSysStatus>(
      "/dji_serial_bridge/ref_sys", rclcpp::SensorDataQoS(),
      [this](dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr m) {ref_ = m;});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {scan(m);});
    reset_ = create_client<Reset>("/slam_toolbox/reset");
    save_ = create_client<SaveMap>("/map_autosaver/save_now");
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }

private:
  static double wall_now()
  {
    return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
  }

  bool fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age >= 0 && age <= 0.5;
  }

  void send(int number)
  {
    dji_serial_bridge::msg::Ping m;
    m.header.stamp = now();
    m.number = number;
    ping_pub_->publish(m);
  }

  void dispatch(int number)
  {
    if (number == 1) {
      workflow_.start_session();
      if (saving_) {
        workflow_.setup_result(false, false);
        workflow_.tick(false, wall_now());
        RCLCPP_ERROR(get_logger(), "MMM new start rejected while save is in progress; sending 6");
        return;
      }
      ++session_generation_;
      setup_ = ready_ = reset_sent_ = reset_done_ = finished_ = false;
      scan_gate_ = core::ScanGate{};
      gate_.reset();
      restart_capture();
      odom_.reset();
      joints_.reset();
      ref_.reset();
      team_.clear();
      if (mode_ != "mapping") {
        setup_failed("running localization mode is not mapping");
        return;
      }
      try {
        const auto id = std::chrono::system_clock::now().time_since_epoch().count();
        session_ = fs::path(save_dir_) / ("mmm-" + std::to_string(id));
        if (!fs::create_directories(session_)) {
          throw std::runtime_error("session directory already exists");
        }
        setup_ = true;
        deadline_ = wall_now() + timeout_;
        RCLCPP_INFO(get_logger(), "MMM SESSION: %s", session_.c_str());
      } catch (const std::exception & e) {
        setup_failed(e.what());
      }
    } else {
      save_now();
    }
  }

  void setup_failed(const std::string & why)
  {
    setup_ = false;
    ready_ = false;
    workflow_.setup_result(mode_ == "mapping", false);
    RCLCPP_ERROR(get_logger(), "MMM SETUP FAILED: %s; sending 6", why.c_str());
  }

  void scan(sensor_msgs::msg::LaserScan::ConstSharedPtr m)
  {
    if (!reset_done_ || (!setup_ && !ready_)) {return;}
    const std::array<double, 4> geometry = {
      static_cast<double>(m->ranges.size()), m->angle_min, m->angle_max, m->angle_increment};
    if (!scan_gate_.update(geometry, rclcpp::Time(m->header.stamp).seconds(), now().seconds())) {
      if (setup_) {
        restart_capture();
        if (scan_gate_.geometry_failed()) {setup_failed("lidar geometry changed during setup");}
      }
    }
  }

  void restart_capture()
  {
    boot_gate_.reset();
    boot_samples_ = 0;
    boot_last_stamp_ = -1;
  }

  std::optional<core::Pose> current_pose(double & stamp)
  {
    try {
      const auto tf = buffer_.lookupTransform("map", "root", tf2::TimePointZero);
      stamp = rclcpp::Time(tf.header.stamp).seconds();
      const auto & p = tf.transform.translation;
      const auto & q = tf.transform.rotation;
      return core::Pose{p.x, p.y,
        std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  std::optional<double> chassis() const
  {
    if (!joints_ || !fresh(joints_->header.stamp)) {return std::nullopt;}
    for (size_t i = 0; i < joints_->name.size() && i < joints_->position.size(); ++i) {
      if (joints_->name[i] == "chassis_yaw" && std::isfinite(joints_->position[i])) {
        return joints_->position[i];
      }
    }
    return std::nullopt;
  }

  bool inputs_ready(core::SpawnCheck check)
  {
    if (!odom_ || !fresh(odom_->header.stamp) || !scan_gate_.ready(now().seconds()))
    {return false;}
    const auto & p = odom_->pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return false;}
    if (room_) {return true;}
    std::string team = explicit_team_;
    if (ref_ && ref_->robot_id && fresh(ref_->header.stamp)) {
      const std::string observed = ref_->is_on_blue_team ? "blue" : "red";
      if (!team.empty() && team != observed) {return false;}
      team = observed;
    } else if (team.empty()) {return false;}
    team_ = team;
    return core::at_team_spawn(p.x, p.y, team == "blue", check);
  }

  static double angle(double a) {return std::atan2(std::sin(a), std::cos(a));}

  void capture()
  {
    double stamp = 0;
    const auto pose = current_pose(stamp);
    const auto yaw = chassis();
    if (!pose || !yaw || !inputs_ready(core::SpawnCheck::Boot) || stamp < reset_stamp_.seconds()) {
      restart_capture();
      return;
    }
    if (!boot_gate_) {
      boot_gate_ = std::make_unique<core::StartGate>(*pose);
      boot_chassis_ = *yaw;
      raw_boot_ = {odom_->pose.pose.position.x, odom_->pose.pose.position.y};
      boot_pose_ = *pose;
    }
    if (std::abs(angle(*yaw - boot_chassis_)) > 0.03 ||
      std::hypot(odom_->pose.pose.position.x - raw_boot_[0],
      odom_->pose.pose.position.y - raw_boot_[1]) > 0.03)
    {
      restart_capture();
      return;
    }
    if (stamp > boot_last_stamp_) {++boot_samples_; boot_last_stamp_ = stamp;}
    if (!boot_gate_->update(pose, stamp, now().seconds()) || boot_samples_ < 20) {return;}
    boot_chassis_ = *yaw;
    try {
      write_metadata(stamp);
      boot_stamp_ = rclcpp::Time(static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME);
      gate_ = std::move(boot_gate_);
      setup_ = false;
      ready_ = true;
      workflow_.setup_result(true, true);
      RCLCPP_INFO(get_logger(), "MMM boot capture complete; sending ready");
    } catch (const std::exception & e) {setup_failed(e.what());}
  }

  void write_metadata(double stamp)
  {
    std::ostringstream boot;
    boot << std::setprecision(17) << "{\"stamp_ns\":" << static_cast<int64_t>(stamp * 1e9)
         << ",\"root\":[" << boot_pose_.x << ',' << boot_pose_.y << ',' << boot_pose_.yaw
         << "],\"chassis_yaw\":" << boot_chassis_ << ",\"geometry\":[";
    for (size_t i = 0; i < 4; ++i) {boot << (i ? "," : "") << (*scan_gate_.geometry())[i];}
    boot << "],\"session_id\":\"" << session_.filename().string()
         << "\",\"frame_mode\":\"" << (room_ ? "room" : "field")
         << "\",\"team\":" << (room_ ? "null" : "\"" + team_ + "\"")
         << ",\"raw_odom\":[" << raw_boot_[0] << ',' << raw_boot_[1] << "]}";
    std::ofstream out(session_ / "boot.json");
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out << boot.str();
    out.close();
    std::ofstream meta(session_ / "session.json");
    meta.exceptions(std::ios::failbit | std::ios::badbit);
    meta << "{\"session_id\":\"" << session_.filename().string()
         << "\",\"frame_mode\":\"" << (room_ ? "room" : "field") << "\"}";
  }

  bool at_start()
  {
    if (!gate_) {return false;}
    double stamp = 0;
    const auto pose = current_pose(stamp);
    return gate_->update(pose, stamp, now().seconds());
  }

  bool parked_guard()
  {
    const auto yaw = chassis();
    return at_start() && inputs_ready(core::SpawnCheck::Save) && yaw &&
           std::abs(angle(*yaw - boot_chassis_)) <= 0.03;
  }

  static void sync_path(const fs::path & path)
  {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {throw std::runtime_error("cannot open saved path for fsync");}
    const int result = ::fsync(fd);
    ::close(fd);
    if (result != 0) {throw std::runtime_error("cannot fsync saved path");}
  }

  static void sync_bundle(const fs::path & directory)
  {
    for (const auto & item : fs::directory_iterator(directory)) {sync_path(item.path());}
    sync_path(directory);
    sync_path(directory.parent_path());
  }

  void save_now()
  {
    const auto decision = core::decide_save(ready_, saving_, at_start(), protocol_.completed_even());
    if (decision == core::SaveDecision::Ignore) {
      RCLCPP_ERROR(get_logger(), "MMM save ignored: setup incomplete or save busy");
      return;
    }
    validated_ = decision == core::SaveDecision::Ready;
    if (validated_ && !parked_guard()) {
      save_finished(false, "Ready save lost fresh parked input evidence");
      return;
    }
    if (!save_->service_is_ready() || scan_gate_.geometry_failed()) {
      save_finished(false, "save service unavailable or lidar geometry invalid");
      return;
    }
    auto req = std::make_shared<SaveMap::Request>();
    output_ = session_ / ((validated_ ? "candidate-" : "incomplete-") +
      std::to_string(++save_index_));
    req->directory = output_.string();
    // Readiness is committed only after the asynchronous parked guard passes.
    req->validated = false;
    req->minimum_stamp = boot_stamp_;
    saving_ = true;
    save_guard_.begin(validated_);
    save_deadline_ = wall_now() + timeout_;
    const auto generation = ++save_generation_;
    RCLCPP_WARN(get_logger(), "MMM SAVE STARTING: DO NOT POWER OFF");
    save_request_ = save_->async_send_request(req, [this, generation](rclcpp::Client<SaveMap>::SharedFuture f) {
      if (generation != save_generation_ || !saving_) {return;}
      try {
        const auto response = f.get();
        save_guard_.observe(parked_guard());
        if (!save_guard_.accept(response && response->success)) {
          save_finished(false, response ? response->message : "empty save response");
        } else {finalize();}
      } catch (const std::exception & e) {save_finished(false, e.what());}
    }).request_id;
  }

  void finalize()
  {
    const auto candidate = output_;
    const auto session = session_;
    const bool validated = validated_;
    const auto target = validated ? session / "snapshot" : candidate;
    output_ = target;
    finalizing_ = true;
    finalization_ = std::async(std::launch::async, [candidate, session, validated, target]() {
      try {
        fs::copy_file(session / "boot.json", candidate / "boot.json");
        if (validated) {
          std::ofstream marker(candidate / "save.json.tmp");
          marker.exceptions(std::ios::failbit | std::ios::badbit);
          marker << "{\"validated\":true}\n";
          marker.close();
          fs::rename(candidate / "save.json.tmp", candidate / "save.json");
        }
        sync_bundle(candidate);
        if (validated) {
          if (fs::exists(target)) {throw std::runtime_error("snapshot already exists");}
          fs::rename(candidate, target);
          sync_path(session);
        }
        return std::string{};
      } catch (const std::exception & e) {return std::string(e.what());}
    });
  }

  void save_finished(bool success, const std::string & message)
  {
    saving_ = false;
    if (!success) {
      RCLCPP_ERROR(get_logger(), "MMM SAVE FAILED: %s", message.c_str());
      if (validated_ && fs::exists(output_)) {
        std::ofstream marker(output_ / "save.json");
        marker << "{\"validated\":false}\n";
        marker.close();
        std::error_code error;
        fs::rename(output_, session_ / ("incomplete-failed-" +
          std::to_string(++save_index_)), error);
        if (error) {RCLCPP_ERROR(get_logger(), "MMM cannot quarantine failed candidate");}
      }
      workflow_.save_result(validated_, false);
      return;
    }
    RCLCPP_INFO(get_logger(), "MMM SAVE COMPLETE: %s", output_.c_str());
    if (!validated_) {return;}
    finished_ = true;
    if (!shutdown_) {
      RCLCPP_INFO(get_logger(), "MMM Ready save complete; host shutdown request disabled");
      return;
    }
    try {
      const fs::path request(request_file_);
      std::string temporary = request.string() + ".XXXXXX";
      const int fd = ::mkstemp(temporary.data());
      if (fd < 0) {throw std::runtime_error("cannot create host request file");}
      constexpr char contents[] = "MMM_READY_SAVE_COMPLETE\n";
      const bool written = ::write(fd, contents, sizeof(contents) - 1) == sizeof(contents) - 1;
      const bool synced = ::fsync(fd) == 0;
      ::close(fd);
      if (!written || !synced) {
        fs::remove(temporary);
        throw std::runtime_error("cannot persist host request file");
      }
      fs::rename(temporary, request);
      RCLCPP_WARN(get_logger(), "MMM host shutdown requested");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "MMM saved, but host shutdown request failed: %s", e.what());
    }
  }

  void tick()
  {
    const double wall = wall_now();
    protocol_.tick(wall);
    if (setup_) {
      if (wall >= deadline_) {setup_failed("reset/boot capture timeout");}
      else if (!reset_done_ && !reset_sent_ && reset_->service_is_ready()) {
        reset_sent_ = true;
        auto req = std::make_shared<Reset::Request>();
        req->pause_new_measurements = false;
        const auto generation = session_generation_;
        reset_->async_send_request(req, [this, generation](rclcpp::Client<Reset>::SharedFuture f) {
          if (!setup_ || generation != session_generation_) {return;}
          try {
            if (f.get()->result != Reset::Response::RESULT_SUCCESS) {
              setup_failed("slam_toolbox reset rejected");
            } else {reset_done_ = true; reset_stamp_ = now();}
          } catch (const std::exception & e) {setup_failed(e.what());}
        });
      } else if (reset_done_) {capture();}
    }
    if (saving_) {
      if (validated_ && !parked_guard())
      {save_guard_.observe(false);}
      if (finalizing_ && finalization_.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        const auto error = finalization_.get();
        finalizing_ = false;
        save_guard_.observe(parked_guard());
        save_finished(save_guard_.accept(error.empty()),
          error.empty() ? "parked guard lost during save completion" : error);
      } else if (wall >= save_deadline_ && !finalizing_) {
        save_->remove_pending_request(save_request_);
        ++save_generation_;
        save_finished(false, "save service timeout; output may still finish as recovery material");
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "MMM SAVE STILL RUNNING: DO NOT POWER OFF");
      }
    }
    if (finished_ || saving_) {return;}
    workflow_.tick(ready_ && at_start(), wall);
  }

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  core::Protocol protocol_;
  core::Workflow workflow_;
  std::unique_ptr<core::StartGate> gate_, boot_gate_;
  core::Pose boot_pose_{};
  std::string mode_, save_dir_, explicit_team_, team_, request_file_;
  fs::path session_, output_;
  double timeout_, deadline_ = 0, save_deadline_ = 0, boot_chassis_ = 0;
  double boot_last_stamp_ = -1;
  std::array<double, 2> raw_boot_{};
  core::ScanGate scan_gate_;
  size_t session_generation_ = 0, save_index_ = 0, save_generation_ = 0, boot_samples_ = 0;
  int64_t save_request_ = 0;
  rclcpp::Time reset_stamp_{0, 0, RCL_ROS_TIME}, boot_stamp_{0, 0, RCL_ROS_TIME};
  bool room_, shutdown_, setup_ = false, reset_sent_ = false;
  bool reset_done_ = false, ready_ = false, saving_ = false, validated_ = false;
  core::SaveGuard save_guard_;
  bool finished_ = false, finalizing_ = false;
  std::future<std::string> finalization_;
  rclcpp::Publisher<dji_serial_bridge::msg::Ping>::SharedPtr ping_pub_;
  rclcpp::Subscription<dji_serial_bridge::msg::Ping>::SharedPtr ping_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::RefSysStatus>::SharedPtr ref_sub_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  sensor_msgs::msg::JointState::ConstSharedPtr joints_;
  dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr ref_;
  rclcpp::Client<Reset>::SharedPtr reset_;
  rclcpp::Client<SaveMap>::SharedPtr save_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mmm>());
  rclcpp::shutdown();
  return 0;
}
