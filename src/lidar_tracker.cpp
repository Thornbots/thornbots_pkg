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

// lidar_tracker node: /scan (skipping sweeps while the lidar turns fast) + saved /map -> map-frame
// robot tracks on /lidar/tracks and rviz markers on /lidar/markers. Publishing
// only; nothing here steers the head. Design: README.md#lidar_tracker.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "dji_serial_bridge/msg/target_state.hpp"
#include "dji_serial_bridge/msg/panel_detection_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "thornbots_pkg/lidar_tracker_core.hpp"
#include "thornbots_pkg/msg/lidar_track_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace thornbots_pkg
{
namespace
{
tf2::Transform transform(const geometry_msgs::msg::Transform & t)
{
  return tf2::Transform(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
    tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
}
double yaw(const tf2::Transform & t)
{
  double roll, pitch, heading;
  t.getBasis().getRPY(roll, pitch, heading);
  return heading;
}
}  // namespace
class LidarTracker : public rclcpp::Node
{
public:
  LidarTracker()
  : Node("lidar_tracker"), buffer_(get_clock()), listener_(buffer_, this, true)
  {
    lidar::Parameters parameters;
    parameters.margin = declare_parameter("lidar_obstacle_margin_m", 0.25);
    if (!std::isfinite(parameters.margin) || parameters.margin < 0 || parameters.margin > 2) {
      throw std::invalid_argument("lidar_obstacle_margin_m must be in [0, 2]");
    }
    tracker_ = lidar::Tracker(parameters);
    mode_ = declare_parameter("localization_mode", std::string("none"));
    instantaneous_ = declare_parameter("scan_is_instantaneous", false);
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    max_rate_ = declare_parameter("max_head_rate_rad_s", 0.5);
    if (!std::isfinite(max_rate_) || max_rate_ <= 0) {
      throw std::invalid_argument("max_head_rate_rad_s must be positive");
    }
    declare_parameter("lidar_looks", true);  // Reserved for stage 4; publishing has no head effect.
    pub_ = create_publisher<msg::LidarTrackArray>("/lidar/tracks", 10);
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lidar/markers", 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
        map_ = {};
        if (m->header.frame_id != "map") {return;}
        map_.width = m->info.width; map_.height = m->info.height;
        map_.resolution = m->info.resolution;
        map_.x = m->info.origin.position.x; map_.y = m->info.origin.position.y;
        const auto & q = m->info.origin.orientation;
        map_.yaw = yaw(tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w)));
        map_.cells = m->data;
      });
    camera_sub_ = create_subscription<dji_serial_bridge::msg::PanelDetectionArray>(
      "/cv/panel_detections", 10,
      [this](dji_serial_bridge::msg::PanelDetectionArray::ConstSharedPtr m) {
        camera_stamp_ = rclcpp::Time(m->header.stamp).seconds();
      });
    target_sub_ = create_subscription<dji_serial_bridge::msg::TargetState>(
      declare_parameter("target_state_topic", std::string("/cv/target_state")), 10,
      [this](dji_serial_bridge::msg::TargetState::ConstSharedPtr m) {target_ = m;});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        pending_.push_back({m, now()});
        while (pending_.size() > 8) {pending_.pop_front();}
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {drain();});
  }

private:
  using Scan = sensor_msgs::msg::LaserScan;
  void drain()
  {
    while (!pending_.empty()) {
      const auto scan = pending_.front().first;
      const auto arrival = pending_.front().second;
      if ((now() - arrival).seconds() < 0) {pending_.clear(); tracker_.clear(); return;}
      if (process(*scan)) {
        pending_.pop_front();
      } else if ((now() - arrival).seconds() > 0.5) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Dropped scan: no capture-time TF within 0.5 s (%s)", tf_error_.c_str());
        pending_.pop_front();
      } else {
        break;
      }
    }
  }
  bool process(const Scan & scan)
  {
    const rclcpp::Time start(scan.header.stamp, get_clock()->get_clock_type());
    const double duration = scan.time_increment > 0 && !scan.ranges.empty() ?
      scan.time_increment * (scan.ranges.size() - 1) : std::max(0.0F, scan.scan_time);
    const auto end = start + rclcpp::Duration::from_seconds(instantaneous_ ? 0 : duration);
    if ((mode_ != "amcl" && mode_ != "slam") || !map_.valid()) {
      tracker_.clear(); publish({}, end);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No saved field map: lidar tracks unavailable (mode=%s)", mode_.c_str());
      return true;
    }
    const double sweep_start = instantaneous_ ? start.seconds() - duration : start.seconds();
    const auto headings = sweep_headings(scan.header.frame_id, sweep_start, end.seconds());
    if (!headings) {return false;}
    if (end.seconds() > sweep_start &&
      !lidar::sweep_is_still(*headings, sweep_start, end.seconds(), max_rate_))
    {
      // Held tracks stay published (predicted, last_return unchanged).
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Skipping smeared/unknown head sweep");
      publish(tracker_.snapshot(end.seconds()), end);
      return true;
    }
    try {
      const auto first = transform(buffer_.lookupTransform("map", scan.header.frame_id,
          start).transform);
      const auto last = transform(buffer_.lookupTransform("map", scan.header.frame_id,
          end).transform);
      std::vector<std::optional<lidar::Point>> rays;
      rays.reserve(scan.ranges.size());
      for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const double range = scan.ranges[i];
        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
          rays.emplace_back(); continue;
        }
        const double fraction = scan.ranges.size() > 1 ?
          static_cast<double>(i) / (scan.ranges.size() - 1) : 0;
        tf2::Transform pose(first.getRotation().slerp(last.getRotation(), fraction),
          first.getOrigin().lerp(last.getOrigin(), fraction));
        const double angle = scan.angle_min + i * scan.angle_increment;
        const auto point = pose * tf2::Vector3(range * std::cos(angle), range * std::sin(angle), 0);
        rays.push_back(lidar::Point{point.x(), point.y()});
      }
      tracker_.update(end.seconds(), rays, map_);
      label(end);
      publish(tracker_.snapshot(end.seconds()), end);
      return true;
    } catch (const tf2::TransformException & error) {
      tf_error_ = error.what();  // usually TF a few ms behind the scan; drain() retries
      return false;
    }
  }
  // Lidar heading in odom_frame across the sweep, so chassis turns count as
  // well as head turns; nullopt until TF covers the sweep.
  std::optional<std::vector<std::pair<double, double>>> sweep_headings(
    const std::string & frame, double from, double to)
  {
    constexpr int kSteps = 8;
    std::vector<std::pair<double, double>> headings;
    for (int k = 0; k <= kSteps; ++k) {
      const double t = from + (to - from) * k / kSteps;
      const rclcpp::Time stamp(static_cast<int64_t>(std::llround(t * 1e9)),
        get_clock()->get_clock_type());
      try {
        headings.emplace_back(t,
          yaw(transform(buffer_.lookupTransform(odom_frame_, frame, stamp).transform)));
      } catch (const tf2::TransformException & error) {
        tf_error_ = error.what();
        return std::nullopt;
      }
    }
    return headings;
  }
  void label(const rclcpp::Time & end)
  {
    try {
      const auto camera = transform(buffer_.lookupTransform("map", "camera", end).transform);
      std::optional<lidar::Point> enemy;
      if (target_ && target_->valid) {
        const rclcpp::Time stamp(target_->header.stamp, get_clock()->get_clock_type());
        const double age = (end - stamp).seconds();
        if (age >= 0 && age <= 0.3) {
          const auto cv = transform(buffer_.lookupTransform("map", target_->header.frame_id,
              stamp).transform);
          const auto & c = target_->center;
          const auto & v = target_->velocity;
          const auto p = cv * tf2::Vector3(c.x + v.x * age, c.y + v.y * age, c.z + v.z * age);
          enemy = lidar::Point{p.x(), p.y()};
        }
      }
      tracker_.camera(end.seconds(), {camera.getOrigin().x(), camera.getOrigin().y()}, yaw(camera),
          enemy,
        camera_stamp_ && end.seconds() - *camera_stamp_ >= 0 &&
        end.seconds() - *camera_stamp_ <= 0.3);
    } catch (const tf2::TransformException &) {
      // Missing camera data must not suppress lidar tracks or imply an empty view.
    }
  }
  void publish(const std::vector<lidar::Track> & tracks, const rclcpp::Time & stamp)
  {
    msg::LidarTrackArray array;
    array.header.stamp = stamp; array.header.frame_id = "map";
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = array.header; clear.action = clear.DELETEALL; markers.markers.push_back(clear);
    for (const auto & t : tracks) {
      msg::LidarTrack out;
      out.header = array.header; out.id = t.id;
      out.position.x = t.position.x; out.position.y = t.position.y; out.position.z = 0.355;
      out.velocity.x = t.velocity.x; out.velocity.y = t.velocity.y;
      out.width = t.width; out.label = static_cast<uint8_t>(t.label);
      out.last_return = rclcpp::Time(static_cast<int64_t>(t.last_return * 1e9));
      array.tracks.push_back(out);
      visualization_msgs::msg::Marker marker;
      marker.header = array.header; marker.ns = "lidar_tracks"; marker.id = t.id;
      marker.type = marker.SPHERE; marker.action = marker.ADD;
      marker.pose.position = out.position; marker.pose.orientation.w = 1;
      marker.scale.x = marker.scale.y = t.width; marker.scale.z = 0.15;
      marker.color.a = 0.8;
      marker.color.r = t.label == lidar::Label::Enemy ? 1 : 0.4;
      marker.color.g = t.label == lidar::Label::Unchecked ? 1 : 0.4;
      marker.color.b = t.label == lidar::Label::CheckedEmpty ? 1 : 0.2;
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      markers.markers.push_back(marker);
    }
    pub_->publish(array); markers_->publish(markers);
  }
  std::optional<double> camera_stamp_;
  rclcpp::Subscription<dji_serial_bridge::msg::PanelDetectionArray>::SharedPtr camera_sub_;
  std::string mode_, odom_frame_, tf_error_;
  double max_rate_{};
  bool instantaneous_{};
  lidar::Grid map_;
  lidar::Tracker tracker_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::deque<std::pair<Scan::ConstSharedPtr, rclcpp::Time>> pending_;
  dji_serial_bridge::msg::TargetState::ConstSharedPtr target_;
  rclcpp::Subscription<Scan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<dji_serial_bridge::msg::TargetState>::SharedPtr target_sub_;
  rclcpp::Publisher<msg::LidarTrackArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace thornbots_pkg
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_pkg::LidarTracker>());
  rclcpp::shutdown();
  return 0;
}
