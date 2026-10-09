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

// Pick the single panel to shoot at from all detections. /cv/panel_detections
// (PanelDetectionArray, 3D) -> team filter -> per-cluster best panel ->
// robot-level hysteresis -> the winner on /cv/panel_detection with its corners
// on /cv/panel_polygon, and all of that robot's panels, winner first, on
// /cv/robot_panels for target_tracker. Logic is in target_selector_core.hpp.
// See README.md's ### target_selector Notes.

#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "dji_serial_bridge/msg/panel_detection.hpp"
#include "dji_serial_bridge/msg/panel_detection_array.hpp"
#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "thornbots_pkg/target_selector_core.hpp"

namespace thornbots_pkg
{

using dji_serial_bridge::msg::PanelDetection;
using dji_serial_bridge::msg::PanelDetectionArray;
using dji_serial_bridge::msg::RefSysStatus;
using geometry_msgs::msg::PolygonStamped;

class TargetSelector : public rclcpp::Node
{
public:
  TargetSelector()
  : Node("target_selector"), hysteresis_(0.0, 0)
  {
    const auto panel_array_topic = declare_parameter(
      "panel_array_topic", std::string("/cv/panel_detections"));
    const auto panel_topic = declare_parameter("panel_topic", std::string("/cv/panel_detection"));
    const auto robot_panels_topic = declare_parameter(
      "robot_panels_topic", std::string("/cv/robot_panels"));
    const auto polygon_topic = declare_parameter(
      "polygon_topic", std::string("/cv/panel_polygon"));
    ref_sys_topic_ = declare_parameter(
      "ref_sys_topic", std::string("/dji_serial_bridge/ref_sys"));
    min_score_ = declare_parameter("min_score", 0.0);
    center_weight_ = declare_parameter("center_weight", 1.0);
    priority_class_bonus_ = declare_parameter("priority_class_bonus", 0.5);
    const auto ids = declare_parameter("priority_class_ids", std::vector<int64_t>{2, 6});
    for (auto c : ids) {
      priority_class_ids_.insert(static_cast<int>(c));
    }
    centrality_max_angle_rad_ = declare_parameter(
      "centrality_max_angle_rad", 45.0 * M_PI / 180.0);
    panel_group_radius_m_ = declare_parameter("panel_group_radius_m", 0.5);
    const double switch_margin = declare_parameter("switch_margin", 0.3);
    const int switch_hold_frames = static_cast<int>(declare_parameter("switch_hold_frames", 5));
    hysteresis_ = RobotHysteresis(switch_margin, switch_hold_frames);

    pub_ = create_publisher<PanelDetection>(panel_topic, 10);
    robot_panels_pub_ = create_publisher<PanelDetectionArray>(robot_panels_topic, 10);
    polygon_pub_ = create_publisher<PolygonStamped>(polygon_topic, 10);
    array_sub_ = create_subscription<PanelDetectionArray>(
      panel_array_topic, 10,
      [this](PanelDetectionArray::ConstSharedPtr msg) {on_array(*msg);});
    // Matches dji_serial_bridge_node's ~/ref_sys SensorDataQoS publisher.
    ref_sys_sub_ = create_subscription<RefSysStatus>(
      ref_sys_topic_, rclcpp::SensorDataQoS(),
      [this](RefSysStatus::ConstSharedPtr msg) {on_ref_sys(*msg);});

    std::string ids_str;
    for (int c : priority_class_ids_) {
      ids_str += (ids_str.empty() ? "" : ", ") + std::to_string(c);
    }
    RCLCPP_INFO(
      get_logger(),
      "target_selector ready\n  %s -> %s, %s, %s\n"
      "  score = conf + %g*centrality + %g if class in [%s]"
      "  (min_score=%g gates on raw confidence only)\n"
      "  panel_group_radius_m=%g\n  team source: %s",
      panel_array_topic.c_str(), panel_topic.c_str(), robot_panels_topic.c_str(),
      polygon_topic.c_str(), center_weight_, priority_class_bonus_, ids_str.c_str(), min_score_,
      panel_group_radius_m_, ref_sys_topic_.c_str());
  }

private:
  void on_ref_sys(const RefSysStatus & msg)
  {
    const auto new_val = team_from_ref_sys(msg.robot_id, msg.is_on_blue_team);
    if (new_val != is_blue_team_) {
      if (!new_val) {
        RCLCPP_INFO(
          get_logger(), "Team colour unknown (robot_id 0): passing all detections through");
      } else {
        RCLCPP_INFO(
          get_logger(), "Team colour set to %s (excluding class IDs %s)",
          *new_val ? "BLUE" : "RED", *new_val ? "0-3" : "4-7");
      }
    }
    is_blue_team_ = new_val;
  }

  void on_array(const PanelDetectionArray & msg)
  {
    struct Candidate
    {
      Point3 pos;
      double score;
      const PanelDetection * det;
    };
    std::vector<Candidate> candidates;
    for (const auto & det : msg.detections) {
      if (!eligible(det.confidence, det.class_id, is_blue_team_, min_score_)) {
        continue;
      }
      const double centrality = centrality_3d(
        det.center.x, det.center.y, det.center.z, centrality_max_angle_rad_);
      const double score = compute_score(
        det.confidence, centrality, det.class_id, center_weight_, priority_class_bonus_,
        priority_class_ids_);
      candidates.push_back({{det.center.x, det.center.y, det.center.z}, score, &det});
    }

    if (candidates.empty()) {
      if (!is_blue_team_ && !msg.detections.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No robot ID on '%s' yet -- team colour unknown, passing all detections through",
          ref_sys_topic_.c_str());
      }
      return;
    }

    std::vector<Point3> points;
    for (const auto & c : candidates) {
      points.push_back(c.pos);
    }
    const auto clusters = group_panels(points, panel_group_radius_m_);
    std::vector<ClusterInfo> infos;
    std::vector<const std::vector<int> *> members;  // by cluster key (best index)
    members.assign(candidates.size(), nullptr);
    for (const auto & indices : clusters) {
      // Best panel within this cluster, per-frame, no stickiness.
      int best_idx = indices[0];
      for (int i : indices) {
        if (candidates[i].score > candidates[best_idx].score) {
          best_idx = i;
        }
      }
      infos.push_back({best_idx, cluster_centroid(points, indices), candidates[best_idx].score});
      members[best_idx] = &indices;
    }

    const auto winner_key = hysteresis_.update(infos);
    if (!winner_key) {
      return;
    }

    PanelDetection winner = *candidates[*winner_key].det;
    winner.robot_track_id = hysteresis_.track_id();
    pub_->publish(winner);

    PolygonStamped polygon;
    polygon.header = winner.header;
    polygon.polygon.points.assign(winner.corners.begin(), winner.corners.end());
    polygon_pub_->publish(polygon);

    // Every panel of the winning robot, winner first: target_tracker's armor
    // model needs them all, since one per-frame pick flips between two visible
    // panels and breaks the spin estimate. See README.md.
    PanelDetectionArray robot;
    robot.header = msg.header;
    robot.detections.push_back(winner);
    for (int i : *members[*winner_key]) {
      if (i != *winner_key) {
        PanelDetection det = *candidates[i].det;
        det.robot_track_id = winner.robot_track_id;
        robot.detections.push_back(det);
      }
    }
    robot_panels_pub_->publish(robot);
  }

  std::string ref_sys_topic_;
  double min_score_, center_weight_, priority_class_bonus_;
  double centrality_max_angle_rad_, panel_group_radius_m_;
  std::set<int> priority_class_ids_;
  RobotHysteresis hysteresis_;
  std::optional<bool> is_blue_team_;  // nullopt until a RefSysStatus carries a robot ID
  rclcpp::Publisher<PanelDetection>::SharedPtr pub_;
  rclcpp::Publisher<PanelDetectionArray>::SharedPtr robot_panels_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::Subscription<PanelDetectionArray>::SharedPtr array_sub_;
  rclcpp::Subscription<RefSysStatus>::SharedPtr ref_sys_sub_;
};

}  // namespace thornbots_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_pkg::TargetSelector>());
  rclcpp::shutdown();
  return 0;
}
