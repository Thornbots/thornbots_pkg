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

// Track the selected robot as a spinning 4-panel armor model.
// /cv/robot_panels (target_selector's robot, winner first) -> /cv/target_state
// (TargetState, odom), from armor_tracker.hpp's ArmorTracker. Capture time is
// the detection stamp less camera_latency_s; each detection waits (up to
// tf_max_wait_s) for the camera's TF at that time, and the state is predicted
// to its publish time and stamped with it. /cv/tracker/measurement echoes each
// folded-in detection's header, so a bench can pace on the tracker's input.
// See README.md's ### target_tracker.

#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "dji_serial_bridge/msg/panel_detection_array.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "thornbots_pkg/armor_tracker.hpp"

namespace thornbots_pkg
{

using dji_serial_bridge::msg::PanelDetectionArray;
using dji_serial_bridge::msg::TargetState;
using namespace std::chrono_literals;

class TargetTracker : public rclcpp::Node
{
public:
  TargetTracker()
  : Node("target_tracker")
  {
    robot_panels_topic_ = declare_parameter("robot_panels_topic", std::string("/cv/robot_panels"));
    output_topic_ = declare_parameter("output_topic", std::string("/cv/target_state"));
    measurement_topic_ = declare_parameter(
      "measurement_topic", std::string("/cv/tracker/measurement"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    pose_latency_s_ = declare_parameter("pose_latency_s", 0.01);
    // Unmeasured on hardware (CV_SPLIT_PLAN.md, Estimation); match the emulator's in sim.
    camera_latency_s_ = declare_parameter("camera_latency_s", 0.0);
    track_max_gap_s_ = declare_parameter("track_max_gap_s", 0.5);
    // How long a detection waits for the camera's TF at its capture time
    // before it is dropped; never matched to a newer pose. See README.md.
    tf_max_wait_s_ = declare_parameter("tf_max_wait_s", 0.25);
    params_.ekf.radius = declare_parameter("panel_radius_m", 0.27);  // both pairs, initial
    meas_noise_base_m_ = declare_parameter("meas_noise_base_m", 0.03);
    meas_noise_range_coeff_ = declare_parameter("meas_noise_range_coeff", 0.01);  // * range^2
    meas_noise_lateral_m_ = declare_parameter("meas_noise_lateral_m", 0.04);  // across the ray
    params_.ekf.q_accel = declare_parameter("process_noise_accel", 2.0);  // m/s^2, centre
    // White jerk (m/s^3) on a Singer acceleration decaying over
    // accel_time_constant_s; kept well under the spin's bandwidth.
    params_.ekf.q_jerk = declare_parameter("process_noise_jerk", 3.0);
    params_.ekf.accel_tau_s = declare_parameter("accel_time_constant_s", 1.0);
    // A panel seen alone faces the camera within about this (rad).
    single_panel_yaw_std_ = declare_parameter("single_panel_yaw_std", 0.3);
    params_.ekf.q_yaw_accel = declare_parameter("process_noise_yaw_accel", 5.0);  // rad/s^2
    params_.ekf.q_radius = declare_parameter("process_noise_radius", 0.02);  // m/sqrt(s)
    // A parked, non-spinning hypothesis; hands the lead back once the summed
    // log-likelihood ratio passes still_exit_llr. See README.md.
    params_.still = declare_parameter("still_hypothesis", true);
    params_.ekf.q_still_pos = declare_parameter("still_process_noise_pos", 0.02);
    params_.ekf.q_still_yaw = declare_parameter("still_process_noise_yaw", 0.05);
    params_.still_exit_llr = declare_parameter("still_exit_llr", 15.0);
    // chi-square(3) gate; max_outliers in a row re-seed the position.
    gate_nis_ = declare_parameter("gate_nis", 16.3);
    max_outliers_ = static_cast<int>(declare_parameter("max_outliers", 3));

    // The listener's own callback group and thread: a slow update can't back
    // up /tf, and lookups here never block. See README.md.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    pub_ = create_publisher<TargetState>(output_topic_, 10);
    measurement_pub_ = create_publisher<std_msgs::msg::Header>(measurement_topic_, 10);
    // Set by the estimation bench only: echo each /clock update once now()
    // reads it, so a lockstep bench sends detections only after it lands.
    const auto clock_ack_topic = declare_parameter("clock_ack_topic", std::string(""));
    if (!clock_ack_topic.empty()) {
      clock_ack_pub_ = create_publisher<std_msgs::msg::Header>(clock_ack_topic, 10);
      rcl_jump_threshold_t every_update{};
      every_update.min_forward.nanoseconds = 1;
      clock_jump_ = get_clock()->create_jump_callback(
        nullptr, [this](const rcl_time_jump_t &) {
          std_msgs::msg::Header h;
          h.stamp = now();
          clock_ack_pub_->publish(h);
        }, every_update);
    }
    sub_ =create_subscription<PanelDetectionArray>(
      robot_panels_topic_, 10, [this](PanelDetectionArray::ConstSharedPtr msg) {
        if (!msg->detections.empty()) {
          waiting_.emplace_back(msg, now());
          drain();
        }
      });
    // Wall-clock retries: a bench that holds sim time until this node
    // publishes would never let a sim-time retry fire.
    drain_timer_ = create_wall_timer(5ms, [this]() {drain();});
    log_timer_ = rclcpp::create_timer(this, get_clock(), 5s, [this]() {log_tf_waits();});

    RCLCPP_INFO(
      get_logger(),
      "target_tracker ready (C++)\n  %s -> %s (frame=%s)\n  pose_latency_s=%.3f "
      "camera_latency_s=%.3f track_max_gap_s=%.2f tf_max_wait_s=%.2f\n  panel_radius_m=%.2f",
      robot_panels_topic_.c_str(), output_topic_.c_str(), odom_frame_.c_str(), pose_latency_s_,
      camera_latency_s_, track_max_gap_s_, tf_max_wait_s_, params_.ekf.radius);
  }

private:
  rclcpp::Time capture_time(const PanelDetectionArray & msg) const
  {
    return rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()) -
           rclcpp::Duration::from_seconds(camera_latency_s_);
  }

  // Process waiting detections, oldest first, once TF covers each capture time.
  void drain()
  {
    while (!waiting_.empty()) {
      const auto & [msg, arrival] = waiting_.front();
      const double waited_s = (now() - arrival).seconds();
      const std::string camera_frame =
        msg->header.frame_id.empty() ? "camera" : msg->header.frame_id;
      const rclcpp::Time query = capture_time(*msg) +
        rclcpp::Duration::from_seconds(pose_latency_s_);
      std::string error;
      auto tf = lookup(camera_frame, query, error);
      if (!tf) {
        if (waited_s <= tf_max_wait_s_ && tf_behind(camera_frame, query)) {
          return;  // TF hasn't reached the capture time yet
        }
        // The listener thread may have caught up since the first lookup.
        tf = lookup(camera_frame, query, error);
      }
      if (!tf) {
        ++tf_drops_;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TF %s<-%s at capture %.3f unavailable after %.3f s (tf_max_wait_s=%.2f) -- "
          "dropping: %s", odom_frame_.c_str(), camera_frame.c_str(), query.seconds(), waited_s,
          tf_max_wait_s_, error.c_str());
        waiting_.pop_front();
        continue;
      }
      const auto next = msg;
      waiting_.pop_front();
      tf_waits_.push_back(waited_s);
      update(*next, *tf);
    }
  }

  std::optional<geometry_msgs::msg::TransformStamped> lookup(
    const std::string & camera_frame, const rclcpp::Time & query, std::string & error) const
  {
    try {
      return tf_buffer_->lookupTransform(odom_frame_, camera_frame, query);
    } catch (const tf2::TransformException & ex) {
      error = ex.what();
      return std::nullopt;
    }
  }

  // True if the newest camera TF is older than query (worth waiting for).
  bool tf_behind(const std::string & camera_frame, const rclcpp::Time & query) const
  {
    try {
      const auto newest = tf_buffer_->lookupTransform(
        odom_frame_, camera_frame, tf2::TimePointZero);
      return rclcpp::Time(newest.header.stamp, query.get_clock_type()) < query;
    } catch (const tf2::TransformException &) {
      return true;  // no TF yet at all: the tree may still be coming up
    }
  }

  void log_tf_waits()
  {
    if (tf_waits_.empty() && tf_drops_ == 0) {
      return;
    }
    auto mean = [](const std::vector<double> & v) {
        return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
      };
    auto max = [](const std::vector<double> & v) {
        return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());
      };
    RCLCPP_INFO(
      get_logger(), "camera TF wait over %zu detections: mean %.3f s, max %.3f s; dropped %d; "
      "capture to update mean %.3f s, max %.3f s", tf_waits_.size(), mean(tf_waits_),
      max(tf_waits_), tf_drops_, mean(lags_), max(lags_));
    tf_waits_.clear();
    lags_.clear();
    tf_drops_ = 0;
  }

  void update(const PanelDetectionArray & msg, const geometry_msgs::msg::TransformStamped & tf)
  {
    const auto & first = msg.detections.front();
    const rclcpp::Time stamp = capture_time(msg);
    if (!track_id_ || first.robot_track_id != *track_id_ ||
      (last_stamp_ && (stamp - *last_stamp_).seconds() > track_max_gap_s_))
    {
      track_id_ = first.robot_track_id;
      tracker_.reset();
      n_updates_ = 0;
    }
    last_stamp_ = stamp;

    const auto & t = tf.transform.translation;
    const auto & q = tf.transform.rotation;
    const Eigen::Matrix3d rot = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    const Vector3d T(t.x, t.y, t.z);
    const double t_sec = stamp.seconds();
    const double facing_std = msg.detections.size() == 1 ? single_panel_yaw_std_ : 0.0;
    std::optional<Vector3d> first_panel;
    for (const auto & det : msg.detections) {
      const Vector3d panel_cam(det.center.x, det.center.y, det.center.z);
      const Vector3d panel = rot * panel_cam + T;
      const double range = panel_cam.norm();
      const double stddev = meas_noise_base_m_ + meas_noise_range_coeff_ * range * range;
      if (!first_panel) {
        first_panel = panel;
      }
      const Matrix3d R_meas = ray_covariance(panel, T, stddev, meas_noise_lateral_m_);
      if (!tracker_) {
        tracker_.emplace(panel, T, t_sec, R_meas, params_);
        continue;
      }
      const auto result =
        tracker_->step(panel, T, t_sec, R_meas, gate_nis_, max_outliers_, facing_std);
      if (result == StepResult::Reacquire) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "track %u: %d outliers in a row, re-seeding position (spin estimate kept)",
          *track_id_, max_outliers_);
      }
    }
    ++n_updates_;

    // TargetState describes the target now: predict to the publish time.
    const rclcpp::Time now_t = std::max(now(), stamp);
    lags_.push_back((now_t - stamp).seconds());
    const auto [s, P] = tracker_->predicted(now_t.seconds());
    TargetState out;
    out.header.stamp = now_t;
    out.header.frame_id = odom_frame_;
    out.robot_track_id = first.robot_track_id;
    out.confidence = first.confidence;
    out.center.x = s[POS];
    out.center.y = s[POS + 1];
    out.center.z = s[POS + 2];
    out.velocity.x = s[VEL];
    out.velocity.y = s[VEL + 1];
    out.velocity.z = s[VEL + 2];
    out.acceleration.x = s[ACC];
    out.acceleration.y = s[ACC + 1];
    out.acceleration.z = s[ACC + 2];
    for (int i = 0; i < 3; ++i) {
      out.variance[i] = static_cast<float>(P(POS + i, POS + i));
      out.variance[3 + i] = static_cast<float>(P(VEL + i, VEL + i));
    }
    out.panel.x = first_panel->x();
    out.panel.y = first_panel->y();
    out.panel.z = first_panel->z();
    out.yaw = static_cast<float>(s[YAW]);
    out.yaw_rate = static_cast<float>(s[W]);
    out.yaw_rate_variance = static_cast<float>(P(W, W));
    out.radius = {static_cast<float>(s[R]), static_cast<float>(tracker_->other_r())};
    out.z_offset = {static_cast<float>(s[DZ]), static_cast<float>(-s[DZ])};
    // Two updates before consumers lead on it; they weigh the variances for anything finer.
    out.valid = n_updates_ >= 2;
    pub_->publish(out);
    measurement_pub_->publish(msg.header);
  }

  std::string robot_panels_topic_, output_topic_, measurement_topic_, odom_frame_;
  double pose_latency_s_, camera_latency_s_, track_max_gap_s_, tf_max_wait_s_;
  double meas_noise_base_m_, meas_noise_range_coeff_, meas_noise_lateral_m_;
  double single_panel_yaw_std_, gate_nis_;
  int max_outliers_;
  TrackerParams params_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<TargetState>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr measurement_pub_, clock_ack_pub_;
  rclcpp::JumpHandler::SharedPtr clock_jump_;
  rclcpp::Subscription<PanelDetectionArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr drain_timer_, log_timer_;

  std::deque<std::pair<PanelDetectionArray::ConstSharedPtr, rclcpp::Time>> waiting_;
  std::vector<double> tf_waits_, lags_;  // s: TF wait, capture to update
  int tf_drops_ = 0;
  std::optional<uint32_t> track_id_;
  std::optional<rclcpp::Time> last_stamp_;
  std::optional<ArmorTracker> tracker_;
  int n_updates_ = 0;
};

}  // namespace thornbots_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_pkg::TargetTracker>());
  rclcpp::shutdown();
  return 0;
}
