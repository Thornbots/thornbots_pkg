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

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "dji_serial_bridge/msg/panel_detection_array.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "thornbots_pkg/target_manager_node.hpp"
#include "thornbots_pkg/target_tracker_node.hpp"

using Message = dji_serial_bridge::msg::TargetState;
using Panels = dji_serial_bridge::msg::PanelDetectionArray;
namespace
{
class ObservedManager : public thornbots_pkg::TargetManagerNode
{
public:
  explicit ObservedManager(const rclcpp::NodeOptions & options) : TargetManagerNode(options)
  {
    observe_ = [this](thornbots_pkg::TargetEvent event, const Message &) {
        events.emplace_back(event, now().seconds());
      };
    ack_ = [this](const std_msgs::msg::Header &) {++consumed;};
  }
  std::vector<std::pair<thornbots_pkg::TargetEvent, double>> events;
  size_t consumed = 0;
};

class TrackerRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  virtual void configure_tracker(rclcpp::NodeOptions &) {}
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true);
    options.append_parameter_override("pose_latency_s", 0.0);
    configure_tracker(options);
    rclcpp::NodeOptions peer_options;
    peer_options.use_intra_process_comms(options.use_intra_process_comms());
    peer = std::make_shared<rclcpp::Node>("tracker_test_peer", peer_options);
    tracker = thornbots_pkg::make_target_tracker(options);
    rclcpp::NodeOptions manager_options;
    manager_options.append_parameter_override("use_sim_time", true);
    manager = std::make_shared<ObservedManager>(manager_options);
    clock = peer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    panels = peer->create_publisher<Panels>("/cv/robot_panels", 10);
    raw_sub = peer->create_subscription<Message>("/cv/tracker/target_state", 10,
      [this](Message::ConstSharedPtr m) {raw.push_back(*m);});
    managed_sub = peer->create_subscription<Message>("/cv/target_state", 10,
      [this](Message::ConstSharedPtr m) {managed.push_back(*m);});
    executor.add_node(peer); executor.add_node(tracker); executor.add_node(manager);
    ASSERT_TRUE(spin_until([this]() {return panels->get_subscription_count() == 1 &&
      raw_sub->get_publisher_count() == 1 && managed_sub->get_publisher_count() == 1;}));
    set_time(10000000000LL);
  }
  bool spin_until(const std::function<bool()> & done)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!done() && std::chrono::steady_clock::now() < end) {executor.spin_some();}
    return done();
  }
  void set_time(int64_t ns)
  {
    rosgraph_msgs::msg::Clock c; c.clock = rclcpp::Time(ns, RCL_ROS_TIME);
    clock->publish(c);
    ASSERT_TRUE(spin_until([this, ns]() {return tracker->now().nanoseconds() == ns &&
      manager->now().nanoseconds() == ns;}));
  }
  Panels capture(int64_t ns = 10000000000LL, float q = 0.8F)
  {
    Panels p; p.header.frame_id = "odom"; p.header.stamp = rclcpp::Time(ns, RCL_ROS_TIME);
    dji_serial_bridge::msg::PanelDetection d;
    d.center.x = 3.0; d.center.z = 0.2; d.robot_track_id = 7; d.confidence = q;
    p.detections.push_back(d); return p;
  }
  void send(const Panels & p)
  {
    const auto count = raw.size(); panels->publish(p);
    ASSERT_TRUE(spin_until([&]() {return raw.size() > count;}));
    ASSERT_TRUE(spin_until([&]() {return manager->consumed >= raw.size();}));
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> peer, tracker;
  std::shared_ptr<ObservedManager> manager;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::Publisher<Panels>::SharedPtr panels;
  rclcpp::Subscription<Message>::SharedPtr raw_sub, managed_sub;
  std::vector<Message> raw, managed;
};

class ReacquireTrackerRosTest : public TrackerRosTest
{
  void configure_tracker(rclcpp::NodeOptions & options) override
  {
    options.append_parameter_override("gate_nis", 1e-6);
    options.append_parameter_override("max_outliers", 1);
  }
};
class MissingTransformTrackerRosTest : public TrackerRosTest
{
protected:
  void configure_tracker(rclcpp::NodeOptions & options) override
  {
    options.use_intra_process_comms(true);
    options.append_parameter_override("target_state_none_topic", "/tracker_test/no_state");
    options.append_parameter_override("tf_max_wait_s", 0.25);
  }

  void SetUp() override
  {
    TrackerRosTest::SetUp();
    no_state_sub = peer->create_subscription<std_msgs::msg::Header>(
      "/tracker_test/no_state", 100,
      [this](std_msgs::msg::Header::ConstSharedPtr header) {no_state.push_back(*header);});
  }

  void send_without_transform(int64_t capture_ns)
  {
    set_time(capture_ns);
    auto p = capture(capture_ns, 1.0F);
    p.header.frame_id = "missing_camera";
    panels->publish(p);
    executor.spin_some();
    set_time(capture_ns + 260000000LL);
  }

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr no_state_sub;
  std::vector<std_msgs::msg::Header> no_state;
};
}  // namespace

TEST_F(TrackerRosTest, FirstCaptureAssignsWithoutConvergenceAndKeepsTrueStamps)
{
  const auto p = capture(); send(p);
  ASSERT_TRUE(spin_until([this]() {return !managed.empty();}));
  EXPECT_TRUE(raw.back().reset); EXPECT_TRUE(raw.back().selected_track);
  EXPECT_EQ(raw.back().update_count, 1u); EXPECT_TRUE(raw.back().valid);
  EXPECT_EQ(raw.back().detection_stamp, p.header.stamp);
  EXPECT_EQ(managed.back().header, raw.back().header);
  EXPECT_EQ(managed.back().detection_stamp, raw.back().detection_stamp);
}

TEST_F(TrackerRosTest, AcceptedPanelMaximumDecaysFromCaptureAndOutlierCannotRefresh)
{
  set_time(10100000000LL);
  auto p = capture(); p.detections.push_back(p.detections.front());
  p.detections[0].confidence = 0.7F; p.detections[1].confidence = 0.9F;
  send(p);
  EXPECT_NEAR(raw.back().confidence, 0.9 * std::exp(-0.1 / 0.3), 1e-6);
  EXPECT_EQ(raw.back().update_count, 1u);
  auto outlier = capture(10100000000LL, 1.0F); outlier.detections[0].center.y = 2.0;
  send(outlier);
  EXPECT_EQ(raw.back().detection_stamp, p.header.stamp);
  EXPECT_EQ(raw.back().update_count, 1u);
}

TEST_F(TrackerRosTest, EmptyInputPreservesEvidenceThenRetiresWithExplicitStatus)
{
  auto empty = capture();
  empty.detections.clear();
  send(empty);
  EXPECT_FALSE(raw.back().selected_track);
  EXPECT_FALSE(raw.back().valid);
  EXPECT_EQ(raw.back().update_count, 0u);

  send(capture());
  ASSERT_TRUE(raw.back().valid);
  const auto original = raw.back();

  set_time(10050000000LL);
  empty.header.stamp = tracker->now();
  send(empty);
  EXPECT_TRUE(raw.back().selected_track);
  EXPECT_FALSE(raw.back().reset);
  EXPECT_EQ(raw.back().header, original.header);
  EXPECT_EQ(raw.back().detection_stamp, original.detection_stamp);
  EXPECT_EQ(raw.back().update_count, original.update_count);
  EXPECT_EQ(raw.back().input_stamp, empty.header.stamp);

  set_time(10525000000LL);
  empty.header.stamp = tracker->now();
  send(empty);
  EXPECT_FALSE(raw.back().selected_track);
  EXPECT_FALSE(raw.back().valid);
  EXPECT_TRUE(raw.back().reset);
}

TEST_F(MissingTransformTrackerRosTest, MissingTransformCannotSeedOrRefreshEvidence)
{
  send_without_transform(10000000000LL);
  ASSERT_TRUE(spin_until([this]() {return raw.size() == 1;}));
  EXPECT_FALSE(raw.back().valid);
  EXPECT_EQ(raw.back().update_count, 0u);

  constexpr int64_t valid_capture = 10300000000LL;
  set_time(valid_capture);
  send(capture(valid_capture));
  ASSERT_TRUE(raw.back().valid);
  const auto original = raw.back();
  const auto model_count = raw.size();

  constexpr int64_t dropped_capture = 10350000000LL;
  send_without_transform(dropped_capture);
  ASSERT_TRUE(spin_until([this]() {return !no_state.empty();}));
  EXPECT_EQ(rclcpp::Time(no_state.back().stamp).nanoseconds(), dropped_capture);
  EXPECT_EQ(raw.size(), model_count);

  auto empty = capture(tracker->now().nanoseconds());
  empty.detections.clear();
  send(empty);
  EXPECT_TRUE(raw.back().selected_track);
  EXPECT_EQ(raw.back().header, original.header);
  EXPECT_EQ(raw.back().detection_stamp, original.detection_stamp);
  EXPECT_EQ(raw.back().update_count, original.update_count);

  // This capture is past the original timeout, but within it of the dropped capture.
  constexpr int64_t expired_capture = 10825000000LL;
  const auto held_count = raw.size();
  send_without_transform(expired_capture);
  ASSERT_TRUE(spin_until([this, held_count]() {return raw.size() > held_count;}));
  EXPECT_FALSE(raw.back().valid);
  EXPECT_TRUE(raw.back().reset);
  EXPECT_EQ(raw.back().update_count, 0u);

  empty.header.stamp = tracker->now();
  send(empty);
  EXPECT_FALSE(raw.back().selected_track);
  EXPECT_FALSE(raw.back().valid);
}

TEST_F(TrackerRosTest, IdentityAndBackwardCaptureResetTheConfidenceAnchor)
{
  auto p = capture(); send(p);
  set_time(10010000000LL); p = capture(10010000000LL, 0.3F);
  p.detections[0].robot_track_id = 99; send(p);
  EXPECT_TRUE(raw.back().reset); EXPECT_EQ(raw.back().update_count, 1u);
  EXPECT_NEAR(raw.back().confidence, 0.3, 1e-6);
  p = capture(10000000000LL, 0.2F); p.detections[0].robot_track_id = 99; send(p);
  EXPECT_TRUE(raw.back().reset); EXPECT_EQ(raw.back().update_count, 1u);
  EXPECT_NEAR(raw.back().confidence, 0.2 * std::exp(-0.01 / 0.3), 1e-6);
}

TEST_F(ReacquireTrackerRosTest, PositionReacquisitionExposesResetAndStartsANewConfidenceAnchor)
{
  send(capture());
  bool saw_reset = false;
  for (int i = 1; i <= 3; ++i) {
    const int64_t at = 10000000000LL + static_cast<int64_t>(i) * 10000000LL;
    set_time(at); auto p = capture(at, 0.3F); p.detections[0].center.y = 2.0;
    send(p);
    if (raw.back().reset) {
      saw_reset = true;
      EXPECT_EQ(raw.back().update_count, 1u);
      EXPECT_NEAR(raw.back().confidence, 0.3, 1e-6);
    }
  }
  EXPECT_TRUE(saw_reset);
}

TEST_F(TrackerRosTest, SimDetectionStreamHandoffMeasurements)
{
  using Event = thornbots_pkg::TargetEvent;
  std::mt19937 random(76);
  std::normal_distribution<double> noise(0.0, 0.005);
  int frame = 0;
  double last_delivery = 10.0, last_capture = 10.0;
  for (int ms = 0; ms < 10000; ms += 5) {
    set_time(10000000000LL + static_cast<int64_t>(ms) * 1000000LL);
    if (ms * 60 / 1000 < frame) {continue;}
    const int64_t capture_ns = tracker->now().nanoseconds() - 40000000LL;
    auto p = capture(capture_ns, 0.85F);
    const double t = ms * 0.001;
    // A seeded sim detector: parked, translating, then rigidly spinning armor.
    const double y = t < 3.0 ? 0.0 : (t < 6.0 ? (t - 3.0) * 0.5 : 1.5);
    const double yaw = t < 6.0 ? M_PI : M_PI + (t - 6.0) * 6.0;
    const double bearing = std::atan2(-y, -3.0);
    const double k = std::round((bearing - yaw) / (M_PI / 2.0));
    const double facing_yaw = yaw + k * M_PI / 2.0;
    p.detections[0].center.x = 3.0 + 0.2 * std::cos(facing_yaw) + noise(random);
    p.detections[0].center.y = y + 0.2 * std::sin(facing_yaw) + noise(random);
    p.detections[0].center.z = 0.2 + noise(random);
    send(p);
    ++frame; last_delivery = tracker->now().seconds(); last_capture = capture_ns * 1e-9;
  }
  for (int ms = 10000; ms <= 10400; ms += 5) {
    set_time(10000000000LL + static_cast<int64_t>(ms) * 1000000LL);
    executor.spin_some();
  }
  size_t assign = 0, update = 0, refresh = 0, clear = 0;
  double first = -1, clear_time = -1;
  for (const auto & [event, at] : manager->events) {
    if (event == Event::Assign) {++assign; if (first < 0) {first = at;}}
    if (event == Event::Update) {++update;}
    if (event == Event::Refresh) {++refresh;}
    if (event == Event::Clear) {++clear; clear_time = at;}
  }
  std::cout << "HANDOFF_MEASUREMENTS frames=" << frame << " raw=" << raw.size()
            << " assign=" << assign << " update=" << update << " refresh=" << refresh
            << " clear=" << clear << " first_from_capture_ms=" << (first - 9.96) * 1000.0
            << " clear_from_delivery_ms=" << (clear_time - last_delivery) * 1000.0
            << " clear_from_capture_ms=" << (clear_time - last_capture) * 1000.0 << std::endl;
  EXPECT_GE(assign, 1u); EXPECT_GE(clear, 1u);
  EXPECT_LT(assign + update + refresh, raw.size());
  EXPECT_GE(first, 10.0);
  EXPECT_GE(clear_time, last_delivery);
  EXPECT_LE(clear_time - last_capture, 0.30 * std::log(0.85 / 0.4) + 0.006);
}
