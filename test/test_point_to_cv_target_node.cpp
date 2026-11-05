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

// Node-level checks for point_to_cv_target without spinning or a ROS graph:
// the publish tick, the consumer acknowledgment and the subscription
// contract. The clock and outgoing publishes are replaced through the node's
// protected seams.

#include <gtest/gtest.h>

#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "dji_serial_bridge/msg/cv_target.hpp"
#include "dji_serial_bridge/msg/target_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "thornbots_pkg/point_to_cv_target_node.hpp"

namespace tp = thornbots_pkg;
using dji_serial_bridge::msg::CVTarget;
using dji_serial_bridge::msg::TargetState;
using std_msgs::msg::Header;

namespace
{

class Probe : public tp::PointToCvTarget
{
public:
  explicit Probe(const rclcpp::NodeOptions & options)
  : tp::PointToCvTarget(with_starts(options)) {}

  static rclcpp::NodeOptions with_starts(rclcpp::NodeOptions options)
  {
    options.append_parameter_override("red_start_x", -2.7);
    options.append_parameter_override("red_start_y", 0.6);
    options.append_parameter_override("blue_start_x", 4.1);
    options.append_parameter_override("blue_start_y", -1.2);
    return options;
  }

  using tp::PointToCvTarget::on_publish_tick;
  using tp::PointToCvTarget::on_target_state;
  using tp::PointToCvTarget::on_ref_sys;

  void gun_transform()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = muzzle_frame_;
    tf.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(tf, "test", true);
  }
  void navigation_transform(double team_start_x, double team_start_y)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "nav_odom";
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = team_start_x;
    tf.transform.translation.y = team_start_y;
    tf.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(tf, "test", true);
  }
  tp::Vector3d idle_point() {return idle_point_now(clock_now());}
  tp::IdleLook::Source idle_source() const
  {return idle_.source(clock_now().seconds(), target_timeout_s_, false);}
  tp::Vector3d last_enemy() const {return idle_.last_enemy();}
  bool active() const {return target_active_;}
  void time_ns(int64_t value) {now_ns_ = value;}
  using tp::PointToCvTarget::check_timeout;
  TargetState::ConstSharedPtr latest() const {return latest_state_;}
  void on_target_state_hook(std::function<void(const Header &)> ack) {send_ack_ = ack;}
  void capture(std::vector<CVTarget> * points, std::vector<Header> * ticks)
  {
    send_target_ = [points](const CVTarget & m) {points->push_back(m);};
    send_tick_ = [ticks](const Header & h) {ticks->push_back(h);};
  }

protected:
  rclcpp::Time clock_now() const override {return rclcpp::Time(now_ns_, RCL_ROS_TIME);}
  int64_t now_ns_ = 123000000;
};

class NodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<Probe> make()
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("idle_direction", "hold");
    return std::make_shared<Probe>(options);
  }

  static std::shared_ptr<TargetState> state(int64_t detection_ns, bool valid = true)
  {
    auto model = std::make_shared<TargetState>();
    model->header.stamp = rclcpp::Time(detection_ns, RCL_ROS_TIME);
    model->detection_stamp = model->header.stamp;
    model->header.frame_id = "odom";
    model->valid = valid;
    return model;
  }
};

}  // namespace

TEST_F(NodeTest, NoTargetTickPublishesAnIdleLookWithHitTurnPermission)
{
  auto node = make();
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(points.back().turn_to_hit);
  EXPECT_FALSE(points.back().type_c_based_patrol);  // mcb_relay owns the flag
  EXPECT_EQ(points.back().header.frame_id, "odom");
  ASSERT_EQ(ticks.size(), 1u);
  EXPECT_EQ(ticks[0], points[0].header);
  EXPECT_EQ(rclcpp::Time(ticks[0].stamp).nanoseconds(), 123000000);
}

TEST_F(NodeTest, ActiveTargetTickSendsNoPointButStillTicks)
{
  auto node = make();
  node->on_target_state(state(100000000));
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  for (int i = 0; i < 3; ++i) {
    node->time_ns(123000000 + i * 25000000);
    node->on_publish_tick();
  }
  EXPECT_TRUE(points.empty());
  ASSERT_EQ(ticks.size(), 3u);
  for (const auto & tick : ticks) {
    EXPECT_TRUE(tick.frame_id.empty());  // no point was published
  }
  EXPECT_EQ(rclcpp::Time(ticks[2].stamp).nanoseconds(), 173000000);
}

TEST_F(NodeTest, IdleResumesAfterTheTargetTimeoutFromTheDetectionStamp)
{
  auto node = make();
  auto model = state(300000000);
  model->detection_stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  node->time_ns(325000000);
  node->on_target_state(model);
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  EXPECT_TRUE(points.empty());
  // Publication time cannot refresh an old detection: 0.5 s from 0.1 s.
  node->time_ns(600000000);
  node->on_publish_tick();
  EXPECT_TRUE(points.empty());
  node->time_ns(600000001);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(points.back().turn_to_hit);
  EXPECT_FALSE(node->active());
  // A new state ends idling at once.
  node->time_ns(650000000);
  node->on_target_state(state(650000000));
  node->on_publish_tick();
  EXPECT_EQ(points.size(), 1u);
}

TEST_F(NodeTest, UnconvergedStatesStillHoldOffIdleLooks)
{
  auto node = make();
  node->on_target_state(state(100000000, false));
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  EXPECT_TRUE(points.empty());
}

TEST_F(NodeTest, LastEnemyIsTheTrackedCenterOrPanelWhenUnconverged)
{
  auto node = make();
  auto model = state(100000000);
  model->center.x = 3.0;
  model->center.y = -2.0;
  model->panel.x = 3.5;
  model->panel.y = -1.5;
  node->on_target_state(model);
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  EXPECT_EQ(node->last_enemy().x(), 3.0);
  EXPECT_EQ(node->last_enemy().y(), -2.0);
  model = state(110000000, false);
  model->center.x = 9.0;
  model->panel.x = 3.5;
  model->panel.y = -1.5;
  node->on_target_state(model);
  node->on_publish_tick();
  EXPECT_EQ(node->last_enemy().x(), 3.5);
  EXPECT_EQ(node->last_enemy().y(), -1.5);
}

TEST_F(NodeTest, IdleLooksFollowTheEnemyJustLostThenReturnToTheZone)
{
  auto node = make();
  EXPECT_EQ(node->idle_source(), tp::IdleLook::Source::Zone);
  node->time_ns(100000000);
  node->on_target_state(state(100000000));
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  node->time_ns(700000000);
  EXPECT_EQ(node->idle_source(), tp::IdleLook::Source::LastEnemy);
}

TEST_F(NodeTest, ModelAckIsSentAfterConsumptionWithTheInputStamp)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  model->detection_stamp = model->header.stamp;
  model->header.frame_id = "odom";
  std::vector<Header> acknowledged;
  node->on_target_state_hook(
    [&](const Header & header) {
      EXPECT_EQ(node->latest(), model);
      EXPECT_TRUE(node->active());
      acknowledged.push_back(header);
    });
  node->on_target_state(model);
  ASSERT_EQ(acknowledged.size(), 1u);
  EXPECT_EQ(acknowledged[0], model->header);
}

TEST_F(NodeTest, SubscribesToTargetStateAndReferee)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<Probe>(options);
  std::set<std::string> types;
  for (const auto & entry : node->get_node_graph_interface()->get_subscriber_names_and_types_by_node(
      node->get_name(), node->get_namespace()))
  {
    for (const auto & type : entry.second) {
      if (type != "tf2_msgs/msg/TFMessage" && type != "rcl_interfaces/msg/ParameterEvent") {
        types.insert(type);
      }
    }
  }
  EXPECT_EQ(
    types, (std::set<std::string>{
    "dji_serial_bridge/msg/RefSysStatus",
    "dji_serial_bridge/msg/TargetState"}));
}

TEST_F(NodeTest, RemovedPlannerParametersAreNotDeclared)
{
  auto node = std::make_shared<Probe>(rclcpp::NodeOptions());
  for (const auto * name : {"lead_enabled", "firmware_latency_s", "min_fire_delay_s",
      "gimbal_lag_s", "v_muzzle", "tof_iterations", "spin_enter_rad_s", "spin_exit_rad_s",
      "chase_settle_s", "chase_margin_s", "panel_switch_margin_rad", "fire_lock_s",
      "fire_lock_grace_s", "fire_stationary_speed_mps", "fire_max_state_age_s",
      "fire_confidence_threshold", "type_c_based_patrol", "robot_pose_topic", "root_frame"})
  {
    EXPECT_FALSE(node->has_parameter(name)) << name;
  }
}

TEST_F(NodeTest, IdleZoneUsesFieldNavigationToRawTransformForBothTeams)
{
  for (bool blue : {false, true}) {
    auto node = std::make_shared<Probe>(rclcpp::NodeOptions());
    node->gun_transform();
    node->navigation_transform(blue ? 4.1 : -2.7, blue ? -1.2 : 0.6);
    dji_serial_bridge::msg::RefSysStatus referee;
    referee.robot_id = blue ? 107 : 7;
    referee.is_on_blue_team = blue;
    referee.delta_angle_got_hit_in = 123.0F;
    node->on_ref_sys(referee);
    node->idle_point();
    node->time_ns(20000000000LL);
    const auto point = node->idle_point();
    EXPECT_NEAR(point.x(), blue ? -6.8 : 6.8, 1e-6);
    EXPECT_NEAR(point.y(), blue ? 1.8 : -1.8, 1e-6);
  }
}

TEST_F(NodeTest, ExplicitIdleZoneUsesFieldCoordinatesForEitherTeam)
{
  for (bool blue : {false, true}) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("idle_zone", std::vector<double>{0.0, -4.625});
    auto node = std::make_shared<Probe>(options);
    node->gun_transform();
    node->navigation_transform(4.625, 0.0);
    dji_serial_bridge::msg::RefSysStatus referee;
    referee.is_on_blue_team = blue;
    referee.delta_angle_got_hit_in = 123.0F;
    node->on_ref_sys(referee);
    node->idle_point();
    node->time_ns(20000000000LL);
    const auto point = node->idle_point();
    EXPECT_NEAR(point.x(), -4.625, 1e-6);
    EXPECT_NEAR(point.y(), -4.625, 1e-6);
  }
}

TEST_F(NodeTest, RejectsMalformedIdleZones)
{
  for (const auto &zone : std::vector<std::vector<double>>{
      {0.0}, {0.0, 1.0, 2.0}, {0.0, std::numeric_limits<double>::quiet_NaN()}})
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("idle_zone", zone);
    EXPECT_THROW(std::make_shared<Probe>(options), std::invalid_argument);
  }
}

TEST_F(NodeTest, HitTurnPermissionFollowsIdleVersusTargetFrames)
{
  auto node = std::make_shared<Probe>(rclcpp::NodeOptions());
  node->gun_transform();
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(points.back().turn_to_hit);
  node->on_target_state(state(100000000));
  node->on_publish_tick();
  EXPECT_EQ(points.size(), 1u);  // target state carries the aim; no hit turn frame
  node->time_ns(700000000);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 2u);
  EXPECT_TRUE(points.back().turn_to_hit);
}

TEST_F(NodeTest, MissingGunTransformDoesNotPublishAFieldPointAsRaw)
{
  auto node = std::make_shared<Probe>(rclcpp::NodeOptions());
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  EXPECT_FLOAT_EQ(points.back().x, 0.0);
  dji_serial_bridge::msg::RefSysStatus ref;
  ref.delta_angle_got_hit_in = 123.0;
  ref.is_on_blue_team = true;
  ref.robot_id = 0;
  node->on_ref_sys(ref);
  node->on_publish_tick();
  EXPECT_FLOAT_EQ(points.back().x, 0.0);
  ref.is_on_blue_team = false;
  node->on_ref_sys(ref);
  node->on_publish_tick();
  EXPECT_FLOAT_EQ(points.back().x, 0.0);
}

TEST_F(NodeTest, FloatRearHitsHoldButSentinelDoesNot)
{
  for (float angle : {static_cast<float>(M_PI), static_cast<float>(-M_PI), 123.0F}) {
    auto node = make();
    dji_serial_bridge::msg::RefSysStatus ref;
    ref.header.stamp = rclcpp::Time(int64_t{123000000}, RCL_ROS_TIME);
    ref.delta_angle_got_hit_in = angle;
    node->on_ref_sys(ref);
    EXPECT_EQ(node->idle_source() == tp::IdleLook::Source::Hit, angle != 123.0F);
    node->time_ns(2123000001);
    EXPECT_NE(node->idle_source(), tp::IdleLook::Source::Hit);
  }
}

TEST_F(NodeTest, EpochZeroStartsTimersOnlyAfterClockAndAcknowledges)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("deterministic_epoch_zero", true);
  options.append_parameter_override("tick_topic", "/epoch_test/tick");
  options.append_parameter_override("idle_direction", "hold");
  auto node = std::make_shared<Probe>(options);
  auto peer = std::make_shared<rclcpp::Node>("epoch_test_peer");
  auto clock = peer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  int ready_count = 0;
  int tick_count = 0;
  auto ready = peer->create_subscription<Header>(
    "/point_to_cv_target/epoch_zero_ready", rclcpp::QoS(1).reliable().transient_local(),
    [&](Header::ConstSharedPtr h) {
      EXPECT_EQ(h->stamp.sec, 0);
      EXPECT_EQ(h->stamp.nanosec, 0u);
      ++ready_count;
    });
  auto tick = peer->create_subscription<Header>(
    "/epoch_test/tick", 10, [&](Header::ConstSharedPtr) {++tick_count;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(peer);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  rosgraph_msgs::msg::Clock msg;
  while (ready_count == 0 && std::chrono::steady_clock::now() < deadline) {
    clock->publish(msg);
    executor.spin_some();
  }
  ASSERT_EQ(ready_count, 1);
  EXPECT_EQ(tick_count, 0);
  msg.clock.nanosec = 25000000;
  while (tick_count == 0 && std::chrono::steady_clock::now() < deadline) {
    clock->publish(msg);
    executor.spin_some();
  }
  EXPECT_EQ(tick_count, 1);
}

TEST_F(NodeTest, EpochZeroRejectsWallTime)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("deterministic_epoch_zero", true);
  EXPECT_THROW(std::make_shared<Probe>(options), std::invalid_argument);
}

TEST_F(NodeTest, EpochZeroRejectsAnAdvancedClock)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("deterministic_epoch_zero", true);
  options.append_parameter_override("idle_direction", "hold");
  auto node = std::make_shared<Probe>(options);
  auto peer = std::make_shared<rclcpp::Node>("epoch_invalid_peer");
  auto clock = peer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  int ready_count = 0;
  auto ready = peer->create_subscription<Header>(
    "/point_to_cv_target/epoch_zero_ready", rclcpp::QoS(1).reliable().transient_local(),
    [&](Header::ConstSharedPtr) {++ready_count;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(peer);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  rosgraph_msgs::msg::Clock msg;
  msg.clock.nanosec = 1000000;
  while (std::chrono::steady_clock::now() < deadline) {
    clock->publish(msg);
    executor.spin_some();
  }
  EXPECT_EQ(ready_count, 0);
}
