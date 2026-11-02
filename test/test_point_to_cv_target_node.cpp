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
// contract. The clock, outgoing publishes and aim/fire decisions are replaced
// through the node's protected seams.

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
  void set_active(bool active) {target_active_ = active;}
  void non_firing_target() {fire_target_ = false;}
  void firing_target() {fire_target_ = true;}
  void fail_aim() {fail_aim_ = true;}
  void real_plan() {real_plan_ = true;}
  using tp::PointToCvTarget::fire_decision;
  void root_transform()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = root_frame_;
    tf.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(tf, "test", true);
  }
  tp::IdleLook::Source idle_source() const
  {return idle_.source(clock_now().seconds(), target_timeout_s_, false);}
  bool active() const {return target_active_;}
  void hold(double yaw) {held_panel_yaw_ = yaw;}
  bool held() const {return held_panel_yaw_.has_value();}
  void time_ns(int64_t value) {now_ns_ = value;}
  using tp::PointToCvTarget::check_timeout;
  void base_plan() {tp::PointToCvTarget::compute_aim_point();}
  TargetState::ConstSharedPtr latest() const {return latest_state_;}
  void on_target_state_hook(std::function<void(const Header &)> ack) {send_ack_ = ack;}
  void capture(std::vector<CVTarget> * points, std::vector<Header> * ticks)
  {
    send_target_ = [points](const CVTarget & m) {points->push_back(m);};
    send_tick_ = [ticks](const Header & h) {ticks->push_back(h);};
  }

protected:
  std::optional<Aim> compute_aim_point() override
  {
    if (real_plan_) {return tp::PointToCvTarget::compute_aim_point();}
    if (fail_aim_) {return std::nullopt;}
    return Aim{tp::Vector3d(3.0, 1.0, 0.3),
      fire_target_ ? std::optional<double>(0.0) : std::nullopt};
  }
  rclcpp::Time clock_now() const override {return rclcpp::Time(now_ns_, RCL_ROS_TIME);}
  int64_t now_ns_ = 123000000;
  bool fire_target_ = true;
  bool fail_aim_ = false;
  bool real_plan_ = false;
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
};

}  // namespace

TEST_F(NodeTest, NoTargetTickDoesNotRequireAnAbsentAimPoint)
{
  auto node = make();
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->set_active(false);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(points.back().patrol);
  EXPECT_TRUE(points.back().turn_to_hit);
  EXPECT_FALSE(points.back().fire);
  ASSERT_EQ(ticks.size(), 1u);
  EXPECT_EQ(ticks[0].frame_id, "odom");
  EXPECT_EQ(rclcpp::Time(ticks[0].stamp).nanoseconds(), 123000000);
}

TEST_F(NodeTest, TargetTickIdentifiesTheOutputFrameAndStamp)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  model->detection_stamp = model->header.stamp;
  model->header.frame_id = "odom";
  model->confidence = 0.5;
  model->valid = true;
  node->on_target_state(model);
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  ASSERT_EQ(ticks.size(), 1u);
  EXPECT_EQ(ticks[0], points[0].header);
  EXPECT_EQ(ticks[0].frame_id, "odom");
  EXPECT_EQ(rclcpp::Time(points[0].header.stamp).nanoseconds(), 123000000);
  EXPECT_NE(points[0].header.stamp, model->header.stamp);
  EXPECT_TRUE(points[0].fire);
  EXPECT_FALSE(points[0].patrol);
  EXPECT_FALSE(points[0].turn_to_hit);
}

TEST_F(NodeTest, EveryFortyHzTickFiresWithAValidWireDelay)
{
  auto node = make();
  node->real_plan();
  node->root_transform();
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  for (int64_t tick = 0; tick < 40; ++tick) {
    const int64_t now = 1000000000 + tick * 25000000;
    node->time_ns(now);
    auto model = std::make_shared<TargetState>();
    model->header.stamp = rclcpp::Time(now, RCL_ROS_TIME);
    model->detection_stamp = model->header.stamp;
    model->confidence = 0.5;
    model->valid = true;
    model->center.x = 3.0;
    model->radius = {0.2, 0.2};
    node->on_target_state(model);
    node->on_publish_tick();
    ASSERT_EQ(points.size(), static_cast<size_t>(tick + 1));
    EXPECT_TRUE(points.back().fire) << tick;
    EXPECT_EQ(points.back().delay_ms, 40) << tick;
  }
}

TEST_F(NodeTest, FireDecisionKeepsConfidenceSolutionAndWireBounds)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->valid = true;
  model->confidence = 0.49;
  node->on_target_state(model);
  EXPECT_FALSE(node->fire_decision(0.0).first);
  model = std::make_shared<TargetState>(*model);
  model->confidence = 0.5;
  node->on_target_state(model);
  EXPECT_FALSE(node->fire_decision(std::nullopt).first);
  EXPECT_EQ(node->fire_decision(0.0), std::make_pair(true, 40));
  EXPECT_EQ(node->fire_decision(0.012), std::make_pair(true, 52));
  EXPECT_EQ(node->fire_decision(70.0), std::make_pair(true, 65535));
}

TEST_F(NodeTest, RealPlannerKeepsUnconvergedAndMissingTfFramesNonFiring)
{
  auto node = make();
  node->real_plan();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  model->detection_stamp = model->header.stamp;
  model->confidence = 1.0;
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_target_state(model);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_FALSE(points.back().fire);
  EXPECT_FALSE(points.back().patrol);
  model = std::make_shared<TargetState>(*model);
  model->valid = true;
  node->on_target_state(model);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 2u);
  EXPECT_FALSE(points.back().fire);
  EXPECT_TRUE(points.back().patrol);
}

TEST_F(NodeTest, BriefSameTrackGapsPreserveDwellAndBlockFire)
{
  for (const std::string reason : {"invalid", "confidence", "stale", "missing"}) {
    for (int frames : {3, 5}) {
      SCOPED_TRACE(reason + std::to_string(frames));
      auto node = make();
      std::vector<CVTarget> points;
      std::vector<Header> ticks;
      node->capture(&points, &ticks);
      // 30 fps detections, including a mid-dwell dropout.
      for (int frame = 0; frame <= 30; ++frame) {
        const int64_t now = 1000000000 + frame * 1000000000LL / 30;
        node->time_ns(now);
        const bool gap = frame >= 15 && frame < 15 + frames;
        if (!gap || reason != "missing") {
          auto model = std::make_shared<TargetState>();
          model->valid = !(gap && reason == "invalid");
          model->confidence = gap && reason == "confidence" ? 0.1 : 0.8;
          model->robot_track_id = 7;
          model->velocity.x = 0.2;
          model->header.stamp = rclcpp::Time(now, RCL_ROS_TIME);
          model->detection_stamp = rclcpp::Time(
            gap && reason == "stale" ? now - 200000000 : now, RCL_ROS_TIME);
          node->on_target_state(model);
        }
        node->on_publish_tick();
        EXPECT_EQ(points.back().fire, frame == 30);
      }
    }
  }
}

TEST_F(NodeTest, EarnedLockHoldsFireDuringBriefIneligibleGap)
{
  for (const std::string reason : {"invalid", "confidence", "stale"}) {
    SCOPED_TRACE(reason);
    auto node = make();
    for (int frame = 0; frame <= 38; ++frame) {
      const int64_t now = 1000000000 + frame * 1000000000LL / 30;
      node->time_ns(now);
      const bool gap = frame >= 32 && frame < 37;
      auto model = std::make_shared<TargetState>();
      model->valid = !(gap && reason == "invalid");
      model->confidence = gap && reason == "confidence" ? 0.1 : 0.8;
      model->robot_track_id = 7;
      model->velocity.x = 0.2;
      model->header.stamp = rclcpp::Time(now, RCL_ROS_TIME);
      model->detection_stamp = rclcpp::Time(
        gap && reason == "stale" ? now - 200000000 : now, RCL_ROS_TIME);
      node->on_target_state(model);
      EXPECT_EQ(node->fire_decision(0.0).first, frame >= 30 && !gap);
    }
  }
}

TEST_F(NodeTest, LongGapsIdentityChangesAndTargetLossRestartDwell)
{
  for (const std::string reason : {"invalid", "confidence", "identity", "stale", "loss"}) {
    SCOPED_TRACE(reason);
    auto node = make();
    auto send = [&](int64_t now, bool gap) {
        node->time_ns(now);
        auto model = std::make_shared<TargetState>();
        model->valid = !(gap && reason == "invalid");
        model->confidence = gap && reason == "confidence" ? 0.1 : 0.8;
        model->robot_track_id = gap && reason == "identity" ? 8 : 7;
        model->velocity.x = 0.2;
        model->header.stamp = rclcpp::Time(now, RCL_ROS_TIME);
        model->detection_stamp = rclcpp::Time(
          gap && reason == "stale" ? now - 200000000 : now, RCL_ROS_TIME);
        node->on_target_state(model);
      };
    send(1000000000, false);
    send(1500000000, false);
    if (reason == "loss") {
      node->time_ns(2100000000);
      node->check_timeout();
    } else {
      send(1600000000, true);
      // No publish tick: callback observations must also enforce the grace.
      send(2000000000, true);
    }
    send(2200000000, false);
    EXPECT_FALSE(node->fire_decision(0.0).first);
    for (int tick = 1; tick <= 10; ++tick) {
      send(2200000000 + tick * 100000000, false);
      EXPECT_EQ(node->fire_decision(0.0).first, tick == 10);
    }
  }
}

TEST_F(NodeTest, IdentitySwitchBetweenTicksAndTimeoutBeforeRecoveryResetLock)
{
  for (bool timeout : {false, true}) {
    auto node = make();
    auto send = [&](int64_t now, uint32_t id) {
        node->time_ns(now);
        auto model = std::make_shared<TargetState>();
        model->valid = true;
        model->confidence = 0.8;
        model->robot_track_id = id;
        model->velocity.x = 0.2;
        model->header.stamp = rclcpp::Time(now, RCL_ROS_TIME);
        model->detection_stamp = model->header.stamp;
        node->on_target_state(model);
      };
    send(1000000000, 7);
    send(1400000000, 7);
    if (!timeout) {send(1500000000, 8);}
    const int64_t recovery = timeout ? 2000000000 : 1510000000;
    for (int tick = 0; tick <= 10; ++tick) {
      send(recovery + tick * 100000000, 7);
      EXPECT_EQ(node->fire_decision(0.0).first, tick == 10);
    }
  }
}

TEST_F(NodeTest, ZeroLockAllowsMovingTargetImmediatelyButStillChecksFreshness)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("fire_lock_s", 0.0);
  auto node = std::make_shared<Probe>(options);
  auto model = std::make_shared<TargetState>();
  model->valid = true;
  model->confidence = 0.8;
  model->velocity.x = 1.0;
  node->on_target_state(model);
  EXPECT_TRUE(node->fire_decision(0.0).first);
  node->time_ns(151000000);
  EXPECT_FALSE(node->fire_decision(0.0).first);
}

TEST_F(NodeTest, PlannerShotWindowsDoNotInterruptTheLock)
{
  auto node = make();
  node->non_firing_target();
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  for (int64_t tick = 0; tick <= 10; ++tick) {
    const int64_t now = 1000000000 + tick * 100000000;
    node->time_ns(now);
    auto model = std::make_shared<TargetState>();
    model->valid = true;
    model->confidence = 0.8;
    model->velocity.x = 0.2;
    model->header.stamp = rclcpp::Time(now, RCL_ROS_TIME);
    model->detection_stamp = model->header.stamp;
    node->on_target_state(model);
    if (tick == 10) {node->firing_target();}
    node->on_publish_tick();
    EXPECT_EQ(points.back().fire, tick == 10);
  }
}

TEST_F(NodeTest, RejectsInvalidLockAndStationarySpeedParameters)
{
  for (const auto * name : {"fire_lock_s", "fire_lock_grace_s", "fire_stationary_speed_mps"}) {
    for (double value : {-0.1, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    {
      rclcpp::NodeOptions options;
      options.append_parameter_override(name, value);
      EXPECT_THROW(std::make_shared<Probe>(options), std::invalid_argument);
    }
  }
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

TEST_F(NodeTest, SubscribesToTargetStatePoseAndReferee)
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
    "dji_serial_bridge/msg/RobotPose",
    "dji_serial_bridge/msg/TargetState"}));
}

TEST_F(NodeTest, PanelHoldSurvivesUpdatesButResetsOnNewTrackInvalidOrSpin)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  model->detection_stamp = model->header.stamp;
  model->robot_track_id = 1;
  model->valid = true;
  node->on_target_state(model);
  node->hold(1.0);
  node->on_target_state(std::make_shared<TargetState>(*model));
  EXPECT_TRUE(node->held());
  model->robot_track_id = 2;
  node->on_target_state(model);
  EXPECT_FALSE(node->held());
  node->hold(1.0);
  model->valid = false;
  node->on_target_state(model);
  EXPECT_FALSE(node->held());
  node->hold(1.0);
  model->valid = true;
  model->yaw_rate = 4.0;
  node->on_target_state(model);
  EXPECT_FALSE(node->held());
}

TEST_F(NodeTest, PanelHoldResetsOnTimeoutAndLateSameTrackReplacement)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  model->detection_stamp = model->header.stamp;
  model->valid = true;
  node->on_target_state(model);
  node->hold(1.0);
  node->time_ns(1000000000);
  auto fresh = std::make_shared<TargetState>(*model);
  fresh->header.stamp = rclcpp::Time(int64_t{1000000000}, RCL_ROS_TIME);
  fresh->detection_stamp = fresh->header.stamp;
  node->on_target_state(fresh);
  EXPECT_FALSE(node->held());
  node->hold(1.0);
  node->time_ns(2000000000);
  node->base_plan();
  EXPECT_FALSE(node->held());
  node->hold(1.0);
  node->check_timeout();
  EXPECT_FALSE(node->held());
  EXPECT_FALSE(node->active());
}

TEST_F(NodeTest, RejectsInvalidPanelSwitchMargins)
{
  for (double margin : {-0.1, tp::QUARTER_TURN, std::numeric_limits<double>::infinity()}) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("panel_switch_margin_rad", margin);
    EXPECT_THROW(std::make_shared<Probe>(options), std::invalid_argument);
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

TEST_F(NodeTest, HitPermissionChangesImmediatelyWithAnyTargetEstimate)
{
  auto node = std::make_shared<Probe>(rclcpp::NodeOptions());
  node->gun_transform();
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(points.back().patrol);
  EXPECT_FALSE(points.back().fire);
  EXPECT_TRUE(points.back().turn_to_hit);
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  model->detection_stamp = model->header.stamp;
  model->valid = true;
  node->on_target_state(model);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 2u);
  EXPECT_FALSE(points.back().patrol);
  EXPECT_FALSE(points.back().turn_to_hit);
  // No new detection: keep the cached estimate during brief occlusion.
  node->time_ns(200000000);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 3u);
  EXPECT_FALSE(points.back().turn_to_hit);
  model = std::make_shared<TargetState>(*model);
  model->valid = false;
  node->on_target_state(model);
  node->non_firing_target();
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 4u);
  EXPECT_FALSE(points.back().patrol);
  EXPECT_FALSE(points.back().fire);
  EXPECT_FALSE(points.back().turn_to_hit);
  node->time_ns(700000000);
  node->on_publish_tick();
  node->time_ns(950000000);
  node->on_publish_tick();
  ASSERT_TRUE(points.back().patrol);
  EXPECT_TRUE(points.back().turn_to_hit);
  model = std::make_shared<TargetState>(*model);
  model->header.stamp = rclcpp::Time(int64_t{950000000}, RCL_ROS_TIME);
  model->detection_stamp = model->header.stamp;
  node->on_target_state(model);
  node->on_publish_tick();
  EXPECT_FALSE(points.back().turn_to_hit);
}

TEST_F(NodeTest, EveryNonTargetFramePermitsHitTurnWhenAimCannotBeComputed)
{
  auto node = std::make_shared<Probe>(rclcpp::NodeOptions());
  node->gun_transform();
  node->fail_aim();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  model->detection_stamp = model->header.stamp;
  node->on_target_state(model);
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(points.back().patrol);
  EXPECT_TRUE(points.back().turn_to_hit);
}

TEST_F(NodeTest, FireAgeBoundaryCoastsUntilTargetTimeout)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  model->detection_stamp = model->header.stamp;
  model->confidence = 0.5;
  model->valid = true;
  node->on_target_state(model);
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->time_ns(250000000);
  node->on_publish_tick();
  EXPECT_TRUE(points.back().fire);
  node->time_ns(250000001);
  node->on_publish_tick();
  EXPECT_FALSE(points.back().fire);
  EXPECT_FALSE(points.back().patrol);
  EXPECT_FALSE(points.back().turn_to_hit);
  node->time_ns(600000001);
  node->on_publish_tick();
  EXPECT_TRUE(points.back().patrol);
  EXPECT_TRUE(points.back().turn_to_hit);
  EXPECT_FALSE(points.back().fire);
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

TEST_F(NodeTest, PublicationTimeCannotRefreshAnOldDetection)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{300000000}, RCL_ROS_TIME);
  model->detection_stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  node->time_ns(325000000);
  node->on_target_state(model);
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  EXPECT_FALSE(points.back().fire);
  EXPECT_FALSE(points.back().patrol);
  node->time_ns(600000001);
  node->on_publish_tick();
  EXPECT_TRUE(points.back().patrol);
  EXPECT_TRUE(points.back().turn_to_hit);
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
