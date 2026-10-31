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
  : tp::PointToCvTarget(options) {}

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
  void navigation_transform(double team_start)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "nav_odom";
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = team_start;
    tf.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(tf, "test", true);
  }
  tp::Vector3d idle_point() {return idle_point_now(clock_now());}
  void set_active(bool active) {target_active_ = active;}
  void non_firing_target() {fire_target_ = false;}
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
    EXPECT_EQ(points.back().delay_ms, 25) << tick;
  }
}

TEST_F(NodeTest, FireDecisionKeepsConfidenceSolutionAndWireBounds)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->confidence = 0.49;
  node->on_target_state(model);
  EXPECT_FALSE(node->fire_decision(0.0).first);
  model = std::make_shared<TargetState>(*model);
  model->confidence = 0.5;
  node->on_target_state(model);
  EXPECT_FALSE(node->fire_decision(std::nullopt).first);
  EXPECT_EQ(node->fire_decision(0.0), std::make_pair(true, 25));
  EXPECT_EQ(node->fire_decision(0.012), std::make_pair(true, 37));
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
    node->navigation_transform(blue ? 4.625 : -4.625);
    dji_serial_bridge::msg::RefSysStatus referee;
    referee.robot_id = blue ? 107 : 7;
    referee.is_on_blue_team = blue;
    referee.delta_angle_got_hit_in = 123.0F;
    node->on_ref_sys(referee);
    node->idle_point();
    node->time_ns(20000000000LL);
    const auto point = node->idle_point();
    EXPECT_NEAR(point.x(), blue ? -9.25 : 9.25, 1e-6);
    EXPECT_NEAR(point.y(), 0.0, 1e-6);
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
