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

  void gun_transform()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = muzzle_frame_;
    tf.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(tf, "test", true);
  }
  void set_active(bool active) {target_active_ = active;}
  void non_firing_target() {fire_target_ = false;}
  void fail_aim() {fail_aim_ = true;}
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
    if (fail_aim_) {return std::nullopt;}
    return Aim{tp::Vector3d(3.0, 1.0, 0.3),
      fire_target_ ? std::optional<double>(0.0) : std::nullopt};
  }
  std::pair<bool, int> fire_decision(std::optional<double> delay, const rclcpp::Time &) override
  {
    return {delay.has_value(), 0};
  }
  rclcpp::Time clock_now() const override {return rclcpp::Time(now_ns_, RCL_ROS_TIME);}
  int64_t now_ns_ = 123000000;
  bool fire_target_ = true;
  bool fail_aim_ = false;
};

class NodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<Probe> make()
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("patrol_enabled", false);
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
  EXPECT_TRUE(points.empty());
  ASSERT_EQ(ticks.size(), 1u);
  EXPECT_EQ(ticks[0].frame_id, "");
  EXPECT_EQ(rclcpp::Time(ticks[0].stamp).nanoseconds(), 123000000);
}

TEST_F(NodeTest, TargetTickIdentifiesTheOutputFrameAndStamp)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  model->header.frame_id = "odom";
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

TEST_F(NodeTest, ModelAckIsSentAfterConsumptionWithTheInputStamp)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
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

TEST_F(NodeTest, SubscribesToTargetStateAndRobotPoseOnly)
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
    "dji_serial_bridge/msg/RobotPose",
    "dji_serial_bridge/msg/TargetState"}));
}

TEST_F(NodeTest, PanelHoldSurvivesUpdatesButResetsOnNewTrackInvalidOrSpin)
{
  auto node = make();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
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
  model->valid = true;
  node->on_target_state(model);
  node->hold(1.0);
  node->time_ns(1000000000);
  auto fresh = std::make_shared<TargetState>(*model);
  fresh->header.stamp = rclcpp::Time(int64_t{1000000000}, RCL_ROS_TIME);
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
  node->on_target_state(model);
  node->on_publish_tick();
  EXPECT_FALSE(points.back().turn_to_hit);
}

TEST_F(NodeTest, HeldEstimateBlocksHitTurnEvenWhenAimCannotBeComputed)
{
  auto node = std::make_shared<Probe>(rclcpp::NodeOptions());
  node->gun_transform();
  node->fail_aim();
  auto model = std::make_shared<TargetState>();
  model->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  node->on_target_state(model);
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(points.back().patrol);
  EXPECT_FALSE(points.back().turn_to_hit);
}
