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

#include <cstdint>
#include <memory>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
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
  using tp::PointToCvTarget::stamped_transform;
  using tp::PointToCvTarget::gun_pose;
  uint64_t fallbacks() const {return latest_tf_fallbacks_;}
  void dynamic_transform(int64_t stamp_ns, double x, const std::string & child = "root")
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    tf.child_frame_id = child;
    tf.transform.translation.x = x;
    tf.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_buffer_->setTransform(tf, "test", false));
  }
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
  bool active() const {return target_active_;}
  void hold(double yaw) {held_panel_yaw_ = yaw;}
  bool held() const {return held_panel_yaw_.has_value();}
  void time_ns(int64_t value) {now_ns_ = value;}
  using tp::PointToCvTarget::check_timeout;
  bool base_plan() {return tp::PointToCvTarget::compute_aim_point().has_value();}
  TargetState::ConstSharedPtr latest() const {return latest_state_;}
  size_t history_size() const {return state_history_.size();}
  void on_target_state_hook(std::function<void(const Header &)> ack) {send_ack_ = ack;}
  void capture(std::vector<CVTarget> * points, std::vector<Header> * ticks)
  {
    send_target_ = [points](const CVTarget & m) {points->push_back(m);};
    send_tick_ = [ticks](const Header & h) {ticks->push_back(h);};
  }

protected:
  std::optional<Aim> compute_aim_point() override
  {
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

TargetState::SharedPtr target_state(int64_t stamp_ns, uint32_t track_id,
                                    bool valid = true)
{
  auto state = std::make_shared<TargetState>();
  state->header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
  state->header.frame_id = "odom";
  state->robot_track_id = track_id;
  state->valid = valid;
  state->center.x = 3.0;
  state->center.z = 0.4;
  state->radius = {0.3f, 0.3f};
  return state;
}

}  // namespace

TEST_F(NodeTest, StrictTfWithholdsFutureAndZeroStampsWithoutLatestFallback)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("allow_latest_tf", false);
  auto node = std::make_shared<Probe>(options);
  node->dynamic_transform(100000000, 1.0);
  EXPECT_FALSE(node->stamped_transform("root", rclcpp::Time(int64_t{200000000}, RCL_ROS_TIME)));
  EXPECT_FALSE(node->stamped_transform("root", rclcpp::Time(int64_t{0}, RCL_ROS_TIME)));
  EXPECT_EQ(node->fallbacks(), 0u);
}

TEST_F(NodeTest, StrictTfRecoversAtTheOriginalStampAfterDelayedDelivery)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("allow_latest_tf", false);
  auto node = std::make_shared<Probe>(options);
  node->dynamic_transform(100000000, 1.0);
  const rclcpp::Time stamp(int64_t{150000000}, RCL_ROS_TIME);
  EXPECT_FALSE(node->stamped_transform("root", stamp));
  node->dynamic_transform(200000000, 3.0);
  const auto tf = node->stamped_transform("root", stamp);
  ASSERT_TRUE(tf);
  EXPECT_DOUBLE_EQ(tf->transform.translation.x, 2.0);
  EXPECT_EQ(rclcpp::Time(tf->header.stamp).nanoseconds(), stamp.nanoseconds());
  EXPECT_EQ(node->fallbacks(), 0u);
}

TEST_F(NodeTest, HardwareLatestFallbackIsCountedForBothAimAndGun)
{
  auto node = make();
  node->dynamic_transform(100000000, 1.0);
  node->dynamic_transform(100000000, 2.0, "muzzle");
  const rclcpp::Time stamp(int64_t{200000000}, RCL_ROS_TIME);
  ASSERT_TRUE(node->stamped_transform("root", stamp));
  ASSERT_TRUE(node->gun_pose(stamp));
  EXPECT_EQ(node->fallbacks(), 2u);
}

TEST_F(NodeTest, StrictAimRetainsResolvableStateUntilNewerStampedTfArrives)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("allow_latest_tf", false);
  auto node = std::make_shared<Probe>(options);
  node->time_ns(175000000);
  node->dynamic_transform(100000000, 0.0);
  auto state = std::make_shared<TargetState>();
  state->header.stamp = rclcpp::Time(int64_t{100000000}, RCL_ROS_TIME);
  state->valid = true;
  state->center.x = 3.0;
  state->center.z = 0.4;
  state->radius = {0.3f, 0.3f};
  node->on_target_state(state);
  EXPECT_TRUE(node->base_plan());
  auto newer = std::make_shared<TargetState>(*state);
  newer->header.stamp = rclcpp::Time(int64_t{150000000}, RCL_ROS_TIME);
  node->on_target_state(newer);
  EXPECT_TRUE(node->base_plan());
  EXPECT_EQ(node->fallbacks(), 0u);
  newer = std::make_shared<TargetState>(*newer);
  newer->robot_track_id = 1;
  node->on_target_state(newer);
  EXPECT_FALSE(node->base_plan()) << "handoff must discard the previous track's history";
}

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

TEST_F(NodeTest, SubscribesToTargetStateRobotPoseAndRefSysOnly)
{
  // The CV interface (README.md): Part 1 aims from TargetState alone, so a
  // truth publisher can replace the whole of Part 2. RefSysStatus only turns
  // the patrol toward a hit. (/tf and parameter events are the TransformListener's and rclcpp's.)
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
    "dji_serial_bridge/msg/RefSysStatus", "dji_serial_bridge/msg/RobotPose",
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

TEST_F(NodeTest, SweepAndHitTurnPointsArePatrolButTargetsAreNot)
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
  dji_serial_bridge::msg::RefSysStatus hit;
  hit.delta_angle_got_hit_in = 1.0;
  node->on_ref_sys(hit);
  node->time_ns(148000000);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 2u);
  EXPECT_TRUE(points.back().patrol);
  EXPECT_FALSE(points.back().fire);
  EXPECT_NEAR(points.back().x, 3.0 * std::cos(1.0), 1e-5);
  node->set_active(true);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 3u);
  EXPECT_FALSE(points.back().patrol);
  node->non_firing_target();
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 4u);
  EXPECT_FALSE(points.back().patrol);
  EXPECT_FALSE(points.back().fire);
}

TEST_F(NodeTest, HoldsReceivedStateUntilCaptureStampIsFourteenMillisecondsOld)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("patrol_enabled", false);
  options.append_parameter_override("state_hold_s", 0.014);
  auto node = std::make_shared<Probe>(options);
  std::vector<Header> acknowledged;
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_target_state_hook([&](const Header & header) {
    EXPECT_FALSE(node->latest());
    acknowledged.push_back(header);
  });

  const auto state = target_state(100000000, 7);
  node->time_ns(110000000);
  node->on_target_state(state);
  ASSERT_EQ(acknowledged.size(), 1u);
  EXPECT_EQ(acknowledged.front(), state->header);
  EXPECT_FALSE(node->latest());

  node->time_ns(113999999);
  node->on_publish_tick();
  EXPECT_FALSE(node->latest());
  EXPECT_TRUE(points.empty());

  node->time_ns(114000000);
  node->on_publish_tick();
  EXPECT_EQ(node->latest(), state);
  EXPECT_EQ(node->history_size(), 1u);
  ASSERT_EQ(points.size(), 1u);
  ASSERT_EQ(ticks.size(), 2u);
  EXPECT_EQ(points.front().header.frame_id, "odom");
  EXPECT_EQ(acknowledged.size(), 1u);

  node->time_ns(115000000);
  node->on_publish_tick();
  EXPECT_EQ(node->latest(), state);
  EXPECT_EQ(node->history_size(), 1u);
  EXPECT_EQ(acknowledged.size(), 1u);
}

TEST_F(NodeTest, EarlyHandoffAndInvalidStateWaitWithoutClearingHeldPanel)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("patrol_enabled", false);
  options.append_parameter_override("state_hold_s", 0.014);
  auto node = std::make_shared<Probe>(options);
  std::vector<Header> acknowledged;
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->on_target_state_hook(
    [&](const Header & header) {acknowledged.push_back(header);});

  const auto current = target_state(100000000, 1);
  node->time_ns(114000000);
  node->on_target_state(current);
  node->on_publish_tick();
  ASSERT_EQ(node->latest(), current);
  node->hold(0.5);

  const auto handoff = target_state(120000000, 2);
  const auto invalid = target_state(121000000, 2, false);
  node->time_ns(121000000);
  node->on_target_state(handoff);
  node->time_ns(122000000);
  node->on_target_state(invalid);
  ASSERT_EQ(acknowledged.size(), 3u);
  EXPECT_EQ(node->latest(), current);
  EXPECT_TRUE(node->held());

  node->time_ns(133999999);
  node->on_publish_tick();
  EXPECT_EQ(node->latest(), current);
  EXPECT_TRUE(node->held());

  node->time_ns(134000000);
  node->on_publish_tick();
  EXPECT_EQ(node->latest(), handoff);
  EXPECT_FALSE(node->held());

  node->hold(0.25);
  node->time_ns(134999999);
  node->on_publish_tick();
  EXPECT_EQ(node->latest(), handoff);
  EXPECT_TRUE(node->held());

  node->time_ns(135000000);
  node->on_publish_tick();
  EXPECT_EQ(node->latest(), invalid);
  EXPECT_FALSE(node->held());
  EXPECT_EQ(acknowledged.size(), 3u);
}

TEST_F(NodeTest, CaptureEligibilityIsIndependentOfStateArrivalClock)
{
  auto options = [] {
    rclcpp::NodeOptions result;
    result.append_parameter_override("patrol_enabled", false);
    result.append_parameter_override("state_hold_s", 0.014);
    return result;
  };
  auto early = std::make_shared<Probe>(options());
  auto late = std::make_shared<Probe>(options());
  std::vector<CVTarget> early_points, late_points;
  std::vector<Header> early_ticks, late_ticks;
  early->capture(&early_points, &early_ticks);
  late->capture(&late_points, &late_ticks);
  const auto state = target_state(100000000, 42);

  early->time_ns(101000000);
  early->on_target_state(state);
  late->time_ns(113000000);
  late->on_target_state(state);

  early->time_ns(113999999);
  late->time_ns(113999999);
  early->on_publish_tick();
  late->on_publish_tick();
  EXPECT_FALSE(early->latest());
  EXPECT_FALSE(late->latest());

  early->time_ns(114000000);
  late->time_ns(114000000);
  early->on_publish_tick();
  late->on_publish_tick();
  ASSERT_TRUE(early->latest());
  ASSERT_TRUE(late->latest());
  EXPECT_EQ(early->latest()->robot_track_id, late->latest()->robot_track_id);
  EXPECT_EQ(early->latest()->robot_track_id, 42u);
}

TEST_F(NodeTest, RejectsInvalidStateHoldDurations)
{
  for (double hold_s : {-0.001, std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::max()})
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("state_hold_s", hold_s);
    EXPECT_THROW(std::make_shared<Probe>(options), std::invalid_argument);
  }
}
