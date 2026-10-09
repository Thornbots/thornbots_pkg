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

  void set_active(bool active) {target_active_ = active;}
  bool active() const {return target_active_;}
  TargetState::ConstSharedPtr latest() const {return latest_state_;}
  void on_target_state_hook(std::function<void(const Header &)> ack) {send_ack_ = ack;}
  void capture(std::vector<CVTarget> * points, std::vector<Header> * ticks)
  {
    send_target_ = [points](const CVTarget & m) {points->push_back(m);};
    send_tick_ = [ticks](const Header & h) {ticks->push_back(h);};
  }

protected:
  std::optional<Aim> compute_aim_point() override {return Aim{tp::Vector3d(3.0, 1.0, 0.3), 0.0};}
  std::pair<bool, int> fire_decision(std::optional<double>, const rclcpp::Time &) override
  {
    return {true, 0};
  }
  rclcpp::Time clock_now() const override {return rclcpp::Time(int64_t{123000000}, RCL_ROS_TIME);}
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
  std::vector<CVTarget> points;
  std::vector<Header> ticks;
  node->capture(&points, &ticks);
  node->set_active(true);
  node->on_publish_tick();
  ASSERT_EQ(points.size(), 1u);
  ASSERT_EQ(ticks.size(), 1u);
  EXPECT_EQ(ticks[0], points[0].header);
  EXPECT_EQ(ticks[0].frame_id, "odom");
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
