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
#include <memory>
#include <stdexcept>
#include <vector>

#include "thornbots_pkg/target_manager_node.hpp"

using Message = dji_serial_bridge::msg::TargetState;
using Header = std_msgs::msg::Header;
namespace
{
class Probe : public thornbots_pkg::TargetManagerNode
{
public:
  using TargetManagerNode::consume;
  using TargetManagerNode::tick;
  std::vector<Message> sent;
  std::vector<Header> acks, suppressed;
  int64_t time_ns = 1100000000;
  Probe()
  {
    send_ = [this](const Message & m) {sent.push_back(m);};
    ack_ = [this](const Header & h) {acks.push_back(h);};
    none_ = [this](const Header & h) {suppressed.push_back(h);};
  }
  rclcpp::Time decision_time() const override {return rclcpp::Time(time_ns, RCL_ROS_TIME);}
};
class ManagerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  std::shared_ptr<Message> model()
  {
    auto m = std::make_shared<Message>();
    m->header.frame_id = "odom";
    m->header.stamp.sec = 1; m->header.stamp.nanosec = 100000000;
    m->detection_stamp.sec = 1; m->detection_stamp.nanosec = 50000000;
    m->input_stamp = m->detection_stamp;
    m->robot_track_id = 7; m->confidence = 0.8F; m->valid = true;
    m->radius = {0.2F, 0.3F}; m->update_count = 1;
    return m;
  }
};
}  // namespace

TEST_F(ManagerNodeTest, ForwardPreservesBothStampsCovarianceAndFirstUpdate)
{
  Probe node; auto m = model(); m->variance[0] = 12.0F;
  node.consume(m);
  ASSERT_EQ(node.sent.size(), 1u);
  EXPECT_EQ(node.sent[0].header, m->header);
  EXPECT_EQ(node.sent[0].detection_stamp, m->detection_stamp);
  EXPECT_EQ(node.sent[0].variance, m->variance);
  EXPECT_EQ(node.sent[0].confidence, m->confidence);
  EXPECT_TRUE(node.sent[0].valid);
  EXPECT_EQ(node.acks[0].stamp, m->input_stamp);
  EXPECT_EQ(node.acks[0].frame_id, m->header.frame_id);
}

TEST_F(ManagerNodeTest, EverySuppressedEstimateIsAcknowledgedWithoutRestamping)
{
  Probe node; auto m = model(); node.consume(m);
  for (int i = 0; i < 5; ++i) {
    node.time_ns += 10000000;
    m->input_stamp.nanosec += 10000000;
    node.consume(m);
  }
  EXPECT_EQ(node.sent.size(), 1u);
  EXPECT_EQ(node.acks.size(), 6u);
  ASSERT_EQ(node.suppressed.size(), 5u);
  EXPECT_EQ(node.suppressed.back().stamp, m->input_stamp);
  EXPECT_EQ(node.acks.back().stamp, m->input_stamp);
}

TEST_F(ManagerNodeTest, RefreshForwardsNewEvidenceAtItsCarriedTrueTime)
{
  Probe node; node.consume(model()); auto next = model();
  node.time_ns = 1310000000;
  next->header.stamp.nanosec = 300000000;
  next->detection_stamp.nanosec = 290000000;
  next->variance[2] = 45.0F;
  node.consume(next);
  ASSERT_EQ(node.sent.size(), 2u);
  EXPECT_EQ(node.sent.back().header, next->header);
  EXPECT_EQ(node.sent.back().detection_stamp, next->detection_stamp);
  EXPECT_EQ(node.sent.back().variance, next->variance);
  EXPECT_EQ(node.sent.back().confidence, next->confidence);
}

TEST_F(ManagerNodeTest, TimerClearIsFreshZeroModelWithReleasedIdentity)
{
  Probe node; node.consume(model());
  node.time_ns = 1310000000; node.tick();
  ASSERT_EQ(node.sent.size(), 2u);
  const auto & clear = node.sent.back();
  EXPECT_TRUE(clear.clear); EXPECT_FALSE(clear.valid);
  EXPECT_EQ(clear.header.stamp.nanosec, 310000000u);
  EXPECT_EQ(clear.detection_stamp.sec, 0);
  EXPECT_EQ(clear.robot_track_id, 7u);
  EXPECT_EQ(clear.center.x, 0); EXPECT_EQ(clear.confidence, 0);
  node.tick(); EXPECT_EQ(node.sent.size(), 2u);
}

TEST_F(ManagerNodeTest, FloatMessageConfidenceHonorsBothEqualityBoundaries)
{
  Probe node; auto m = model(); m->confidence = 0.60F;
  node.consume(m); ASSERT_EQ(node.sent.size(), 1u); EXPECT_TRUE(node.sent[0].valid);
  m->confidence = 0.40F; node.consume(m);
  ASSERT_EQ(node.sent.size(), 2u); EXPECT_TRUE(node.sent[1].clear);
}

TEST_F(ManagerNodeTest, ExplicitRetirementAndWeakResetClearImmediately)
{
  for (bool retired : {true, false}) {
    Probe node; node.consume(model()); auto status = model();
    status->selected_track = !retired; status->reset = !retired;
    status->confidence = 0.5F;
    node.consume(status);
    ASSERT_EQ(node.sent.size(), 2u);
    EXPECT_TRUE(node.sent.back().clear);
  }
}

TEST_F(ManagerNodeTest, WrongFrameFailsClosedAndClearAcknowledgesInput)
{
  Probe node; node.consume(model()); auto bad = model(); bad->header.frame_id = "map";
  node.consume(bad);
  EXPECT_TRUE(node.sent.back().clear);
  EXPECT_EQ(node.acks.back().stamp, bad->input_stamp);
  EXPECT_EQ(node.acks.back().frame_id, bad->header.frame_id);
}

TEST_F(ManagerNodeTest, ConfidentSameIdentityResetReleasesPlannerBeforeReassigning)
{
  Probe node; node.consume(model()); auto reset = model(); reset->reset = true;
  node.consume(reset);
  ASSERT_EQ(node.sent.size(), 3u);
  EXPECT_TRUE(node.sent[1].clear);
  EXPECT_TRUE(node.sent[2].valid); EXPECT_FALSE(node.sent[2].clear);
  node.tick(); EXPECT_EQ(node.sent.size(), 3u);
}

TEST_F(ManagerNodeTest, EpochZeroRequiresSimulatedClock)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("deterministic_epoch_zero", true);
  EXPECT_THROW(thornbots_pkg::TargetManagerNode{options}, std::invalid_argument);
}

TEST_F(ManagerNodeTest, EpochZeroAcknowledgesInitializationBeforeTheClockAdvances)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("deterministic_epoch_zero", true);
  auto manager = std::make_shared<thornbots_pkg::TargetManagerNode>(options);
  auto peer = std::make_shared<rclcpp::Node>("manager_epoch_test_peer");
  auto clock = peer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  int ready_count = 0;
  auto ready = peer->create_subscription<Header>(
    "/target_manager/epoch_zero_ready", rclcpp::QoS(1).reliable().transient_local(),
    [&](Header::ConstSharedPtr h) {
      EXPECT_EQ(h->stamp.sec, 0); EXPECT_EQ(h->stamp.nanosec, 0u); ++ready_count;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(manager); executor.add_node(peer);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  rosgraph_msgs::msg::Clock zero;
  while (ready_count == 0 && std::chrono::steady_clock::now() < deadline) {
    clock->publish(zero); executor.spin_some();
  }
  EXPECT_EQ(ready_count, 1);
}
