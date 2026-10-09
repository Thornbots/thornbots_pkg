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

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rcl/time.h"
#include "thornbots_pkg/pose_translator_node.hpp"

using dji_serial_bridge::msg::RobotPose;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::JointState;

namespace
{

class Probe : public thornbots_pkg::PoseTranslator
{
public:
  explicit Probe(const rclcpp::NodeOptions & options)
  : PoseTranslator(options) {}

  using PoseTranslator::pose_callback;
  std::optional<Odometry> odom;
  std::optional<JointState> joints;

protected:
  void publish_outputs(const Odometry & output_odom, const JointState & output_joints) override
  {
    odom = output_odom;
    joints = output_joints;
  }
};

class PoseTranslatorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("odom_frame", "test_odom");
    options.append_parameter_override("base_frame", "test_root");
    node = std::make_shared<Probe>(options);
    auto clock = node->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 100987654321LL), RCL_RET_OK);
    ASSERT_EQ(node->now().nanoseconds(), 100987654321LL);
  }

  void expect_source_stamp(const RobotPose & pose)
  {
    ASSERT_NO_THROW(node->pose_callback(pose));
    ASSERT_TRUE(node->odom.has_value());
    ASSERT_TRUE(node->joints.has_value());
    EXPECT_EQ(node->odom->header.stamp, pose.header.stamp);
    EXPECT_EQ(node->joints->header.stamp, pose.header.stamp);
  }

  std::shared_ptr<Probe> node;
};

}  // namespace

TEST_F(PoseTranslatorTest, PreservesOldCaptureTimeWithANewerClock)
{
  RobotPose pose;
  pose.header.stamp.sec = 12;
  pose.header.stamp.nanosec = 345678901;
  expect_source_stamp(pose);
}

TEST_F(PoseTranslatorTest, PreservesSimulationEpochZero)
{
  RobotPose pose;
  pose.header.stamp.sec = 0;
  pose.header.stamp.nanosec = 0;
  expect_source_stamp(pose);
}

TEST_F(PoseTranslatorTest, PreservesNegativeSecondsWithoutConstructingRosTime)
{
  RobotPose pose;
  // builtin_interfaces/Time represents -1.7 seconds as (-2, 300000000).
  pose.header.stamp.sec = -2;
  pose.header.stamp.nanosec = 300000000;
  expect_source_stamp(pose);
}

TEST_F(PoseTranslatorTest, KeepsPoseVelocityJointsFramesAndCovariance)
{
  RobotPose pose;
  pose.x = 1.25;
  pose.y = -2.5;
  pose.vel_x = -0.75;
  pose.vel_y = 0.5;
  pose.chassis_yaw = 0.25;
  pose.head_yaw = -0.5;
  pose.head_pitch = 0.125;
  node->pose_callback(pose);
  ASSERT_TRUE(node->odom.has_value());
  ASSERT_TRUE(node->joints.has_value());
  EXPECT_EQ(node->odom->header.frame_id, "test_odom");
  EXPECT_EQ(node->odom->child_frame_id, "test_root");
  EXPECT_DOUBLE_EQ(node->odom->pose.pose.position.x, pose.x);
  EXPECT_DOUBLE_EQ(node->odom->pose.pose.position.y, pose.y);
  EXPECT_DOUBLE_EQ(node->odom->pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(node->odom->pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(node->odom->pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(node->odom->pose.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(node->odom->twist.twist.linear.x, pose.vel_x);
  EXPECT_DOUBLE_EQ(node->odom->twist.twist.linear.y, pose.vel_y);
  for (std::size_t i = 0; i < 36; ++i) {
    const double variance = (i == 0 || i == 7) ? 0.0001 : 0.0;
    EXPECT_DOUBLE_EQ(node->odom->pose.covariance[i], variance);
    EXPECT_DOUBLE_EQ(node->odom->twist.covariance[i], variance);
  }
  EXPECT_EQ(node->joints->name, (std::vector<std::string>{"chassis_yaw", "headlink", "headpitch"}));
  EXPECT_EQ(node->joints->position, (std::vector<double>{0.25, -0.5, 0.125}));
}
