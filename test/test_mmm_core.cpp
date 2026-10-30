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

#include "thornbots_pkg/mmm_core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace mmm = thornbots_pkg::mmm;
namespace
{
class ProtocolTest : public ::testing::Test
{
protected:
  std::vector<std::pair<std::string, int>> events;
  std::vector<std::string> warnings;
  mmm::Protocol protocol{
    [this](int n) {events.emplace_back("send", n);},
    [this](int n) {events.emplace_back("dispatch", n);},
    [this](std::string text) {warnings.push_back(text);}};

  int count(const std::string & kind, int number) const
  {
    return static_cast<int>(std::count(events.begin(), events.end(),
      std::make_pair(kind, number)));
  }
  void timeout(double start)
  {
    for (int i = 1; i <= 10; ++i) {protocol.tick(start + i * 0.5);}
  }
};

TEST_F(ProtocolTest, MatchingEvenEchoCompletesOnlyPendingTransaction)
{
  for (int number : {2, 4, 6}) {
    SCOPED_TRACE(number);
    const double now = number * 10;
    ASSERT_TRUE(protocol.request(number, now));
    EXPECT_EQ(protocol.pending(), number);
    EXPECT_FALSE(protocol.request(number, now));
    EXPECT_FALSE(protocol.request(number == 2 ? 4 : 2, now));
    protocol.receive(number == 2 ? 4 : 2, now + 0.05);
    EXPECT_EQ(protocol.pending(), number);
    protocol.receive(number, now + 0.1);
    EXPECT_FALSE(protocol.pending());
    EXPECT_EQ(protocol.completed_even(), number);
    protocol.receive(number, now + 0.2);
    protocol.receive(number, now + 4);
    EXPECT_EQ(count("send", number), 2);
    EXPECT_EQ(count("dispatch", number), 0);
    EXPECT_FALSE(protocol.request(number, now + 5));
    EXPECT_TRUE(protocol.request(number, now + 5.11));
    protocol.receive(number, now + 5.2);
  }
}

TEST_F(ProtocolTest, RetriesTenAttemptsAndCooldownStartsAtTimeout)
{
  for (int number : {2, 4, 6}) {
    SCOPED_TRACE(number);
    const double start = number * 20;
    ASSERT_TRUE(protocol.request(number, start));
    protocol.tick(start + 0.49);
    EXPECT_EQ(count("send", number), 1);
    timeout(start);
    EXPECT_EQ(count("send", number), 10);
    EXPECT_FALSE(protocol.pending());
    EXPECT_EQ(protocol.failed(), number);
    EXPECT_FALSE(protocol.completed_even());
    EXPECT_NE(warnings.back().find("after 10 attempts"), std::string::npos);
    protocol.receive(number, start + 6);
    EXPECT_EQ(count("send", number), 10);
    EXPECT_FALSE(protocol.request(number, start + 9.999));
    EXPECT_TRUE(protocol.request(number, start + 10));
    EXPECT_FALSE(protocol.failed());
    timeout(start + 10);
  }
  EXPECT_EQ(warnings.size(), 6U);
}

TEST_F(ProtocolTest, OddEchoPrecedesDispatchAndSlidingGateIsPerNumber)
{
  protocol.receive(1, 0);
  protocol.receive(3, 0);
  const std::vector<std::pair<std::string, int>> initial{
    {"send", 1}, {"dispatch", 1}, {"send", 3}, {"dispatch", 3}};
  EXPECT_EQ(events, initial);
  for (double now : {0.1, 1.0, 4.0, 8.9}) {protocol.receive(1, now);}
  EXPECT_EQ(count("dispatch", 1), 1);
  EXPECT_EQ(count("send", 1), 5);
  protocol.receive(3, 5);
  EXPECT_EQ(count("dispatch", 3), 2);
  protocol.receive(1, 13.9);
  EXPECT_EQ(count("dispatch", 1), 2);
  EXPECT_EQ(events[events.size() - 2], std::make_pair(std::string("send"), 1));
  EXPECT_EQ(events.back(), std::make_pair(std::string("dispatch"), 1));
}

TEST_F(ProtocolTest, UnsolicitedAndUnknownFramesDoNothing)
{
  for (int number : {-1, 0, 2, 4, 5, 6, 7, 200, 255}) {
    protocol.receive(number, 0);
  }
  EXPECT_TRUE(events.empty());
  EXPECT_FALSE(protocol.pending());
  EXPECT_FALSE(protocol.completed_even());
  for (int number : {-1, 0, 1, 3, 5, 7, 255}) {
    EXPECT_FALSE(protocol.request(number, 0));
  }
  EXPECT_TRUE(events.empty());
}

TEST_F(ProtocolTest, HeartbeatIsIndependentAndHasNoEchoLeg)
{
  protocol.tick(0);
  protocol.receive(0, 0.1);
  protocol.tick(0.999);
  EXPECT_EQ(count("send", 0), 1);
  protocol.tick(1);
  EXPECT_EQ(count("send", 0), 2);
  ASSERT_TRUE(protocol.request(6, 1.1));
  protocol.tick(2);
  EXPECT_EQ(count("send", 0), 3);
  EXPECT_EQ(count("send", 6), 2);
  EXPECT_EQ(protocol.pending(), 6);
}

TEST(MmmProtocol, EverySingleFrameDropTerminatesWithFakePeer)
{
  for (int number : {1, 2, 3, 4, 6}) {
    // Three exchange legs; a lost echo can expose a fourth frame on retry.
    for (int drop : {-1, 0, 1, 2, 3}) {
      SCOPED_TRACE(::testing::Message() << "number=" << number << " drop=" << drop);
      std::deque<std::pair<char, int>> frames;
      std::vector<int> work, transitions;
      mmm::Protocol protocol(
        [&](int n) {if (n != 0) {frames.emplace_back('C', n);}},
        [&](int n) {work.push_back(n);},
        [](std::string text) {ADD_FAILURE() << text;});
      std::optional<int> peer_pending = number % 2 ? std::optional<int>(number) : std::nullopt;
      int sent = 0;
      bool dropped = false;
      if (peer_pending) {frames.emplace_back('J', number);} else {
        ASSERT_TRUE(protocol.request(number, 0));
      }
      for (int step = 0; step < 12; ++step) {
        const double now = step * 0.5;
        while (!frames.empty()) {
          ASSERT_LT(sent, 7) << "echo loop did not terminate";
          const auto frame = frames.front();
          frames.pop_front();
          if (sent++ == drop) {dropped = true; continue;}
          if (frame.first == 'J') {
            protocol.receive(frame.second, now);
          } else if (frame.second % 2 == 0) {
            frames.emplace_back('J', frame.second);
            if (transitions.empty()) {transitions.push_back(frame.second);}
          } else if (peer_pending == frame.second) {
            peer_pending.reset();
            frames.emplace_back('J', frame.second);
          }
        }
        if (step < 10) {
          protocol.tick(now);
          if (peer_pending) {frames.emplace_back('J', *peer_pending);}
        }
      }
      EXPECT_TRUE(frames.empty());
      EXPECT_FALSE(peer_pending);
      EXPECT_FALSE(protocol.pending());
      EXPECT_EQ(work, number % 2 ? std::vector<int>{number} : std::vector<int>{});
      EXPECT_EQ(transitions, number % 2 ? std::vector<int>{} : std::vector<int>{number});
      EXPECT_EQ(dropped, drop >= 0);
    }
  }
}

bool settle(mmm::StartGate & gate, mmm::Pose pose = {0, 0, 0}, double start = 0)
{
  for (int i = 0; i <= 21; ++i) {
    const double now = start + i * 0.1;
    gate.update(pose, now, now);
  }
  return gate.at_start();
}

TEST(MmmStartGate, InwardHysteresisOuterLimitAndParkedReturn)
{
  mmm::StartGate gate({0, 0, 0});
  EXPECT_FALSE(settle(gate, {0.14, 0, 0}));
  EXPECT_TRUE(settle(gate, {0.11, 0, 0}, 3));
  EXPECT_TRUE(gate.update(mmm::Pose{0.14, 0, 0}, 5.2, 5.2));
  EXPECT_FALSE(gate.update(mmm::Pose{0.151, 0, 0}, 5.3, 5.3));
  EXPECT_FALSE(settle(gate, {0.14, 0, 0}, 6));
  EXPECT_TRUE(settle(gate, {0.1, 0, 0}, 9));
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0.151}, 11.2, 11.2));
}

TEST(MmmStartGate, ExactDwellAndRepeatedStampCannotAccumulateEvidence)
{
  mmm::StartGate gate({0, 0, 0});
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0}, 0, 0));
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0}, 0, 0.5));
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0}, 1.999, 1.999));
  EXPECT_TRUE(gate.update(mmm::Pose{0, 0, 0}, 2, 2));
  gate.reset({0, 0, 0});
  for (double now : {0.0, 0.1, 0.4, 3.0}) {
    EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0}, 0, now));
  }
}

TEST(MmmStartGate, MissingStaleFutureBackwardsAndNonfiniteClearReadiness)
{
  mmm::StartGate gate({0, 0, 0});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  ASSERT_TRUE(settle(gate));
  EXPECT_FALSE(gate.update(std::nullopt, 2.2, 2.2));
  ASSERT_TRUE(settle(gate, {0, 0, 0}, 3));
  EXPECT_TRUE(gate.update(mmm::Pose{0, 0, 0}, 5.25, 5.75));
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0}, 5.25, 5.751));
  ASSERT_TRUE(settle(gate, {0, 0, 0}, 6));
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0}, 8.3, 8.2));
  ASSERT_TRUE(settle(gate, {0, 0, 0}, 9));
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0}, 1, 1));
  for (mmm::Pose pose : {mmm::Pose{nan, 0, 0}, {0, inf, 0}, {0, 0, nan}}) {
    ASSERT_TRUE(settle(gate, {0, 0, 0}, 12));
    EXPECT_FALSE(gate.update(pose, 14.2, 14.2));
  }
  ASSERT_TRUE(settle(gate, {0, 0, 0}, 15));
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0}, nan, 17.2));
  ASSERT_TRUE(settle(gate, {0, 0, 0}, 18));
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0}, 20.2, inf));
}

TEST(MmmStartGate, ParkedToleranceIncludesBoundaryAndRejectsMotion)
{
  mmm::StartGate gate({0, 0, 0});
  ASSERT_TRUE(settle(gate));
  EXPECT_TRUE(gate.update(mmm::Pose{0.03, 0, 0.03}, 2.2, 2.2));
  EXPECT_FALSE(gate.update(mmm::Pose{0.03001, 0, 0.03}, 2.3, 2.3));
  ASSERT_TRUE(settle(gate, {0, 0, 0}, 3));
  EXPECT_FALSE(gate.update(mmm::Pose{0, 0, 0.03001}, 5.2, 5.2));
}

TEST(MmmStartGate, ResetReanchorsAndYawWraps)
{
  const double pi = std::acos(-1.0);
  mmm::StartGate gate({1, 2, pi - 0.01});
  EXPECT_TRUE(settle(gate, {1, 2, -pi + 0.01}));
  gate.reset({3, 4, 0});
  EXPECT_FALSE(gate.at_start());
  EXPECT_FALSE(settle(gate, {1, 2, 0}, 3));
  EXPECT_TRUE(settle(gate, {3, 4, 0}, 6));
}

TEST(MmmStartGate, SaveGateAllowsFullOuterTolerance)
{
  mmm::StartGate gate({0, 0, 0}, 0.15);
  EXPECT_TRUE(settle(gate, {0.15, 0, 0.15}));
  gate.reset({0, 0, 0});
  EXPECT_TRUE(settle(gate, {0.09, 0.12, 0.15}, 3));
}

TEST(MmmScanGate, TransientFailuresRequireTwentyNewScans)
{
  const std::array<double, 4> geometry{10, 0, 0.9, 0.1};
  mmm::ScanGate gate;
  double stamp = 1;
  for (int failure = 0; failure < 4; ++failure) {
    for (int i = 0; i < 20; ++i) {
      stamp += 0.1;
      ASSERT_TRUE(gate.update(geometry, stamp, stamp));
      EXPECT_EQ(gate.ready(stamp), i == 19);
    }
    EXPECT_FALSE(gate.ready(stamp + 0.501));
    if (failure == 0) {EXPECT_FALSE(gate.update(geometry, stamp, stamp));}
    if (failure == 1) {EXPECT_FALSE(gate.update(geometry, stamp - 1, stamp));}
    if (failure == 2) {EXPECT_FALSE(gate.update(geometry, stamp + 1, stamp));}
    if (failure == 3) {
      EXPECT_FALSE(gate.update({10, 0, 0.9, 0}, stamp + 0.1, stamp + 0.1));
    }
    EXPECT_FALSE(gate.ready(stamp));
    EXPECT_FALSE(gate.geometry_failed());
  }
  for (int i = 0; i < 20; ++i) {
    stamp += 0.1;
    EXPECT_TRUE(gate.update(geometry, stamp, stamp));
  }
  EXPECT_TRUE(gate.ready(stamp));
}

TEST(MmmScanGate, RealGeometryChangeRemainsRejectedUntilNewSession)
{
  mmm::ScanGate gate;
  ASSERT_TRUE(gate.update({10, 0, 0.9, 0.1}, 1, 1));
  EXPECT_FALSE(gate.update({11, 0, 1, 0.1}, 2, 2));
  EXPECT_TRUE(gate.geometry_failed());
  for (int i = 3; i < 30; ++i) {
    EXPECT_FALSE(gate.update({10, 0, 0.9, 0.1}, i, i));
  }
  EXPECT_FALSE(gate.ready(29));
  gate = mmm::ScanGate{};
  EXPECT_TRUE(gate.update({11, 0, 1, 0.1}, 30, 30));
  EXPECT_FALSE(gate.geometry_failed());
}

TEST(MmmWorkflow, FreshStartResetsReadyFailurePendingAndCooldown)
{
  for (bool first_success : {false, true}) {
    std::vector<int> sends;
    mmm::Workflow * active = nullptr;
    mmm::Protocol protocol([&](int n) {sends.push_back(n);}, [&](int n) {
        if (n == 1) {active->start_session();}
      }, [](std::string) {});
    mmm::Workflow workflow(protocol);
    active = &workflow;
    protocol.receive(1, 0);
    workflow.setup_result(true, first_success);
    workflow.tick(true, 4.9);
    const int first = first_success ? 2 : 6;
    ASSERT_EQ(protocol.pending(), first);
    protocol.receive(first, 5.0);
    protocol.receive(1, 5.1);
    EXPECT_FALSE(protocol.completed_even());
    workflow.tick(true, 5.1);
    EXPECT_FALSE(protocol.pending());
    workflow.setup_result(true, first_success);
    workflow.tick(true, 5.2);
    EXPECT_EQ(protocol.pending(), first);
    protocol.receive(1, 5.3);  // Final leg cannot start more work.
    EXPECT_EQ(protocol.pending(), first);
    protocol.receive(1, 10.4);  // A new start cancels the previous pending even.
    EXPECT_FALSE(protocol.pending());
    workflow.setup_result(true, true);
    workflow.tick(true, 10.5);
    EXPECT_EQ(protocol.pending(), 2);
    protocol.receive(2, 10.6);
    EXPECT_EQ(protocol.completed_even(), 2);
  }
}

TEST(MmmSave, SessionBusyPoseAndCompletedReadyAllMatter)
{
  for (bool session : {false, true}) {
    for (bool busy : {false, true}) {
      for (bool at_start : {false, true}) {
        for (auto completed : {std::optional<int>{}, {2}, {4}, {6}}) {
          const auto expected = !session || busy ? mmm::SaveDecision::Ignore :
            (at_start && completed == 2 ? mmm::SaveDecision::Ready : mmm::SaveDecision::Incomplete);
          EXPECT_EQ(mmm::decide_save(session, busy, at_start, completed), expected);
        }
      }
    }
  }
}

TEST_F(ProtocolTest, SaveAckIsPublishedBeforeBusyGuardOrWork)
{
  for (bool busy : {false, true}) {
    std::vector<std::string> order;
    mmm::Protocol core([&](int) {order.push_back("ack");}, [&](int) {
        if (mmm::decide_save(true, busy, true, 2) != mmm::SaveDecision::Ignore) {
          order.push_back("launch");
        }
      }, [](std::string) {});
    core.receive(3, 0);
    core.receive(3, 0.1);
    EXPECT_EQ(order, busy ? std::vector<std::string>({"ack", "ack"}) :
      std::vector<std::string>({"ack", "launch", "ack"}));
  }
}

TEST_F(ProtocolTest, WorkflowFollowsPoseAfterSuccessfulMappingSetup)
{
  mmm::Workflow workflow(protocol);
  workflow.tick(true, 0);
  EXPECT_TRUE(events.empty());
  workflow.setup_result(true, true);
  workflow.tick(true, 0);
  EXPECT_EQ(protocol.pending(), 2);
  protocol.receive(2, 0.1);
  workflow.tick(true, 0.2);
  EXPECT_FALSE(protocol.pending());
  workflow.tick(false, 0.3);
  EXPECT_EQ(protocol.pending(), 4);
  protocol.receive(4, 0.4);
  workflow.tick(true, 1);
  EXPECT_FALSE(protocol.pending());
  workflow.tick(true, 5.1);
  EXPECT_EQ(protocol.pending(), 2);
}

TEST_F(ProtocolTest, WorkflowFailedReadyMustCompleteFourBeforeRetryingTwo)
{
  mmm::Workflow workflow(protocol);
  workflow.setup_result(true, true);
  workflow.tick(true, 0);
  protocol.receive(2, 0.1);
  workflow.save_result(true, false);
  workflow.tick(true, 5);
  EXPECT_EQ(protocol.pending(), 4);
  // Stale Ready echoes cannot bypass the forced Mapping exchange.
  protocol.receive(2, 5.1);
  workflow.tick(true, 10);
  EXPECT_EQ(protocol.pending(), 4);
  protocol.receive(4, 10.1);
  workflow.tick(true, 10.2);
  EXPECT_EQ(protocol.pending(), 2);
  protocol.receive(2, 10.3);
  EXPECT_EQ(protocol.completed_even(), 2);
  EXPECT_EQ(count("send", 4), 2);
}

TEST_F(ProtocolTest, WorkflowFailedRecoveryFourRetriesBeforeReady)
{
  mmm::Workflow workflow(protocol);
  workflow.setup_result(true, true);
  workflow.tick(true, 0);
  protocol.receive(2, 0.1);
  workflow.save_result(true, false);
  workflow.tick(true, 5);
  timeout(5);
  workflow.tick(true, 10);
  EXPECT_FALSE(protocol.pending());
  workflow.tick(true, 14.99);
  EXPECT_FALSE(protocol.pending());
  workflow.tick(true, 15);
  EXPECT_EQ(protocol.pending(), 4);
  protocol.receive(4, 15.1);
  workflow.tick(true, 15.2);
  EXPECT_EQ(protocol.pending(), 2);
}

TEST_F(ProtocolTest, WorkflowFailedOrNonmappingSetupSelectsSix)
{
  for (const auto & setup : {std::make_pair(false, false), {false, true}, {true, false}}) {
    SCOPED_TRACE(::testing::Message() << setup.first << "," << setup.second);
    mmm::Workflow workflow(protocol);
    workflow.setup_result(setup.first, setup.second);
    const double now = 20 + count("send", 6) * 10;
    workflow.tick(true, now);
    EXPECT_EQ(protocol.pending(), 6);
    protocol.receive(6, now + 0.1);
    workflow.tick(false, now + 0.2);
    EXPECT_FALSE(protocol.pending());
    EXPECT_EQ(protocol.completed_even(), 6);
  }
}
TEST(MmmSave, ParkedGuardLatchesFailureUntilNextSave)
{
  mmm::SaveGuard guard;
  guard.begin(true);
  guard.observe(true);
  EXPECT_FALSE(guard.accept(false));
  EXPECT_TRUE(guard.accept(true));
  guard.observe(false);
  guard.observe(true);
  EXPECT_FALSE(guard.accept(true));
  guard.begin(true);
  EXPECT_TRUE(guard.accept(true));
  guard.begin(false);
  guard.observe(false);
  EXPECT_TRUE(guard.accept(true));
  EXPECT_FALSE(guard.accept(false));
}

TEST_F(ProtocolTest, IncompleteFailureAndSuccessfulReadyDoNotForceRecovery)
{
  mmm::Workflow workflow(protocol);
  workflow.setup_result(true, true);
  workflow.tick(true, 0);
  protocol.receive(2, 0.1);
  workflow.save_result(false, false);
  workflow.tick(true, 1);
  EXPECT_FALSE(protocol.pending());
  workflow.save_result(true, true);
  workflow.tick(true, 2);
  EXPECT_FALSE(protocol.pending());
}
}  // namespace
