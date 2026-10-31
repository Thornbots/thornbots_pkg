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

#include "thornbots_pkg/room_check_core.hpp"
#include <gtest/gtest.h>

namespace rc = thornbots_pkg::room_check;
TEST(RoomCheck, HealthyCadenceAndMissingReferee)
{
  rc::Stream s;
  for (int i = 0; i < 900; ++i) {
    const double t = 100 + i / 90.0;
    s.add(t, t, t - 0.01, 0.5, 0.05);
  }
  EXPECT_EQ(s.cadence("POSE", 100, 110, 90, 0.2, 0.1).status, "PASS");
  EXPECT_TRUE(s.stamps_ok());
  rc::Stream missing;
  EXPECT_EQ(missing.availability("REF", false).status, "SKIP");
  EXPECT_EQ(missing.availability("REF", true).status, "FAIL");
}
TEST(RoomCheck, DisappearingAndLateTopicsFail)
{
  rc::Stream s;
  for (int i = 0; i < 450; ++i) {
    const double t = 100 + i / 90.0;
    s.add(t, t, t, 0.5, 0.05);
  }
  EXPECT_EQ(s.cadence("POSE", 100, 110, 90, 0.2, 0.1).status, "FAIL");
  EXPECT_GT(s.gap(100, 110), 5);
  EXPECT_EQ(s.cadence("POSE", 95, 105, 90, 0.2, 0.1).status, "FAIL");
}
TEST(RoomCheck, StampFailures)
{
  rc::Stream s;
  s.add(100, 1, 0, 0.5, 0.05);
  s.add(101, 2, 102, 0.5, 0.05);
  s.add(102, 3, 100, 0.5, 0.05);
  EXPECT_EQ(s.invalid, 1U);
  EXPECT_EQ(s.future, 1U);
  EXPECT_EQ(s.stale, 2U);
  EXPECT_EQ(s.backwards, 1U);
  EXPECT_FALSE(s.stamps_ok());
}
TEST(RoomCheck, GeometryAndBadBeamInjection)
{
  rc::Geometry g;
  for (int i = 0; i < 10; ++i) {g.add(720, 0.008726646, -M_PI, M_PI, 720);}
  EXPECT_EQ(g.score().status, "PASS");
  g.add(719, 0.008726646, -M_PI, M_PI, 720);
  EXPECT_EQ(g.bad, 1U);
  EXPECT_EQ(g.changed, 1U);
  EXPECT_EQ(g.score().status, "FAIL");
  g.add(720, std::numeric_limits<double>::quiet_NaN(), -M_PI, M_PI, 720);
  EXPECT_EQ(g.bad, 2U);
}
TEST(RoomCheck, TransientDriftReturningToStartStillFails)
{
  rc::Drift d;
  d.add(1, 0, 0, 3.14);
  d.add(6, 0.2, 0, -3.14);
  d.add(11, 0, 0, 3.14);
  EXPECT_EQ(d.score("drift", 10, 0.03, 0.03, true).status, "FAIL");
  EXPECT_NEAR(d.path, 0.4, 1e-6);
  EXPECT_LT(d.max_yaw, 0.01);
}
TEST(RoomCheck, ShortAndInvalidDriftCannotPass)
{
  rc::Drift d;
  EXPECT_EQ(d.score("missing", 10, 0.03, 0.03, false).status, "SKIP");
  d.add(1, 0, 0, 0);
  d.add(2, 0, 0, 0);
  EXPECT_EQ(d.score("short", 10, 0.03, 0.03, true).status, "FAIL");
  d.add(11, 0, 0, 0);
  EXPECT_EQ(d.score("parked", 10, 0.03, 0.03, true).status, "PASS");
  d.add(12, NAN, 0, 0);
  EXPECT_EQ(d.score("invalid", 10, 0.03, 0.03, true).status, "FAIL");
}
TEST(RoomCheck, StaleCvNeedsLiveTargetAndCutoffEvidence)
{
  rc::StaleCV cv;
  EXPECT_EQ(cv.score(10).status, "FAIL");
  cv.detection(1); cv.detection(1.1);
  cv.decision(1.15, true);
  cv.decision(1.4, false); cv.decision(1.5, false);
  EXPECT_EQ(cv.score(3).status, "PASS");
  cv.decision(1.31, true);
  EXPECT_EQ(cv.score(3).status, "FAIL");
  const auto info = rc::diagnostic(cv.score(3));
  EXPECT_EQ(info.status, "INFO");
  EXPECT_EQ(info.detail.rfind("FAIL (diagnostic only): ", 0), 0u);
}
TEST(RoomCheck, LoggedDropReports)
{
  EXPECT_TRUE(rc::drop_report("Message Filter dropping message"));
  EXPECT_TRUE(rc::drop_report("Scan gap 0.3s: a scan was skipped"));
  EXPECT_TRUE(rc::drop_report("Scan geometry mismatch"));
  EXPECT_FALSE(rc::drop_report("SLAM ready"));
}

TEST(RoomCheck, LatencyRequiresCoverageAndMeetsBudget)
{
  EXPECT_EQ(rc::latency_score({}, 0, 0.05, 0.2).status, "FAIL");
  EXPECT_EQ(rc::latency_score(std::vector<double>(20, 0.08), 20, 0.05, 0.2).status, "PASS");
  EXPECT_EQ(rc::latency_score(std::vector<double>(20, 0.3), 20, 0.05, 0.2).status, "FAIL");
  EXPECT_EQ(rc::latency_score(std::vector<double>(20, -0.1), 20, 0.05, 0.2).status, "FAIL");
  EXPECT_EQ(rc::latency_score(std::vector<double>(20, 0.08), 40, 0.05, 0.2).status, "FAIL");
}
