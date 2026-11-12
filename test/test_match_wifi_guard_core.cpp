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

#include "thornbots_pkg/match_wifi_guard_core.hpp"
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

TEST(MatchWifiGuard, ConsecutiveBattleAndOneShot) {
  thornbots_pkg::MatchWifiGuard guard;
  EXPECT_FALSE(guard.observe(4));
  EXPECT_FALSE(guard.observe(4));
  EXPECT_FALSE(guard.observe(3));
  EXPECT_FALSE(guard.observe(4));
  EXPECT_FALSE(guard.observe(4));
  EXPECT_TRUE(guard.observe(4));
  for (unsigned stage : {4, 5, 0, 4, 4, 4}) { EXPECT_FALSE(guard.observe(stage)); }
}
TEST(MatchWifiGuard, AtomicFixedRequestAndMissingDirectory) {
  char directory[] = "/tmp/match-wifi-core-XXXXXX";
  ASSERT_NE(mkdtemp(directory), nullptr);
  const std::filesystem::path root(directory), request = root / "wifi-off.request";
  std::string error;
  EXPECT_FALSE(thornbots_pkg::write_wifi_off_request((root / "missing/request").string(), error));
  EXPECT_FALSE(std::filesystem::exists(root / "missing"));
  EXPECT_FALSE(error.empty());
  ASSERT_TRUE(thornbots_pkg::write_wifi_off_request(request.string(), error));
  struct stat info {};
  ASSERT_EQ(stat(request.c_str(), &info), 0);
  EXPECT_EQ(info.st_mode & 07777, 0600);
  EXPECT_EQ(info.st_nlink, 1U);
  std::ifstream input(request);
  const std::string contents((std::istreambuf_iterator<char>(input)), {});
  EXPECT_EQ(contents, "MATCH_BATTLE_WIFI_OFF\n");
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(root),
    std::filesystem::directory_iterator()), 1);
  // Failed rename must remove the temporary file as well.
  EXPECT_FALSE(thornbots_pkg::write_wifi_off_request(root.string(), error));
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(root),
    std::filesystem::directory_iterator()), 1);
  std::filesystem::remove_all(root);
}
