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
#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace thornbots_pkg {
bool MatchWifiGuard::observe(unsigned stage) {
  if (fired_) { return false; }
  consecutive_ = stage == 4 ? consecutive_ + 1 : 0;
  if (consecutive_ < 3) { return false; }
  fired_ = true;
  return true;
}
bool write_wifi_off_request(const std::string &path, std::string &error) {
  std::string temporary = path + ".XXXXXX";
  const int fd = mkstemp(temporary.data());
  if (fd < 0) { error = std::strerror(errno); return false; }
  constexpr char contents[] = "MATCH_BATTLE_WIFI_OFF\n";
  bool ok = fchmod(fd, 0600) == 0;
  std::size_t offset = 0;
  while (ok && offset < sizeof(contents) - 1) {
    const auto count = write(fd, contents + offset, sizeof(contents) - 1 - offset);
    if (count < 0 && errno == EINTR) { continue; }
    if (count <= 0) { ok = false; break; }
    offset += count;
  }
  if (ok) { ok = fsync(fd) == 0; }
  if (close(fd) != 0) { ok = false; }
  if (ok) { ok = rename(temporary.c_str(), path.c_str()) == 0; }
  if (!ok) {
    error = std::strerror(errno);
    unlink(temporary.c_str());
  }
  return ok;
}
}  // namespace thornbots_pkg
