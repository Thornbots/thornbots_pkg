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
#include <dji_serial_bridge/msg/ref_sys_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

class MatchWifiGuardNode : public rclcpp::Node {
 public:
  MatchWifiGuardNode() : Node("match_wifi_guard") {
    request_ = declare_parameter<std::string>(
      "request_file", "/run/thornbots-net/wifi-off.request");
    subscription_ = create_subscription<dji_serial_bridge::msg::RefSysStatus>(
      "/dji_serial_bridge/ref_sys", rclcpp::SensorDataQoS(),
      [this](dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr message) {
        if (!guard_.observe(message->game_stage)) { return; }
        std::string error;
        if (thornbots_pkg::write_wifi_off_request(request_, error)) {
          RCLCPP_WARN(get_logger(), "Battle confirmed: host Wi-Fi disconnect requested");
        } else {
          RCLCPP_ERROR(get_logger(), "Wi-Fi request failed: %s (check host request mount)",
            error.c_str());
        }
      });
  }
 private:
  thornbots_pkg::MatchWifiGuard guard_;
  std::string request_;
  rclcpp::Subscription<dji_serial_bridge::msg::RefSysStatus>::SharedPtr subscription_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MatchWifiGuardNode>());
  rclcpp::shutdown();
  return 0;
}
