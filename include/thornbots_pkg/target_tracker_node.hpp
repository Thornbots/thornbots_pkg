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

#ifndef THORNBOTS_PKG__TARGET_TRACKER_NODE_HPP_
#define THORNBOTS_PKG__TARGET_TRACKER_NODE_HPP_
#include <memory>
#include "rclcpp/rclcpp.hpp"
namespace thornbots_pkg
{
std::shared_ptr<rclcpp::Node> make_target_tracker(const rclcpp::NodeOptions & options);
}  // namespace thornbots_pkg
#endif  // THORNBOTS_PKG__TARGET_TRACKER_NODE_HPP_
