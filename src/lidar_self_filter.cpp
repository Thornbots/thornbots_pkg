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

// Filter the robot's own head out of the lidar scan: blank a fixed angular
// sector of scan_raw where the head sits in the lidar's FOV (fixed in the
// lidar's frame; no joint-state sub needed), republishing on scan. Works for
// sim and real hardware.
//
// Current values: blind_angle_start=0.32, blind_angle_end=1.62 in the scan frame
// (18.3-92.8 deg CCW from scan zero), from the 2026-10-26 CAD. See README.md for design rationale and
// retuning notes.

#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace thornbots_pkg
{

using sensor_msgs::msg::LaserScan;

class LidarSelfFilter : public rclcpp::Node
{
public:
  LidarSelfFilter()
  : Node("lidar_self_filter")
  {
    declare_parameter("blind_angle_start", 0.32);
    declare_parameter("blind_angle_end", 1.62);
    sub_ = create_subscription<LaserScan>(
      "scan_raw", 10, [this](LaserScan::ConstSharedPtr m) {scan_callback(*m);});
    pub_ = create_publisher<LaserScan>("scan", 10);
  }

private:
  void scan_callback(const LaserScan & msg)
  {
    const double start = get_parameter("blind_angle_start").as_double();
    const double end = get_parameter("blind_angle_end").as_double();

    LaserScan out = msg;
    const double two_pi = 2.0 * M_PI;
    const double angle_min = msg.angle_min, increment = msg.angle_increment;
    for (size_t i = 0; i < out.ranges.size(); ++i) {
      const double angle = angle_min + static_cast<double>(i) * increment;
      // Wrap into [0, 2*pi) so this compares correctly for any angle_min.
      // Sim and the RPLIDAR driver both publish -pi..+pi (720 bins); the
      // sector is stated in 0..2*pi. Floor modulo.
      double wrapped = std::fmod(angle, two_pi);
      if (wrapped < 0.0) {
        wrapped += two_pi;
      }
      if (start <= wrapped && wrapped <= end) {
        out.ranges[i] = std::numeric_limits<float>::infinity();
        if (i < out.intensities.size()) {
          out.intensities[i] = 0.0f;
        }
      }
    }
    pub_->publish(out);
  }

  rclcpp::Subscription<LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
};

}  // namespace thornbots_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thornbots_pkg::LidarSelfFilter>());
  rclcpp::shutdown();
  return 0;
}
