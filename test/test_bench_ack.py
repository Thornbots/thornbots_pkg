# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check consumer acknowledgments without starting a ROS graph."""
from types import SimpleNamespace

from dji_serial_bridge.msg import TargetState
from rclpy.clock import ClockType
from rclpy.time import Time
from thornbots_pkg.point_to_cv_target import PointToCvTarget
from thornbots_pkg.point_to_cv_target_core import LatencyStat


def _ros_now():
    return Time(nanoseconds=123000000, clock_type=ClockType.ROS_TIME)


def probe(active):
    points, ticks = [], []
    node = SimpleNamespace(
        target_active=active, patrol=None, odom_frame='odom',
        type_c_based_patrol=False, turn_to_hit=False,
        get_clock=lambda: SimpleNamespace(now=_ros_now),
        _compute_aim_point=lambda: ((3.0, 1.0, 0.3), 0.0),
        _fire_decision=lambda delay, now: (True, 0),
        pub=SimpleNamespace(publish=points.append),
        tick_pub=SimpleNamespace(publish=ticks.append))
    PointToCvTarget.on_publish_tick(node)
    return points, ticks


def test_no_target_tick_does_not_require_an_absent_aim_point():
    points, ticks = probe(False)
    assert not points
    assert len(ticks) == 1
    assert ticks[0].frame_id == ''
    assert Time.from_msg(ticks[0].stamp).nanoseconds == 123000000


def test_target_tick_identifies_the_output_frame_and_stamp():
    points, ticks = probe(True)
    assert len(points) == len(ticks) == 1
    assert ticks[0] == points[0].header
    assert ticks[0].frame_id == 'odom'


def test_model_ack_is_sent_after_consumption_with_the_input_stamp():
    model = TargetState()
    model.header.stamp = Time(nanoseconds=100000000).to_msg()
    model.header.frame_id = 'odom'
    acknowledged = []
    node = SimpleNamespace(
        latest_state=None, target_active=False, latency_stat=LatencyStat(),
        get_clock=lambda: SimpleNamespace(now=_ros_now),
        get_logger=lambda: SimpleNamespace(info=lambda *args, **kwargs: None))

    def received(header):
        assert node.latest_state is model
        assert node.target_active
        acknowledged.append(header)

    node.state_ack_pub = SimpleNamespace(publish=received)
    PointToCvTarget.on_target_state(node, model)
    assert acknowledged == [model.header]
