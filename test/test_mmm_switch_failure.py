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

"""Failed localization handoffs must send 6 while the coordinator stays alive."""
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from dji_serial_bridge.msg import Ping
import pytest
import rclpy
from std_srvs.srv import Trigger


@pytest.mark.parametrize('mode,accepted', [
    ('amcl', False), ('amcl', True), ('none', None), ('slam', False)])
def test_failed_switch_sends_six(tmp_path, mode, accepted):
    """Reject absent, rejected and acknowledged-but-stalled handoffs safely."""
    rclpy.init()
    node = rclpy.create_node('mmm_switch_failure_test')
    events, requests = [], []
    node.create_subscription(Ping, '/dji_serial_bridge/ping_to_mcb',
                             lambda message: events.append(message.number), 10)
    publisher = node.create_publisher(Ping, '/dji_serial_bridge/ping_from_mcb', 10)

    def trigger(request, response):
        requests.append(request)
        response.success = accepted
        return response

    if accepted is not None:
        node.create_service(Trigger, '/localization/start_mapping', trigger)
    executable = Path(get_package_prefix('thornbots_pkg')) / 'lib/thornbots_pkg/mmm'
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', f'localization_mode:={mode}',
        '-p', 'timeout_s:=2.5', '-p', f'map_save_dir:={tmp_path}'])

    def wait(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert process.poll() is None, 'coordinator exited during handoff'
            assert time.monotonic() < deadline, f'deadline exceeded, events={events}'
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        wait(lambda: publisher.get_subscription_count() > 0 and 0 in events)
        message = Ping()
        message.header.stamp = node.get_clock().now().to_msg()
        message.number = 1
        publisher.publish(message)
        wait(lambda: 6 in events)
        assert 2 not in events
        if accepted is not None:
            assert len(requests) == 1
        assert process.poll() is None
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
