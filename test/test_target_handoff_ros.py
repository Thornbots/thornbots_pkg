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
"""The real manager, relay and bridge preserve stamps and send explicit clear on a PTY."""

import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from dji_serial_bridge.msg import TargetState
import rclpy
from rclpy.qos import qos_profile_sensor_data


def test_manager_relay_bridge_clear():
    rclpy.init()
    node = rclpy.create_node('target_handoff_ros_test')
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    processes, managed, relayed = [], [], []
    source = node.create_publisher(TargetState, '/cv/tracker/target_state', 10)
    subscriptions = [
        node.create_subscription(TargetState, '/cv/target_state', managed.append, 10),
        node.create_subscription(TargetState, '/dji_serial_bridge/target_state',
                                 relayed.append, qos_profile_sensor_data)]
    data = bytearray()
    with tempfile.TemporaryFile(mode='w+') as log:
        def start(package, name, *params):
            executable = Path(get_package_prefix(package)) / 'lib' / package / name
            command = [str(executable), '--ros-args']
            for param in params:
                command += ['-p', param]
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))

        def poll(predicate, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.005)
                try:
                    data.extend(os.read(master, 8192))
                except BlockingIOError:
                    pass
                if predicate():
                    return True
                if any(p.poll() is not None for p in processes):
                    break
            return False

        def wait(predicate):
            if poll(predicate, 8.0):
                return
            log.seek(0)
            raise AssertionError(log.read() + f'\nPTY bytes={data.hex()}')

        def state_frames():
            frames, offset = [], 0
            while offset + 7 <= len(data):
                assert data[offset] == 0xa5
                length, = struct.unpack_from('<H', data, offset + 1)
                if offset + length + 9 > len(data):
                    break
                msg_type, = struct.unpack_from('<H', data, offset + 5)
                if msg_type == 7:
                    frames.append(data[offset + 7:offset + 7 + length])
                offset += length + 9
            return frames

        try:
            start('dji_serial_bridge', 'dji_serial_bridge_node',
                  f'device:={os.ttyname(slave)}', 'diag_interval_s:=0', 'debug_log:=false')
            start('thornbots_pkg', 'mcb_relay', 'type_c_based_patrol:=true')
            start('thornbots_pkg', 'target_manager')
            wait(lambda: source.get_subscription_count() == 1 and
                 node.count_publishers('/cv/target_state') == 1 and
                 node.count_subscribers('/cv/target_state') == 2 and
                 node.count_publishers('/dji_serial_bridge/target_state') == 1 and
                 node.count_subscribers('/dji_serial_bridge/target_state') == 2)

            def model_state():
                model = TargetState()
                model.header.stamp = node.get_clock().now().to_msg()
                model.detection_stamp = model.header.stamp
                model.header.frame_id = 'odom'
                model.confidence = 0.9
                model.radius = [0.2, 0.3]
                model.valid = True
                model.robot_track_id = 76
                return model

            def retire():
                status = TargetState()
                status.header.stamp = node.get_clock().now().to_msg()
                status.header.frame_id = 'odom'
                status.selected_track = False
                return status

            # Graph counts do not prove each hop's reader matched its writer, and the
            # manager's reliable send is not replayed. Cycle a throwaway model through
            # manager, relay and bridge until it lands, then retire and discard it; the
            # checked cycle then cannot lose its model to discovery lag.
            def delivered():
                return (any(m.valid for m in relayed) and
                        any(f[79] == 0x03 for f in state_frames()))

            for _ in range(30):
                source.publish(model_state())
                if poll(delivered, 0.3):
                    break
            wait(delivered)
            source.publish(retire())  # a no-op if decay already released the model
            wait(lambda: relayed[-1].clear and state_frames()[-1][79] == 0x06)
            managed.clear()
            relayed.clear()
            data.clear()
            model = model_state()
            source.publish(model)
            wait(lambda: bool(managed) and bool(relayed) and bool(state_frames()))
            assert relayed[0].header == model.header
            assert relayed[0].detection_stamp == model.detection_stamp
            assert state_frames()[0][79] == 0x03
            source.publish(retire())
            wait(lambda: any(m.clear for m in relayed) and len(state_frames()) >= 2)
            clear = next(m for m in relayed if m.clear)
            assert clear.header == next(m for m in managed if m.clear).header
            assert clear.robot_track_id == 76 and not clear.valid
            assert clear.detection_stamp.sec == 0 and clear.detection_stamp.nanosec == 0
            payload = state_frames()[-1]
            assert len(payload) == 80 and payload[79] == 0x06
            assert struct.unpack_from('<H', payload, 74)[0] == 0
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in reversed(processes):
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            os.close(master)
            os.close(slave)
            del subscriptions
            node.destroy_node()
            rclpy.shutdown()
