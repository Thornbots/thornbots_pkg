# Copyright 2026 RHIT Thornbots
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

"""Exercise the in-stack mapping coordinator against Gazebo and a fake MCB."""
import copy
import json
import math
import os
from pathlib import Path
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from dji_serial_bridge.msg import Ping
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def generate_test_description():
    """Run the actual sim and robot entrypoints in the test's isolated domain."""
    directory = tempfile.mkdtemp(prefix='mmm-integration-')
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('sim'), 'launch', 'sim.launch.py')),
        launch_arguments={'gui': 'false', 'rviz': 'false', 'foxglove': 'false',
                          'model': 'sentry_v2', 'yaw': '0.0', 'real_time_factor': '1.0',
                          'odom_noise_enabled': 'false'}.items())
    mode = os.environ.get('THORNBOTS_MMM_START_MODE', 'mapping')
    robot = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        launch_arguments={'real_hardware': 'false', 'localization_mode': mode,
                          'load_map': 'false', 'autosave_map': 'false', 'use_rf2o': 'false',
                          'mmm_room': 'true', 'map_save_dir': directory,
                          'mmm_shutdown_after_save': 'false', 'mmm_timeout_s': '45.0',
                          'enable_target_selector': 'false', 'enable_target_tracker': 'false',
                          'enable_cv_target_bridge': 'false', 'dds_transport': 'udp_only'}.items())
    return LaunchDescription([
        sim, TimerAction(period=5.0, actions=[robot]), launch_testing.actions.ReadyToTest()
    ]), {'directory': directory, 'mode': mode}


class TestMmmFlow(unittest.TestCase):
    """MCB echoes all Jetson transactions and controls the fake operator inputs."""

    def test_start_drive_return_save(self, directory, mode):
        rclpy.init()
        node = rclpy.create_node('fake_mcb_mmm_test')
        ping = node.create_publisher(Ping, '/dji_serial_bridge/ping_from_mcb', 10)
        drive = node.create_publisher(Twist, '/cmd_vel', 10)
        scans = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        evidence = {'pose': None, 'even': None, 'odd': None, 'events': [], 'scan': None}
        buffer = Buffer()
        listener = TransformListener(buffer, node)

        def persistent_processes():
            names = {'mmm', 'dji_serial_bridge_node', 'pose_translator',
                     'odom_tf_broadcaster', 'mcb_relay', 'robot_state_publisher'}
            found = {}
            for path in Path('/proc').glob('[0-9]*/cmdline'):
                try:
                    name = Path(path.read_bytes().split(b'\0')[0].decode()).name
                    if name in names:
                        found[name] = int(path.parent.name)
                except (OSError, UnicodeError):
                    pass
            return found

        def send(number):
            message = Ping()
            message.header.stamp = node.get_clock().now().to_msg()
            message.number = number
            ping.publish(message)

        def receive(message):
            number = message.number
            evidence['events'].append(number)
            if number in (2, 4, 6):
                evidence['even'] = number
                send(number)
            elif number == evidence['odd']:
                evidence['odd'] = None
                send(number)

        node.create_subscription(Ping, '/dji_serial_bridge/ping_to_mcb', receive, 10)
        node.create_subscription(Odometry, '/odom',
                                 lambda m: evidence.update(pose=m.pose.pose.position),
                                 qos_profile_sensor_data)

        node.create_subscription(LaserScan, '/scan',
                                 lambda m: evidence.update(scan=m), qos_profile_sensor_data)

        def bad_scan():
            message = copy.deepcopy(evidence['scan'])
            self.assertIsNotNone(message)
            message.header.stamp.sec = 0
            message.header.stamp.nanosec = 0
            scans.publish(message)

        def wait(predicate, seconds=45, action=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if action:
                    action()
                if predicate():
                    return
                if evidence['even'] == 6:
                    self.fail('setup failed (PING 6), events=' + str(evidence['events']))
            self.fail('MMM flow deadline, events=' + str(evidence['events'])
                      + ', pose=' + str(evidence['pose']))

        def command(value):
            message = Twist()
            message.linear.x = value
            drive.publish(message)

        try:
            wait(lambda: evidence['pose'] is not None and 0 in evidence['events'])
            # Let the mapping lifecycle and first map settle before requesting reset.
            until = time.monotonic() + 3
            wait(lambda: time.monotonic() >= until)
            names_before = node.get_node_names()
            processes_before = persistent_processes()
            self.assertIn('mmm', processes_before)
            self.assertIn('pose_translator', processes_before)
            self.assertIn('mmm', names_before)
            if mode == 'amcl':
                self.assertNotIn('map_autosaver', names_before)
                self.assertIn('amcl', names_before)
                self.assertIn('map_server', names_before)
                self.assertNotIn('slam_toolbox', names_before)
            evidence['odd'] = 1
            started = time.monotonic()
            send(1)
            wait(lambda: any(Path(directory).glob('mmm-*')) and evidence['scan'] is not None)
            until = time.monotonic() + 1.0
            wait(lambda: time.monotonic() >= until)
            bad_scan()  # A transient scan during boot must restart evidence, not send 6.
            wait(lambda: evidence['even'] == 2)
            print(f'MMM START TO READY ({mode}): {time.monotonic() - started:.3f} s', flush=True)
            names_after = node.get_node_names()
            self.assertEqual(persistent_processes(), processes_before)
            self.assertIn('slam_toolbox', names_after)
            self.assertIn('map_autosaver', names_after)
            wait(lambda: 'amcl' not in node.get_node_names()
                 and 'map_server' not in node.get_node_names())
            first_boot = next(Path(directory).glob('mmm-*/boot.json'))
            until = time.monotonic() + 5.1
            wait(lambda: time.monotonic() >= until)
            evidence['even'] = None
            evidence['odd'] = 1
            send(1)
            wait(lambda: evidence['even'] == 2)
            boots = list(Path(directory).glob('mmm-*/boot.json'))
            self.assertEqual(len(boots), 2)
            boot_path = next(path for path in boots if path != first_boot)
            bad_scan()  # Ready saves must recover from late scans, too.
            until = time.monotonic() + 2.0
            wait(lambda: time.monotonic() >= until)
            start = evidence['pose'].x
            wait(lambda: evidence['pose'].x >= start + 0.7,
                 action=lambda: command(0.15))
            command(0.0)
            wait(lambda: evidence['even'] == 4)
            boot = json.loads(boot_path.read_text())

            def return_to_map_start():
                try:
                    tf = buffer.lookup_transform('map', 'root', Time())
                except TransformException:
                    command(0.0)
                    return
                p, q = tf.transform.translation, tf.transform.rotation
                # Re-park inside the wider gate without returning to the exact boot mark.
                dx, dy = boot['root'][0] + 0.4 - p.x, boot['root'][1] - p.y
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                speed = min(1.0, 0.15 / max(0.001, math.hypot(dx, dy)))
                message = Twist()
                if math.hypot(dx, dy) > 0.015:
                    message.linear.x = speed * (math.cos(yaw) * dx + math.sin(yaw) * dy)
                    message.linear.y = speed * (-math.sin(yaw) * dx + math.cos(yaw) * dy)
                drive.publish(message)

            wait(lambda: evidence['even'] == 2, action=return_to_map_start)
            command(0.0)
            parked = buffer.lookup_transform('map', 'root', Time()).transform.translation
            offset = math.hypot(parked.x - boot['root'][0], parked.y - boot['root'][1])
            self.assertGreater(offset, 0.3)
            self.assertLessEqual(offset, 0.5)
            evidence['odd'] = 3
            send(3)
            wait(lambda: evidence['odd'] is None)
            outputs = lambda: list(Path(directory).glob('mmm-*/snapshot/save.json'))
            wait(lambda: bool(outputs()))
            bundle = outputs()[0].parent
            self.assertTrue(json.loads(outputs()[0].read_text())['validated'])
            for name in ('map.posegraph', 'map.data', 'map.pgm', 'map.yaml', 'boot.json'):
                wait(lambda: (bundle / name).exists())
                self.assertGreater((bundle / name).stat().st_size, 0)
            self.assertTrue(json.loads((bundle / 'boot.json').read_text())['session_id'])
            print('MMM INTEGRATION PASS: transient boot scan -> ready -> restart -> ready -> '
                  'transient Ready scan -> mapping -> ready -> save; '
                  + str(bundle), flush=True)
        finally:
            command(0.0)
            del listener
            node.destroy_node()
            rclpy.shutdown()
