# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License
# is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language governing
# permissions and limitations under the License.
"""auto.launch.py declarations resolve with no arguments, as the robot service launches it."""

from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import pytest

LAUNCH = Path(__file__).parents[1] / 'launch/auto.launch.py'


@pytest.mark.parametrize('arguments', [{}, {'real_hardware': 'false'}])
def test_argument_defaults_resolve_in_declaration_order(arguments):
    description = PythonLaunchDescriptionSource(str(LAUNCH)).get_launch_description(
        LaunchContext())
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    declarations = [e for e in description.entities if isinstance(e, DeclareLaunchArgument)]
    assert declarations
    for declaration in declarations:
        declaration.visit(context)
    expected = 'false' if not arguments else 'true'
    assert context.launch_configurations['mmm_room'] == expected


REMOVED_ARGUMENTS = {
    'lead_enabled', 'firmware_latency_s', 'chase_settle_s', 'chase_margin_s',
    'v_muzzle', 'fire_lock_s', 'gimbal_lag_s',
}


def test_planner_arguments_moved_to_the_mcb_are_not_declared():
    description = PythonLaunchDescriptionSource(str(LAUNCH)).get_launch_description(
        LaunchContext())
    declared = {e.name for e in description.entities if isinstance(e, DeclareLaunchArgument)}
    assert not declared & REMOVED_ARGUMENTS
    assert {'cv_target_publish_rate_hz', 'idle_direction', 'target_state_topic'} <= declared


def _description_with(environment, arguments=None):
    """Perform the DDS environment actions and return the resulting process env."""
    import os
    from launch.actions import SetEnvironmentVariable, UnsetEnvironmentVariable

    context = LaunchContext()
    context.launch_configurations.update(arguments or {})
    description = PythonLaunchDescriptionSource(str(LAUNCH)).get_launch_description(context)
    saved = dict(os.environ)
    os.environ.clear()
    os.environ.update(environment)
    try:
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.visit(context)
        for entity in description.entities:
            if isinstance(entity, (SetEnvironmentVariable, UnsetEnvironmentVariable)):
                entity.visit(context)
        return dict(context.environment)
    finally:
        os.environ.clear()
        os.environ.update(saved)


def test_remote_dds_is_off_by_default():
    environment = _description_with({'FASTRTPS_DEFAULT_PROFILES_FILE': '/etc/fastdds/profile.xml'})
    assert environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in environment


@pytest.mark.parametrize('how', ['argument', 'environment'])
def test_remote_dds_on_restores_the_peer_profile(how):
    environment = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    arguments = {}
    if how == 'argument':
        arguments['remote_dds'] = 'true'
    else:
        environment['THORNBOTS_REMOTE_DDS'] = '1'
    result = _description_with(environment, arguments)
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE' not in result
    assert result['FASTRTPS_DEFAULT_PROFILES_FILE'] == '/etc/fastdds/profile.xml'


def test_pinned_profiles_have_peers_only_when_remote():
    config = Path(__file__).parents[1] / 'config'
    assert '100.' in (config / 'fastdds_udp_only.xml').read_text()
    local = (config / 'fastdds_udp_only_local.xml').read_text()
    import re
    assert set(re.findall(r'<address>([^<]+)</address>', local)) == {'127.0.0.1'}
