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


def producer_actions(producer, hardware, engine=''):
    import importlib.util
    spec = importlib.util.spec_from_file_location('auto_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        detection_producer=producer, real_hardware=hardware, engine_file_path=engine)
    return module.detection_producers(context)


def test_external_producer_needs_no_optional_packages(monkeypatch):
    def unexpected_lookup(*args):
        pytest.fail('external mode resolved an optional producer package')
    monkeypatch.setattr('ament_index_python.packages.get_package_share_directory',
                        unexpected_lookup)
    assert producer_actions('external', 'true') == []
    assert producer_actions('external', 'false') == []


def test_sim_producer_is_one_complete_chain():
    actions = producer_actions('sim', 'false')
    assert [action.node_executable[0].text for action in actions] == [
        'target_driver', 'cv_target_emulator', 'cv_head_aim']
    assert all(action.node_package[0].text == 'sim' for action in actions)
    # Visiting the nodes in the installed-package smoke resolves their executables.


def test_hardware_producer_disables_second_uart(monkeypatch):
    monkeypatch.setattr('ament_index_python.packages.get_package_share_directory',
                        lambda package: '/share/' + package)
    actions = producer_actions('hardware', 'true', '/models/device.plan')
    assert len(actions) == 1
    arguments = dict(actions[0].launch_arguments)
    assert arguments == {'engine_file_path': '/models/device.plan',
                         'enable_serial_bridge': 'False'}


@pytest.mark.parametrize('producer,hardware,engine', [
    ('sim', 'true', ''), ('hardware', 'false', '/models/device.plan'),
    ('hardware', 'true', ''), ('invalid', 'false', ''),
])
def test_invalid_producer_configuration_fails(producer, hardware, engine):
    with pytest.raises(RuntimeError):
        producer_actions(producer, hardware, engine)


@pytest.mark.parametrize('operator_only', ['true', 'false'])
def test_managed_producers_share_consumer_gate(operator_only):
    from launch.actions import GroupAction, OpaqueFunction
    description = PythonLaunchDescriptionSource(str(LAUNCH)).get_launch_description(
        LaunchContext())
    groups = [e for e in description.entities if isinstance(e, GroupAction)]
    producer_groups = [g for g in groups if any(
        isinstance(a, OpaqueFunction) for a in g.get_sub_entities())]
    assert len(producer_groups) == 1
    context = LaunchContext()
    context.launch_configurations['operator_inputs_only'] = operator_only
    assert producer_groups[0].condition.evaluate(context) == (operator_only == 'false')
