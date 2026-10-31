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
