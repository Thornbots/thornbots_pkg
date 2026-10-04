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

"""Unit tests for mcb_axes, no rclpy: `pytest test/test_mcb_axes.py`."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from thornbots_pkg.mcb_axes import aim_to_mcb, from_mcb, to_mcb  # noqa: E402


def test_forward_on_the_mcb_is_forward_in_rep105():
    # The MCB's y is forward of the boot heading, REP-105's x.
    assert from_mcb(0.0, 1.0) == pytest.approx((1.0, 0.0))
    assert from_mcb(1.0, 0.0) == pytest.approx((0.0, -1.0))  # its x is right


@pytest.mark.parametrize('x, y', [(0.3, -1.2), (-2.0, 0.5), (0.0, 0.0)])
def test_to_mcb_inverts_from_mcb(x, y):
    assert from_mcb(*to_mcb(x, y)) == pytest.approx((x, y))
    assert to_mcb(*from_mcb(x, y)) == pytest.approx((x, y))


@pytest.mark.parametrize('o', [(0.0, 0.0), (1.5, -0.7), (-3.0, 2.0)])
def test_firmware_aim_sees_the_rep105_offset(o):
    # AutoAimAndFireCommand.cpp:70-71 at 0885a69: atan2 of x/y less its
    # odometry, read as REP-105. Its odometry is to_mcb of our pose.
    t = (4.0, 1.0)
    wx, wy = aim_to_mcb(*t, *o)
    ox_mcb, oy_mcb = to_mcb(*o)
    dx, dy = wx - ox_mcb, wy - oy_mcb
    assert (dx, dy) == pytest.approx((t[0] - o[0], t[1] - o[1]))
    assert math.atan2(dy, dx) == pytest.approx(math.atan2(t[1] - o[1], t[0] - o[0]))


def test_aim_unchanged_at_the_origin():
    assert aim_to_mcb(2.0, -1.0, 0.0, 0.0) == pytest.approx((2.0, -1.0))
