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

"""Unit tests for mcb_relay_core.Relocalizer, no rclpy: `pytest test/test_mcb_relay.py`."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from thornbots_pkg.mcb_relay_core import Relocalizer  # noqa: E402


def _drive(rel, vx, t_end, dt=0.01, x0=0.0):
    """Feed /odom at 100 Hz for a chassis moving at vx along x from x0 at t=0."""
    t = 0.0
    while t <= t_end + 1e-9:
        rel.add_odom(t, x0 + vx * t, 0.0, vx, 0.0)
        t += dt


def test_still_offset_sends_localization_pose():
    rel = Relocalizer()
    _drive(rel, 0.0, 0.5)
    x, y, apply_t, error, std = rel.decide(0.4, 0.10, 0.0, 1e-6, now=0.5)
    assert x == pytest.approx(0.10) and y == pytest.approx(0.0)
    assert error == pytest.approx(0.10)
    assert apply_t == pytest.approx(0.5 + 0.005 + 0.002)


def test_moving_offset_lands_where_the_mcb_will_be():
    rel = Relocalizer(latency_std_s=0.001)
    _drive(rel, 2.0, 0.5)
    # Localization at t=0.4 says we were 0.1 m further on than /odom did.
    x, _, apply_t, _, _ = rel.decide(0.4, 2.0 * 0.4 + 0.1, 0.0, 1e-6, now=0.52)
    # The MCB reads its odometry at apply_t; /odom shows that one UART leg later.
    mcb_odom_at_apply = 2.0 * (apply_t + rel.uart_latency_s)
    assert x == pytest.approx(mcb_odom_at_apply + 0.1)


def test_small_or_uncertain_offsets_are_not_sent():
    rel = Relocalizer()
    _drive(rel, 0.0, 0.5)
    assert rel.decide(0.4, 0.04, 0.0, 1e-6, now=0.5) is None  # under 0.05 m
    assert rel.decide(0.4, 0.10, 0.0, 0.03 ** 2, now=0.5) is None  # std over max_std_m
    # 0.055 m clears the threshold but not 3 sigma of 0.019 m.
    assert rel.decide(0.4, 0.055, 0.0, 0.019 ** 2, now=0.5) is None


def test_speed_adds_extrapolation_uncertainty():
    rel = Relocalizer(latency_std_s=0.003, max_std_m=0.02)
    _drive(rel, 8.0, 0.5)  # 8 m/s x 3 ms = 0.024 m std, over max_std_m
    assert rel.decide(0.4, 8.0 * 0.4 + 0.2, 0.0, 1e-6, now=0.5) is None


def test_hold_off_after_a_send():
    rel = Relocalizer(hold_off_s=0.3)
    _drive(rel, 0.0, 1.0)
    assert rel.decide(0.4, 0.1, 0.0, 1e-6, now=0.5) is not None
    assert rel.decide(0.6, 0.1, 0.0, 1e-6, now=0.7) is None
    assert rel.decide(0.8, 0.1, 0.0, 1e-6, now=0.85) is not None


def test_localization_stamp_outside_odom_history():
    rel = Relocalizer(history_s=1.0)
    _drive(rel, 0.0, 2.0)
    assert rel.decide(0.5, 0.1, 0.0, 1e-6, now=2.0) is None  # too old, dropped
    assert rel.decide(2.3, 0.1, 0.0, 1e-6, now=2.3) is None  # too far ahead
    assert rel.decide(2.05, 0.1, 0.0, 1e-6, now=2.05) is not None  # just ahead


def test_interpolates_between_odom_samples():
    rel = Relocalizer()
    rel.add_odom(0.0, 0.0, 0.0, 0.0, 0.0)
    rel.add_odom(0.1, 1.0, 2.0, 0.0, 0.0)
    assert rel.odom_at(0.025) == pytest.approx((0.25, 0.5))
    rel.add_odom(0.05, 0.0, 0.0, 0.0, 0.0)  # stamps went back: a sim restart
    assert rel.odom_at(0.05) == (0.0, 0.0) and len(rel._odom) == 1
