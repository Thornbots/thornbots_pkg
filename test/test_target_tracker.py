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

"""
Unit tests for target_tracker_core.py's armor-model EKF.

Synthetic 4-panel targets only (radii 0.30/0.24, the emulator's layout), no
rclpy. Run with `python3 -m pytest test/test_target_tracker.py`.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from thornbots_pkg.target_tracker_core import (  # noqa: E402
    ArmorEKF, ArmorTracker, panel_positions, ray_covariance,
)

CAMERA = np.array([0.0, 0.0, 0.4])
RX, RY = 0.30, 0.24
NOISE_M = 0.03
DT = 1.0 / 60.0


def _true_panels(centre, yaw, stagger=0.0):
    # Pair 0 (k = 0, 2) sits stagger/2 above the centre, pair 1 below, as in
    # cv_target_emulator.
    return [(centre + (RX if k % 2 == 0 else RY)
             * np.array([math.cos(yaw + k * math.pi / 2.0),
                         math.sin(yaw + k * math.pi / 2.0), 0.0])
             + np.array([0.0, 0.0, stagger / 2.0 if k % 2 == 0 else -stagger / 2.0]),
             yaw + k * math.pi / 2.0) for k in range(4)]


def _seen_panel(centre, yaw, stagger=0.0):
    """Most head-on panel within 75 degrees of the camera, like the emulator."""
    best = None
    for pos, yaw_k in _true_panels(centre, yaw, stagger):
        to_cam = CAMERA - pos
        cos_view = (math.cos(yaw_k) * to_cam[0] + math.sin(yaw_k) * to_cam[1]) \
            / np.linalg.norm(to_cam[:2])
        if cos_view > math.cos(math.radians(75.0)) and (best is None or cos_view > best[0]):
            best = (cos_view, pos)
    return None if best is None else best[1]


def _run(velocity, spin_rad_s, seconds, seed=0, noise=NOISE_M, start=(3.0, 0.0, 0.3),
         cls=ArmorEKF, bad_start_s=0.0, yaw0=0.3, stagger=0.0):
    """Feed a noisy constant-velocity spinning target; return (filter, truth centre, truth yaw)."""
    rng = np.random.default_rng(seed)
    ekf = None
    centre, yaw = np.array(start), 0.3
    for i in range(int(seconds / DT)):
        t = i * DT
        centre = np.array(start) + np.array(velocity) * t
        yaw = yaw0 + spin_rad_s * t
        seen = _seen_panel(centre, yaw, stagger)
        if seen is None:
            continue
        meas = seen + rng.normal(0.0, noise if t >= bad_start_s else 0.15, 3)
        if ekf is None:
            ekf = cls(meas, CAMERA, t, 0.05 ** 2, 0.27, 2.0, 5.0, 0.02)
        else:
            ekf.step(meas, CAMERA, t, 0.05 ** 2)
    return ekf, centre, yaw


def test_stationary_target_estimates_centre_and_no_spin():
    ekf, centre, _ = _run((0.0, 0.0, 0.0), 0.0, 3.0)
    # Only the seen panel is observable, so check it rather than the centre.
    panel = ekf.state[:3] + ekf.state[8] * np.array(
        [math.cos(ekf.state[6]), math.sin(ekf.state[6]), 0.0])
    assert np.linalg.norm(panel - _seen_panel(centre, 0.3)) < 0.03
    assert abs(ekf.state[7]) < 1.0


def test_spin_in_place_recovers_rate_centre_and_both_radii():
    w = 2.0 * math.pi * 1.5
    ekf, centre, _ = _run((0.0, 0.0, 0.0), w, 4.0)
    assert abs(ekf.state[7] - w) < 0.1 * w
    assert np.linalg.norm(ekf.state[:2] - centre[:2]) < 0.05
    radii = sorted([ekf.state[8], ekf.other_r])
    assert abs(radii[0] - RY) < 0.04 and abs(radii[1] - RX) < 0.04


def test_spin_direction_is_signed():
    ekf, _, _ = _run((0.0, 0.0, 0.0), -2.0 * math.pi, 4.0)
    assert ekf.state[7] < -0.9 * 2.0 * math.pi


def test_spinning_while_translating_recovers_velocity_and_rate():
    w = 2.0 * math.pi * 1.5
    ekf, centre, _ = _run((0.0, 1.0, 0.0), w, 3.0, start=(3.0, -1.5, 0.3))
    assert abs(ekf.state[7] - w) < 0.15 * w
    assert abs(ekf.state[4] - 1.0) < 0.3
    assert np.linalg.norm(ekf.state[:2] - centre[:2]) < 0.1


def test_handoff_steps_yaw_a_quarter_turn_and_swaps_radius():
    # Panel at (2.7, 0) faces the camera at yaw pi; turn the chassis 50 deg
    # clockwise so the k=1 panel (yaw pi + 40 deg) is the more head-on one.
    ekf = ArmorEKF((2.7, 0.0, 0.3), CAMERA, 0.0, 1e-4, 0.30, 2.0, 5.0, 0.02)
    ekf.other_r = 0.24
    ekf.state[6] -= math.radians(50.0)
    yaw_before = ekf.state[6]
    k1_pos = panel_positions(ekf.state, ekf.other_r)[1][2]
    k, distance = ekf.associate(k1_pos, CAMERA)
    assert k == 1 and distance < 1e-9
    assert math.isclose(ekf.state[6], yaw_before + math.pi / 2.0)
    assert ekf.state[8] == 0.24 and ekf.other_r == 0.30


def test_back_panel_is_never_associated():
    ekf = ArmorEKF((2.7, 0.0, 0.3), CAMERA, 0.0, 1e-4, 0.30, 2.0, 5.0, 0.02)
    back = panel_positions(ekf.state, ekf.other_r)[2][2]
    k, _ = ekf.associate(back, CAMERA)
    assert k != 2


def test_predicted_does_not_mutate_or_alias():
    ekf, _, _ = _run((0.0, 1.0, 0.0), 6.0, 1.0)
    before, P_before, t_before = ekf.state.copy(), ekf.P.copy(), ekf.t_sec
    state, P = ekf.predicted(ekf.t_sec + 0.2)
    assert state[1] > before[1] and state[6] > before[6]
    state[:] = 0.0
    P[:] = 0.0
    same_t, same_P = ekf.predicted(ekf.t_sec)
    same_t[:] = 0.0
    same_P[:] = 0.0
    assert np.array_equal(ekf.state, before) and np.array_equal(ekf.P, P_before)
    assert ekf.t_sec == t_before


def test_jink_reacquires_position_and_keeps_spin():
    w = 2.0 * math.pi * 1.5
    ekf, _, _ = _run((0.0, 0.0, 0.0), w, 3.0)
    t = ekf.t_sec
    jumped = np.array([3.0, 0.8, 0.3]) + np.array([-RX, 0.0, 0.0])
    results = [ekf.step(jumped, CAMERA, t + (i + 1) * DT, 0.05 ** 2) for i in range(3)]
    assert results == ['outlier', 'outlier', 'reacquire']
    assert abs(ekf.state[1] - 0.8) < 0.1
    assert abs(ekf.state[7] - w) < 0.1 * w


def test_radius_is_clamped():
    ekf = ArmorEKF((2.7, 0.0, 0.3), CAMERA, 0.0, 1e-4, 0.27, 2.0, 5.0, 0.02)
    ekf.update((1.0, 0.0, 0.3), 1e-6)
    assert ekf.r_min <= ekf.state[8] <= ekf.r_max


def test_tracker_bank_recovers_the_spin_after_a_bad_first_second():
    # A lone EKF fed 15cm noise for the first second (the head slewing in,
    # in sim) locks onto a wrong spin for good on most seeds; the bank must
    # find the true rate on nearly all of them.
    w = 2.0 * math.pi * 1.5
    single = bank = 0
    for seed in range(8):
        kwargs = {'seed': seed, 'bad_start_s': 1.0, 'yaw0': seed * 0.37}
        single += abs(_run((0.0, 0.0, 0.0), w, 6.0, **kwargs)[0].state[7] - w) < 0.1 * w
        bank += abs(_run((0.0, 0.0, 0.0), w, 6.0, cls=ArmorTracker, **kwargs)[0].state[7] - w) \
            < 0.1 * w
    assert bank >= 7
    assert bank > single


def test_tracker_bank_leads_with_the_right_spin_sign():
    tracker, _, _ = _run((0.0, 0.0, 0.0), -2.0 * math.pi * 1.5, 4.0, cls=ArmorTracker)
    assert tracker.state[7] < -0.9 * 2.0 * math.pi * 1.5


def test_ray_covariance_is_depth_along_the_ray_and_lateral_across():
    cov = ray_covariance((3.0, 4.0, 0.4), CAMERA, depth_std=0.1, lateral_std=0.02)
    ray = np.array([3.0, 4.0, 0.0]) / 5.0
    across = np.array([-4.0, 3.0, 0.0]) / 5.0
    assert math.isclose(ray @ cov @ ray, 0.01, rel_tol=1e-9)
    assert math.isclose(across @ cov @ across, 0.0004, rel_tol=1e-9)
    assert math.isclose(cov[2, 2], 0.0004, rel_tol=1e-9)


def test_tracker_bank_locks_spin_under_heavy_depth_noise_with_ray_covariance():
    # 12cm depth noise, 3cm lateral: the sim's range model at 3m. An
    # isotropic 12cm R let w=0 explain the sweep; the ray covariance must not.
    w = 2.0 * math.pi * 2.0
    locked = 0
    for seed in range(8):
        rng = np.random.default_rng(seed)
        tracker = None
        for i in range(int(5.0 / DT)):
            t = i * DT
            seen = _seen_panel(np.array([3.0, 0.0, 0.3]), seed * 0.4 + w * t)
            if seen is None:
                continue
            u = (seen - CAMERA) / np.linalg.norm(seen - CAMERA)
            meas = seen + rng.normal(0.0, 0.03, 3) + u * rng.normal(0.0, 0.12)
            cov = ray_covariance(meas, CAMERA, 0.12, 0.04)
            if tracker is None:
                tracker = ArmorTracker(meas, CAMERA, t, cov, 0.27, 2.0, 5.0, 0.02)
            else:
                tracker.step(meas, CAMERA, t, cov)
        locked += abs(tracker.state[7] - w) < 0.1 * w
    assert locked >= 7


STAGGER_M = 0.09  # sim's staggered layout: 90% of a 0.1 m panel


def _panel_error(filt, centre, yaw, stagger):
    # Worst distance from a true panel to the nearest panel the state implies.
    implied = [pos for _, _, pos in panel_positions(filt.state, filt.other_r)]
    return max(min(np.linalg.norm(pos - p) for p in implied)
               for pos, _ in _true_panels(centre, yaw, stagger))


def test_staggered_spin_recovers_both_pair_heights():
    w = 2.0 * math.pi * 1.5
    for cls in (ArmorEKF, ArmorTracker):
        filt, centre, yaw = _run((0.0, 0.0, 0.0), w, 4.0, cls=cls, stagger=STAGGER_M)
        assert abs(abs(filt.state[9]) - STAGGER_M / 2.0) < 0.015
        assert abs(filt.state[2] - centre[2]) < 0.015
        # The sign too: every implied panel sits on a true one, heights included.
        assert _panel_error(filt, centre, yaw, STAGGER_M) < 0.05


def test_flat_spin_keeps_dz_near_zero():
    filt, _, _ = _run((0.0, 0.0, 0.0), 2.0 * math.pi * 1.5, 4.0, cls=ArmorTracker)
    assert abs(filt.state[9]) < 0.01


def test_odd_handoff_flips_dz_and_its_covariance():
    ekf = ArmorEKF((2.7, 0.0, 0.3), CAMERA, 0.0, 1e-4, 0.30, 2.0, 5.0, 0.02)
    ekf.state[9], ekf.P[2, 9] = 0.04, -1e-4
    ekf.P[9, 2] = ekf.P[2, 9]
    ekf.state[6] -= math.radians(50.0)
    k1_pos = panel_positions(ekf.state, ekf.other_r)[1][2]
    assert math.isclose(k1_pos[2], 0.3 - 0.04)
    k, _ = ekf.associate(k1_pos, CAMERA)
    assert k == 1
    assert math.isclose(ekf.state[9], -0.04) and ekf.P[2, 9] == ekf.P[9, 2] == 1e-4
    assert np.all(np.linalg.eigvalsh(ekf.P) >= -1e-12)
