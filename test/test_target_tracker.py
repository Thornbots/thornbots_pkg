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
Unit tests for target_tracker_core.py's normal/spin/Kalman-filter logic.

Unit tests for target_tracker_core.py's pure normal-estimation/spin-
detection/Kalman-filter logic, against synthetic inputs. Mirrors
test_target_selector.py -- no rclpy, no ROS message packages, runs on a
bare Python 3 + pytest install. Run with
`python3 -m pytest test/test_target_tracker.py`.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from thornbots_pkg.target_tracker_core import (  # noqa: E402
    corrected_centre, KalmanFilter6D, SpinDetector,
)


# ── corrected_centre ──────────────────────────────────────────────────────

def test_corrected_centre_extends_along_boresight():
    panel = np.array([4.0, 0.0, 0.0])
    c = corrected_centre(panel, 0.3)
    assert np.allclose(c, [4.3, 0.0, 0.0])


def test_corrected_centre_extends_along_off_axis_bearing():
    # Off-boresight panel: correction must follow the panel's own bearing
    # from the camera, not a fixed axis -- this is exactly what the
    # corner-cross-product approach got wrong (see target_tracker_core.py's
    # docstring).
    panel = np.array([3.0, 4.0, 0.0])  # range 5, bearing (0.6, 0.8, 0)
    c = corrected_centre(panel, 0.5)
    expected = panel + 0.5 * np.array([0.6, 0.8, 0.0])
    assert np.allclose(c, expected)


def test_corrected_centre_degenerate_at_origin():
    c = corrected_centre(np.array([0.0, 0.0, 0.0]), 0.3)
    assert np.allclose(c, [0.0, 0.0, 0.0])


# ── SpinDetector ──────────────────────────────────────────────────────────

def test_no_handoffs_reports_not_spinning():
    s = SpinDetector(handoff_timeout_s=1.5, min_handoffs=3, cv_max=0.35)
    spinning, hz, phase = s.update(0.0, class_id=2)
    assert not spinning
    for t in np.arange(0.1, 2.0, 0.1):
        spinning, hz, phase = s.update(float(t), class_id=2)
    assert not spinning
    assert hz == 0.0


def test_regular_handoffs_detected_as_spinning():
    s = SpinDetector(handoff_timeout_s=1.5, min_handoffs=3, cv_max=0.35)
    # 4 panels, handoff every 0.25s -> spin period ~1s -> spin_hz ~1.0
    class_ids = [0, 1, 2, 3, 0, 1, 2, 3, 0]
    spinning = False
    hz = 0.0
    for i, cid in enumerate(class_ids):
        t = i * 0.25
        spinning, hz, phase = s.update(t, cid)
    assert spinning
    assert abs(hz - 1.0) < 0.1


def test_irregular_handoffs_not_spinning():
    s = SpinDetector(handoff_timeout_s=5.0, min_handoffs=3, cv_max=0.35)
    class_ids_times = [(0, 0), (1, 0.1), (2, 0.9), (3, 1.0), (0, 2.5)]
    spinning = True
    for cid, t in class_ids_times:
        spinning, hz, phase = s.update(t, cid)
    assert not spinning


def test_stale_handoff_times_out():
    s = SpinDetector(handoff_timeout_s=0.5, min_handoffs=2, cv_max=0.35)
    s.update(0.0, 0)
    s.update(0.2, 1)
    s.update(0.4, 2)
    spinning, hz, phase = s.update(0.6, 2)
    assert spinning  # not yet timed out relative to last change at 0.4
    spinning, hz, phase = s.update(1.0, 2)
    assert not spinning  # 0.6s since last change > 0.5s timeout


# ── KalmanFilter6D ────────────────────────────────────────────────────────

def test_kf_tracks_constant_velocity():
    kf = KalmanFilter6D([0.0, 0.0, 0.0], t_sec=0.0, pos_var=0.01)
    v = np.array([2.0, 0.0, 0.0])
    t = 0.0
    for _ in range(50):
        t += 0.05
        pos = v * t
        kf.predict(t, process_noise_accel=0.5)
        kf.update(pos, pos_var=0.01)
    assert np.allclose(kf.state[:3], v * t, atol=0.1)
    assert np.allclose(kf.state[3:], v, atol=0.3)


def test_kf_stationary_stays_near_zero_velocity():
    kf = KalmanFilter6D([1.0, 2.0, 0.5], t_sec=0.0, pos_var=0.01)
    t = 0.0
    for _ in range(30):
        t += 0.05
        kf.predict(t, process_noise_accel=0.2)
        kf.update([1.0, 2.0, 0.5], pos_var=0.01)
    assert np.allclose(kf.state[3:], [0.0, 0.0, 0.0], atol=0.1)


def test_kf_predicted_extrapolates_without_mutating():
    kf = KalmanFilter6D([0.0, 0.0, 0.0], t_sec=0.0, pos_var=0.01)
    v = np.array([2.0, 0.0, 0.0])
    t = 0.0
    for _ in range(50):
        t += 0.05
        kf.predict(t, process_noise_accel=0.5)
        kf.update(v * t, pos_var=0.01)

    before = kf.state.copy()
    state, variance = kf.predicted(t + 0.25, process_noise_accel=0.5)
    # Extrapolated a quarter second along the tracked velocity...
    assert np.allclose(state[:3], before[:3] + before[3:] * 0.25)
    # ...with more position uncertainty than at the filter's own time...
    assert (variance[:3] > np.diag(kf.P)[:3]).all()
    # ...and the filter itself untouched.
    assert np.allclose(kf.state, before)


def test_kf_predicted_at_or_before_filter_time_is_current_state():
    kf = KalmanFilter6D([1.0, 2.0, 3.0], t_sec=10.0, pos_var=0.01)
    state, variance = kf.predicted(9.5, process_noise_accel=0.5)
    assert np.allclose(state, kf.state)
    assert np.allclose(variance, np.diag(kf.P))


def test_lagged_measurement_time_recovers_true_velocity():
    # The spin branch feeds a windowed MEAN, whose effective time is the
    # window's mean time, ~window/2 behind the newest sample. Stamping it
    # at the newest sample instead biases the velocity low; stamping it
    # correctly recovers it.
    v = np.array([1.5, 0.0, 0.0])
    window_s = 0.5
    dt = 1.0 / 60.0

    def run(stamp_at_newest):
        kf = None
        window = []
        t = 0.0
        for _ in range(240):
            t += dt
            window.append((t, *(v * t)))
            window = [w for w in window if t - w[0] <= window_s]
            meas = np.array([w[1:] for w in window]).mean(axis=0)
            meas_t = t if stamp_at_newest else float(np.mean([w[0] for w in window]))
            if kf is None:
                kf = KalmanFilter6D(meas, meas_t, pos_var=0.01)
            else:
                kf.predict(meas_t, process_noise_accel=0.5)
                kf.update(meas, pos_var=0.01)
        return kf, t

    kf_wrong, t_end = run(stamp_at_newest=True)
    kf_right, _ = run(stamp_at_newest=False)

    assert np.allclose(kf_right.state[3:], v, atol=0.05)
    # Position at the newest sample's time, once extrapolated forward.
    state, _ = kf_right.predicted(t_end, process_noise_accel=0.5)
    assert np.allclose(state[:3], v * t_end, atol=0.05)
    # The mis-stamped filter lags in position by roughly half the window.
    lag_m = (v * t_end)[0] - kf_wrong.state[0]
    assert lag_m > 0.5 * window_s * v[0] * 0.5
