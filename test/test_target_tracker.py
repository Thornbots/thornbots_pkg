"""
Unit tests for target_tracker_core.py's pure normal-estimation/spin-
detection/Kalman-filter logic, against synthetic inputs. Mirrors
test_target_selector.py -- no rclpy, no ROS message packages, runs on a
bare Python 3 + pytest install. Run with
`python3 -m pytest test/test_target_tracker.py`.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sentry_pkg.target_tracker_core import (  # noqa: E402
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
