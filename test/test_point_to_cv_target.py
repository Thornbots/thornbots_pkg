"""
Unit tests for point_to_cv_target_core.py's pure intercept-solve math.
No rclpy, no ROS message packages. Run with
`python3 -m pytest test/test_point_to_cv_target.py`.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sentry_pkg.point_to_cv_target_core import (  # noqa: E402
    LatencyStat, solve_intercept,
)


def test_stationary_target_no_lead():
    aim, t = solve_intercept((4.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0), tau=0.0, v_muzzle=25.0)
    assert math.isclose(aim[0], 4.0, abs_tol=1e-6)
    assert math.isclose(aim[1], 0.0, abs_tol=1e-6)
    assert math.isclose(t, 4.0 / 25.0, rel_tol=1e-3)


def test_crossing_target_leads_along_travel():
    # Target 4m ahead, moving 2 m/s sideways (+y). Lead must push aim +y.
    aim, t = solve_intercept((4.0, 0.0, 0.0), (0.0, 2.0, 0.0),
                              (0.0, 0.0, 0.0), tau=0.0, v_muzzle=25.0)
    assert aim[1] > 0.0
    assert math.isclose(aim[1], 2.0 * t, rel_tol=1e-6)


def test_no_vertical_lead_from_horizontal_motion():
    aim, t = solve_intercept((4.0, 0.0, 0.0), (0.0, 2.0, 0.0),
                              (0.0, 0.0, 0.0), tau=0.0, v_muzzle=25.0)
    assert math.isclose(aim[2], 0.0, abs_tol=1e-9)


def test_latency_extends_lead():
    aim_no_latency, _ = solve_intercept((4.0, 0.0, 0.0), (0.0, 2.0, 0.0),
                                         (0.0, 0.0, 0.0), tau=0.0, v_muzzle=25.0)
    aim_latency, _ = solve_intercept((4.0, 0.0, 0.0), (0.0, 2.0, 0.0),
                                      (0.0, 0.0, 0.0), tau=0.1, v_muzzle=25.0)
    assert aim_latency[1] > aim_no_latency[1]


def test_zero_velocity_converges_immediately():
    aim, t = solve_intercept((10.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                              (0.0, 0.0, 0.0), tau=0.0, v_muzzle=25.0,
                              iterations=1)
    assert math.isclose(t, 10.0 / 25.0, rel_tol=1e-6)


def test_latency_stat_running_mean():
    stat = LatencyStat()
    for sample in (0.05, 0.07, 0.06):
        stat.add(sample)
    assert stat.count == 3
    assert math.isclose(stat.mean, 0.06, rel_tol=1e-6)
