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
Unit tests for point_to_cv_target_core.py's pure intercept-solve math.

No rclpy, no ROS message packages. Run with
`python3 -m pytest test/test_point_to_cv_target.py`.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from thornbots_pkg.point_to_cv_target_core import (  # noqa: E402
    LatencyStat, solve_intercept,
)

V_MUZZLE = 25.0
# solve_intercept's default iteration count leaves a bounded residual; the
# figures come from GEOMETRIES below, measured, not guessed.
ITER3_REL_TOL = 1e-2
CONVERGED_REL_TOL = 1e-9
# Worst measured componentwise aim error at the default iteration count is
# 6.2mm over GEOMETRIES; 2cm leaves headroom and still sits well inside the
# 0.1m armor panel, so a real aiming error can't hide under it.
ITER3_AIM_TOL_M = 0.02

# (target_pos, target_vel, tau), spanning crossing / receding / closing /
# oblique motion at ARCC ranges and speeds. Shooter is at the origin
# throughout -- moving it is CV_TEST_GAPS.md's gap 2, not this one.
GEOMETRIES = [
    ((4.0, 0.0, 0.0), (0.0, 2.0, 0.0), 0.0),
    ((4.0, 0.0, 0.0), (0.0, 2.0, 0.0), 0.12),
    ((8.0, 0.0, 0.0), (0.0, 4.0, 0.0), 0.0),
    ((8.0, 0.0, 0.0), (4.0, 0.0, 0.0), 0.0),    # receding head-on
    ((8.0, 0.0, 0.0), (-4.0, 0.0, 0.0), 0.0),   # closing head-on
    ((2.0, 0.0, 1.5), (0.0, 2.0, 0.0), 0.08),   # elevated target
    ((3.0, 4.0, 0.0), (1.0, -3.0, 0.5), 0.12),  # oblique, 3-D velocity
]


def _analytic_flight_time(target_pos, target_vel, tau, v_muzzle):
    """
    Solve the same intercept in closed form, as truth for the iterative solve.

    With the shooter at the origin the intercept condition
    |p + v*(tau + t)| = v_muzzle*t is a quadratic in t:
    (|v|^2 - v_muzzle^2) t^2 + 2 (d.v) t + |d|^2 = 0, where d = p + v*tau.
    Returns the smallest positive root (the first time the shot can
    arrive); the linear branch covers a target closing at exactly
    v_muzzle.
    """
    d = [target_pos[i] + target_vel[i] * tau for i in range(3)]
    a = sum(x * x for x in target_vel) - v_muzzle ** 2
    b = 2.0 * sum(d[i] * target_vel[i] for i in range(3))
    c = sum(x * x for x in d)
    if abs(a) < 1e-15:
        return -c / b
    disc = b * b - 4.0 * a * c
    assert disc >= 0.0, 'no real intercept for this geometry'
    roots = [r for r in ((-b - math.sqrt(disc)) / (2.0 * a),
                         (-b + math.sqrt(disc)) / (2.0 * a)) if r > 0.0]
    assert roots, 'no positive intercept time for this geometry'
    return min(roots)


def test_intercept_condition_holds():
    # The invariant the solve exists to satisfy: the aim point must be
    # exactly v_muzzle*t_flight away from the shooter, so the projectile
    # and the target arrive together. Everything else in this file is a
    # consequence of this plus the returned aim expression.
    for target_pos, target_vel, tau in GEOMETRIES:
        aim, t = solve_intercept(target_pos, target_vel, (0.0, 0.0, 0.0),
                                 tau=tau, v_muzzle=V_MUZZLE)
        assert math.isclose(math.dist(aim, (0.0, 0.0, 0.0)), V_MUZZLE * t,
                            rel_tol=ITER3_REL_TOL), (target_pos, target_vel, tau)


def test_flight_time_matches_closed_form():
    for target_pos, target_vel, tau in GEOMETRIES:
        _, t = solve_intercept(target_pos, target_vel, (0.0, 0.0, 0.0),
                               tau=tau, v_muzzle=V_MUZZLE)
        expected = _analytic_flight_time(target_pos, target_vel, tau, V_MUZZLE)
        assert math.isclose(t, expected, rel_tol=ITER3_REL_TOL), (
            target_pos, target_vel, tau)


def test_aim_point_matches_closed_form_componentwise():
    # Per-axis, not just |aim|: dropping the z lead moves the norm by well
    # under ITER3_REL_TOL on a low target while putting the shot 15cm high.
    for target_pos, target_vel, tau in GEOMETRIES:
        aim, _ = solve_intercept(target_pos, target_vel, (0.0, 0.0, 0.0),
                                 tau=tau, v_muzzle=V_MUZZLE)
        t = _analytic_flight_time(target_pos, target_vel, tau, V_MUZZLE)
        for axis in range(3):
            expected = target_pos[axis] + target_vel[axis] * (tau + t)
            assert math.isclose(aim[axis], expected, abs_tol=ITER3_AIM_TOL_M), (
                target_pos, target_vel, tau, axis)


def test_iterating_to_convergence_reaches_the_closed_form():
    # Pins the docstring's "2-3 converges in practice" as a measurement:
    # the fixed point is the closed-form root, and the default count is
    # a truncation of it rather than a different answer.
    for target_pos, target_vel, tau in GEOMETRIES:
        _, t = solve_intercept(target_pos, target_vel, (0.0, 0.0, 0.0),
                               tau=tau, v_muzzle=V_MUZZLE, iterations=40)
        expected = _analytic_flight_time(target_pos, target_vel, tau, V_MUZZLE)
        assert math.isclose(t, expected, rel_tol=CONVERGED_REL_TOL), (
            target_pos, target_vel, tau)


def test_stationary_target_no_lead():
    aim, t = solve_intercept((4.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0), tau=0.0, v_muzzle=25.0)
    assert math.isclose(aim[0], 4.0, abs_tol=1e-6)
    assert math.isclose(aim[1], 0.0, abs_tol=1e-6)
    assert math.isclose(t, 4.0 / 25.0, rel_tol=1e-3)


def test_crossing_target_leads_by_the_closed_form_distance():
    # Target 4m ahead, moving 2 m/s sideways (+y). Asserting against the
    # closed form, not against 2.0*t: aim[1] == target_vel[1]*(tau + t) is
    # the returned aim expression restated, so it holds for any t the
    # solve produces, right or wrong.
    aim, _ = solve_intercept((4.0, 0.0, 0.0), (0.0, 2.0, 0.0),
                             (0.0, 0.0, 0.0), tau=0.0, v_muzzle=V_MUZZLE)
    expected = _analytic_flight_time((4.0, 0.0, 0.0), (0.0, 2.0, 0.0), 0.0, V_MUZZLE)
    assert aim[1] > 0.0
    assert math.isclose(aim[1], 2.0 * expected, rel_tol=ITER3_REL_TOL)


def test_no_vertical_lead_from_horizontal_motion():
    # Elevated target (z=1.5) so a solve that zeroed or cross-mixed the z
    # channel fails here; with the old z=0 target, aim[2]==0 held either way.
    aim, _ = solve_intercept((4.0, 0.0, 1.5), (0.0, 2.0, 0.0),
                             (0.0, 0.0, 0.0), tau=0.1, v_muzzle=V_MUZZLE)
    assert math.isclose(aim[2], 1.5, abs_tol=1e-9)


def test_latency_extends_lead_by_tau_times_velocity():
    # tau shifts the aim point by exactly v*tau beyond the no-latency
    # lead, plus whatever the longer reach adds to the flight time.
    target_pos, target_vel = (4.0, 0.0, 0.0), (0.0, 2.0, 0.0)
    tau = 0.1
    aim_no_latency, _ = solve_intercept(target_pos, target_vel, (0.0, 0.0, 0.0),
                                        tau=0.0, v_muzzle=V_MUZZLE)
    aim_latency, _ = solve_intercept(target_pos, target_vel, (0.0, 0.0, 0.0),
                                     tau=tau, v_muzzle=V_MUZZLE)
    t0 = _analytic_flight_time(target_pos, target_vel, 0.0, V_MUZZLE)
    t_tau = _analytic_flight_time(target_pos, target_vel, tau, V_MUZZLE)
    assert math.isclose(aim_no_latency[1], 2.0 * t0, rel_tol=ITER3_REL_TOL)
    assert math.isclose(aim_latency[1], 2.0 * (tau + t_tau), rel_tol=ITER3_REL_TOL)


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
