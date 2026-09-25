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
Unit tests for point_to_cv_target_core.py's intercept solve and shot planner.

No rclpy, no ROS message packages. Run with
`python3 -m pytest test/test_point_to_cv_target.py`.
"""
import ast
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from thornbots_pkg.point_to_cv_target_core import (  # noqa: E402
    LatencyStat, plan_shot, solve_intercept,
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

ORIGIN = (0.0, 0.0, 0.0)

# (target_pos, target_vel, tau, shooter_pos, shooter_vel), spanning crossing /
# receding / closing / oblique motion at ARCC ranges and speeds. The last four
# rows put the shooter off the origin, including behind and above the target,
# so a solve that ignored shooter_pos (or folded it in with the wrong sign)
# can't pass.
#
# Every row is a STATIONARY shooter today. _analytic_flight_time already
# handles a moving one (w = v - shooter_vel), so closing gap 2's second half
# when the moving-robot test lands is adding rows here with a non-zero last
# column -- no new math, no changes to the four tests below.
GEOMETRIES = [
    ((4.0, 0.0, 0.0), (0.0, 2.0, 0.0), 0.0, ORIGIN, ORIGIN),
    ((4.0, 0.0, 0.0), (0.0, 2.0, 0.0), 0.12, ORIGIN, ORIGIN),
    ((8.0, 0.0, 0.0), (0.0, 4.0, 0.0), 0.0, ORIGIN, ORIGIN),
    ((8.0, 0.0, 0.0), (4.0, 0.0, 0.0), 0.0, ORIGIN, ORIGIN),   # receding head-on
    ((8.0, 0.0, 0.0), (-4.0, 0.0, 0.0), 0.0, ORIGIN, ORIGIN),  # closing head-on
    ((2.0, 0.0, 1.5), (0.0, 2.0, 0.0), 0.08, ORIGIN, ORIGIN),  # elevated target
    ((3.0, 4.0, 0.0), (1.0, -3.0, 0.5), 0.12, ORIGIN, ORIGIN),  # oblique
    # Shooter off-origin: same crossing geometry as row 1, shifted whole.
    ((6.0, 1.0, 0.0), (0.0, 2.0, 0.0), 0.0, (2.0, 1.0, 0.0), ORIGIN),
    # Shooter behind the target in +x: the range is 3m, not 11m, so a solve
    # that dropped shooter_pos would overshoot the flight time ~3.7x.
    ((8.0, 0.0, 0.0), (0.0, 3.0, 0.0), 0.1, (11.0, 0.0, 0.0), ORIGIN),
    # Turret above a low target -- exercises the z offset on its own.
    ((5.0, 0.0, 0.2), (0.0, 2.5, 0.0), 0.06, (0.0, 0.0, 1.2), ORIGIN),
    # Fully oblique: shooter off-axis in all three, 3-D target velocity.
    ((3.0, 4.0, 0.5), (1.0, -3.0, 0.5), 0.12, (-1.0, 1.5, 0.9), ORIGIN),
]

# No row above moves the shooter, so the chassis-velocity correction's
# CORRECTNESS is still untested; test_shooter_velocity_is_wired_into_the_solve
# only pins that the parameter reaches the math at all. See
# sim/CV_TEST_GAPS.md gap 2 -- deliberately open until the moving-robot test
# exists, not an oversight.


def _analytic_flight_time(target_pos, target_vel, tau, v_muzzle,
                          shooter_pos=(0.0, 0.0, 0.0),
                          shooter_vel=(0.0, 0.0, 0.0)):
    """
    Solve the same intercept in closed form, as truth for the iterative solve.

    The intercept condition |p + v*(tau + t) - (s + sv*t)| = v_muzzle*t is
    a quadratic in t: (|w|^2 - v_muzzle^2) t^2 + 2 (d.w) t + |d|^2 = 0,
    where d = p + v*tau - s and w = v - sv is the closing velocity.
    Returns the smallest positive root (the first time the shot can
    arrive); the linear branch covers a target closing at exactly
    v_muzzle. The shooter_vel term is carried here so a moving-shooter row
    in GEOMETRIES needs no new math, but no row exercises it yet.
    """
    d = [target_pos[i] + target_vel[i] * tau - shooter_pos[i] for i in range(3)]
    w = [target_vel[i] - shooter_vel[i] for i in range(3)]
    a = sum(x * x for x in w) - v_muzzle ** 2
    b = 2.0 * sum(d[i] * w[i] for i in range(3))
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
    for target_pos, target_vel, tau, shooter, shooter_vel in GEOMETRIES:
        aim, t = solve_intercept(target_pos, target_vel, shooter,
                                 tau=tau, v_muzzle=V_MUZZLE,
                                 shooter_vel=shooter_vel)
        # The shooter has moved by shooter_vel*t by the time the shot lands.
        muzzle_at_impact = [shooter[i] + shooter_vel[i] * t for i in range(3)]
        assert math.isclose(math.dist(aim, muzzle_at_impact), V_MUZZLE * t,
                            rel_tol=ITER3_REL_TOL), (
            target_pos, target_vel, tau, shooter, shooter_vel)


def test_flight_time_matches_closed_form():
    for target_pos, target_vel, tau, shooter, shooter_vel in GEOMETRIES:
        _, t = solve_intercept(target_pos, target_vel, shooter,
                               tau=tau, v_muzzle=V_MUZZLE,
                               shooter_vel=shooter_vel)
        expected = _analytic_flight_time(target_pos, target_vel, tau, V_MUZZLE,
                                         shooter, shooter_vel)
        assert math.isclose(t, expected, rel_tol=ITER3_REL_TOL), (
            target_pos, target_vel, tau, shooter, shooter_vel)


def test_aim_point_matches_closed_form_componentwise():
    # Per-axis, not just |aim|: dropping the z lead moves the norm by well
    # under ITER3_REL_TOL on a low target while putting the shot 15cm high.
    for target_pos, target_vel, tau, shooter, shooter_vel in GEOMETRIES:
        aim, _ = solve_intercept(target_pos, target_vel, shooter,
                                 tau=tau, v_muzzle=V_MUZZLE,
                                 shooter_vel=shooter_vel)
        t = _analytic_flight_time(target_pos, target_vel, tau, V_MUZZLE,
                                  shooter, shooter_vel)
        for axis in range(3):
            expected = target_pos[axis] + target_vel[axis] * (tau + t)
            assert math.isclose(aim[axis], expected, abs_tol=ITER3_AIM_TOL_M), (
                target_pos, target_vel, tau, shooter, shooter_vel, axis)


def test_iterating_to_convergence_reaches_the_closed_form():
    # Pins the docstring's "2-3 converges in practice" as a measurement:
    # the fixed point is the closed-form root, and the default count is
    # a truncation of it rather than a different answer.
    for target_pos, target_vel, tau, shooter, shooter_vel in GEOMETRIES:
        _, t = solve_intercept(target_pos, target_vel, shooter,
                               tau=tau, v_muzzle=V_MUZZLE, iterations=40,
                               shooter_vel=shooter_vel)
        expected = _analytic_flight_time(target_pos, target_vel, tau, V_MUZZLE,
                                         shooter, shooter_vel)
        assert math.isclose(t, expected, rel_tol=CONVERGED_REL_TOL), (
            target_pos, target_vel, tau, shooter, shooter_vel)


def test_shooter_offset_is_not_ignored():
    # Same target and velocity, shooter moved 4m closer along the line of
    # sight. Guards the specific failure the origin-only table couldn't
    # see: shooter_pos silently dropped, or added instead of subtracted.
    far, _ = solve_intercept((8.0, 0.0, 0.0), (0.0, 2.0, 0.0), ORIGIN,
                             tau=0.0, v_muzzle=V_MUZZLE)
    near, _ = solve_intercept((8.0, 0.0, 0.0), (0.0, 2.0, 0.0), (4.0, 0.0, 0.0),
                              tau=0.0, v_muzzle=V_MUZZLE)
    # Half the range, so half the flight time, so half the crossing lead.
    assert math.isclose(near[1], far[1] / 2.0, rel_tol=ITER3_REL_TOL)
    assert near[1] > 0.0


def test_shooter_velocity_is_wired_into_the_solve():
    """
    Pin that shooter_vel reaches the math, without checking the correction.

    A moving shooter must not produce the stationary answer -- that is
    what deleting the parameter would look like. This asserts only that
    the two differ and that the sign is the intuitive one (chasing the
    target shortens the closing distance, so the shot arrives sooner).
    Whether the magnitude is *right* is untested; see gap 2.
    """
    target_pos, target_vel, tau = (8.0, 0.0, 0.0), (0.0, 2.0, 0.0), 0.05
    stationary, t_stationary = solve_intercept(
        target_pos, target_vel, ORIGIN, tau=tau, v_muzzle=V_MUZZLE)
    chasing, t_chasing = solve_intercept(
        target_pos, target_vel, ORIGIN, tau=tau, v_muzzle=V_MUZZLE,
        shooter_vel=(5.0, 0.0, 0.0))
    assert t_chasing < t_stationary
    assert chasing[1] < stationary[1]


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


# ── plan_shot ─────────────────────────────────────────────────────────────

SHOOTER = (0.0, 0.0, 0.4)
TICK_S = 1.0 / 30.0
RADII = (0.30, 0.24)
FLAT = (0.0, 0.0)
STAGGER = (0.045, -0.045)


def _armor(center=(3.0, 0.0, 0.3), vel=(0.0, 0.0, 0.0), yaw=math.pi, w=0.0):
    return (*center, *vel, yaw, w)


def _plan(state, horizon, spinning, z_offset=FLAT, **kw):
    # Zero age, and a gimbal lag that puts the aim horizon (lag + half a
    # tick) on the fire horizon: horizon is both.
    return plan_shot(state, RADII, z_offset, 0.0, SHOOTER, V_MUZZLE, spinning, TICK_S,
                     gimbal_lag_s=horizon - TICK_S / 2.0, firmware_latency_s=horizon, **kw)


def _flight(aim):
    return math.dist(aim, SHOOTER) / V_MUZZLE


def test_plan_non_spinning_stationary_aims_at_the_facing_panel_now():
    aim, delay = _plan(_armor(), 0.1, False)
    assert delay == 0.0
    assert math.dist(aim, (2.7, 0.0, 0.3)) < 1e-9


def test_plan_non_spinning_picks_the_facing_panel_and_its_pair():
    # Panel 0 faces away; panel 2 (same pair) faces the shooter, panel 1 or 3
    # (other pair) would at a quarter turn.
    aim, _ = _plan(_armor(yaw=0.0), 0.1, False, z_offset=STAGGER)
    assert math.dist(aim, (2.7, 0.0, 0.345)) < 1e-9
    aim, _ = _plan(_armor(yaw=math.pi / 2.0), 0.1, False, z_offset=STAGGER)
    assert math.dist(aim, (2.76, 0.0, 0.255)) < 1e-9


def test_plan_non_spinning_crossing_panel_meets_the_intercept_condition():
    horizon = 0.12
    aim, _ = _plan(_armor(vel=(0.0, 2.0, 0.0)), horizon, False, iterations=50)
    t = _analytic_flight_time((2.7, 0.0, 0.3), (0.0, 2.0, 0.0), horizon, V_MUZZLE,
                              shooter_pos=SHOOTER)
    assert math.isclose(aim[1], 2.0 * (horizon + t), rel_tol=1e-6)
    assert math.isclose(math.dist(aim, SHOOTER), V_MUZZLE * t, rel_tol=1e-6)


def test_plan_non_spinning_leads_the_tangential_panel_velocity():
    # Panel facing the shooter on a slowly turning chassis moves sideways at
    # r*w even with a still center.
    aim, _ = _plan(_armor(w=1.0), 0.1, False)
    assert aim[1] < -0.02  # yaw pi, w > 0: the panel sweeps toward -y


def _alignment_error(state, horizon, delay):
    aim, _ = _plan(state, horizon, True)
    xc, yc, _, _, _, _, yaw, w = state
    t_impact = horizon + _flight(aim) + delay
    bearing = math.atan2(SHOOTER[1] - yc, SHOOTER[0] - xc)
    phase = (yaw + w * t_impact - bearing) % (math.pi / 2.0)
    return min(phase, math.pi / 2.0 - phase)


def test_plan_spinning_delay_lands_a_panel_square_to_the_shooter():
    w = 2.0 * math.pi * 1.5
    fired = 0
    for i in range(40):
        yaw = math.pi + i * (math.pi / 2.0) / 40.0
        for spin in (w, -w):
            state = _armor(yaw=yaw, w=spin)
            _, delay = _plan(state, 0.08, True)
            if delay is None:
                continue
            fired += 1
            assert 0.0 <= delay < TICK_S
            assert _alignment_error(state, 0.08, delay) < 0.02  # rad; flight-time residual
    # A tick-long window catches tick*|w| of each quarter turn, both directions.
    expected = 2 * 40 * (TICK_S * w) / (math.pi / 2.0)
    assert abs(fired - expected) <= 4


def test_plan_spinning_aims_on_the_center_to_shooter_line():
    aim, _ = _plan(_armor(center=(3.0, 1.0, 0.3), w=9.0), 0.1, True)
    to_shooter = math.atan2(SHOOTER[1] - 1.0, SHOOTER[0] - 3.0)
    assert math.isclose(math.atan2(aim[1] - 1.0, aim[0] - 3.0), to_shooter, abs_tol=1e-9)
    assert math.hypot(aim[0] - 3.0, aim[1] - 1.0) in (
        pytest.approx(RADII[0]), pytest.approx(RADII[1]))


def test_plan_spinning_aims_at_the_arriving_pairs_radius_and_height():
    # Sweep the phase: whichever pair the delay lands on, the aim uses its
    # radius and height, and both pairs come up.
    w = 9.0
    seen = set()
    for i in range(160):
        state = _armor(yaw=math.pi + i * (2.0 * math.pi) / 160.0, w=w)
        aim, delay = _plan(state, 0.08, True, z_offset=STAGGER)
        if delay is None:
            continue
        t_impact = 0.08 + _flight(aim) + delay
        k = round((math.pi - (state[6] + w * t_impact)) / (math.pi / 2.0)) % 4
        assert math.isclose(math.hypot(aim[0] - 3.0, aim[1]), RADII[k % 2], abs_tol=1e-9)
        assert math.isclose(aim[2], 0.3 + STAGGER[k % 2], abs_tol=1e-9)
        seen.add(k % 2)
    assert seen == {0, 1}


def test_plan_aim_horizon_sets_the_lead_and_fire_horizon_the_timing():
    state = _armor(vel=(0.0, 2.0, 0.0), w=9.0)
    # Equal radii, so the pair each horizon picks can't move the aim.
    near, d_near = plan_shot(state, (0.27, 0.27), FLAT, 0.0, SHOOTER, V_MUZZLE, True,
                             TICK_S, gimbal_lag_s=0.03 - TICK_S / 2.0,
                             firmware_latency_s=0.08)
    far, d_far = plan_shot(state, (0.27, 0.27), FLAT, 0.0, SHOOTER, V_MUZZLE, True,
                           TICK_S, gimbal_lag_s=0.08 - TICK_S / 2.0,
                           firmware_latency_s=0.08)
    # The line to the shooter turns a little as the center moves, hence 1 cm.
    assert math.isclose(far[1] - near[1], 2.0 * 0.05, abs_tol=0.01)
    assert d_near == d_far


def test_plan_chase_aims_at_the_facing_panel_and_fires_once_settled():
    w = 9.0
    quarter = (math.pi / 2.0) / w
    fired = 0
    for i in range(160):
        state = _armor(yaw=math.pi + i * (2.0 * math.pi) / 160.0, w=w)
        aim, delay = plan_shot(state, RADII, STAGGER, 0.0, SHOOTER, V_MUZZLE, True, TICK_S,
                               gimbal_lag_s=0.02 - TICK_S / 2.0, firmware_latency_s=0.05,
                               chase_settle_s=0.3 * quarter, chase_margin_s=0.1 * quarter)
        if delay is None:
            continue
        fired += 1
        # It leaves mid-hold: aim horizon minus fire horizon, mod a tick.
        assert math.isclose(delay, (0.02 - 0.05) % TICK_S)
        # A fired shot's aim is where a panel facing the shooter (within
        # 45 deg) is at impact, on its circle.
        t_impact = 0.02 + _flight(aim)  # the aim horizon; a mid-hold exit meets it
        miss, off = min(
            (math.dist(aim, (3.0 + RADII[k % 2] * math.cos(yaw_k),
                             RADII[k % 2] * math.sin(yaw_k), 0.3 + STAGGER[k % 2])),
             abs((yaw_k - math.pi + math.pi) % (2.0 * math.pi) - math.pi))
            for k in range(4) for yaw_k in [state[6] + w * t_impact + k * math.pi / 2.0])
        assert miss < 0.01
        assert off < math.radians(45.0)
    # Fires on the middle 60% of each panel's facing window.
    assert abs(fired - 0.6 * 160) <= 4


def test_plan_extrapolates_with_acceleration():
    # A target braking at 6 m/s^2 from 2 m/s: the aim is on the parabola at
    # impact, and the intercept condition holds on it.
    horizon = 0.1
    aim, _ = _plan(_armor(vel=(0.0, 2.0, 0.0)), horizon, False, iterations=50,
                   accel=(0.0, -6.0, 0.0))
    t = horizon + _flight(aim)
    assert math.isclose(aim[1], 2.0 * t - 3.0 * t * t, abs_tol=1e-9)
    assert math.isclose(aim[0], 2.7, abs_tol=1e-9)
    straight, _ = _plan(_armor(vel=(0.0, 2.0, 0.0)), horizon, False, iterations=50)
    assert straight[1] - aim[1] > 0.05


def test_plan_without_lead_aims_at_the_current_estimate_and_fires_now():
    state = _armor(vel=(0.0, 4.0, 0.0), w=9.0)
    aim, delay = _plan(state, 0.3, True, lead=False)
    assert delay == 0.0
    assert abs(aim[1]) < 1e-9


# ── node contract ─────────────────────────────────────────────────────────

NODE_SRC = os.path.join(os.path.dirname(__file__), '..', 'thornbots_pkg',
                        'point_to_cv_target.py')


def test_node_subscribes_to_target_state_and_robot_pose_only():
    # The CV split's seam (CV_SPLIT_PLAN.md 1.0): Part 1 aims from
    # TargetState alone, so a truth publisher can replace the whole of Part 2.
    tree = ast.parse(open(NODE_SRC).read())
    subscribed = [call.args[0].id for call in ast.walk(tree)
                  if isinstance(call, ast.Call)
                  and getattr(call.func, 'attr', None) == 'create_subscription']
    assert sorted(subscribed) == ['RobotPose', 'TargetState']
