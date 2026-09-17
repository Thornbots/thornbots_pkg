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
Pure intercept-solve math for point_to_cv_target.py.

point_to_cv_target_core.py -- pure intercept-solve math for
point_to_cv_target.py (no rclpy import), unit-tested standalone in
test/test_point_to_cv_target.py. See README.md's
### point_to_cv_target.py Notes for the design rationale.
"""
import math


def solve_intercept(target_pos, target_vel, shooter_pos, tau, v_muzzle,
                    iterations=3, shooter_vel=(0.0, 0.0, 0.0)):
    """
    Solve for time-of-flight intercept via fixed-point iteration.

    Fixed-point time-of-flight solve, no gravity/drag/elevation (Type-C
    owns those): t <- |p + v*(tau+t) - (shooter + shooter_vel*t)| / v_muzzle.

    target_pos, target_vel, shooter_pos, shooter_vel: (x,y,z) tuples, same
    frame (odom -- an inertial-ish frame the constant-velocity model holds
    in). shooter_vel is the sentry's own chassis velocity (RobotPose
    vel_x/vel_y rotated into odom, z=0) -- a small second-order correction
    since the sentry itself keeps moving during the flight; defaults to
    stationary. Filter in odom, emit in root.
    tau: total pipeline+firmware latency already elapsed/expected (s).
    v_muzzle: projectile speed (m/s).
    iterations: fixed-point iteration count (2-3 converges in practice for
    these ranges/speeds).

    Returns (aim_pos, t_flight): aim_pos is the predicted intercept point
    (target_pos + target_vel*(tau+t_flight)), t_flight is the solved flight
    time, used only to size the prediction horizon -- Type-C computes its
    own real ballistic flight time.
    """
    t = 0.0
    px, py, pz = target_pos
    vx, vy, vz = target_vel
    sx, sy, sz = shooter_pos
    svx, svy, svz = shooter_vel
    for _ in range(max(1, iterations)):
        dt = tau + t
        ax, ay, az = px + vx * dt, py + vy * dt, pz + vz * dt
        bx, by, bz = sx + svx * t, sy + svy * t, sz + svz * t
        dist = math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)
        t = dist / v_muzzle if v_muzzle > 0.0 else 0.0
    dt = tau + t
    aim_pos = (px + vx * dt, py + vy * dt, pz + vz * dt)
    return aim_pos, t


def plan_shot(state, other_r, horizon_s, shooter_pos, v_muzzle, spinning,
              tick_s, lead=True, iterations=3, shooter_vel=(0.0, 0.0, 0.0)):
    """
    Choose an aim point and fire delay against target_tracker's armor model.

    state: [xc, yc, zc, vx, vy, vz, yaw, w, r] in odom at its stamp;
    horizon_s: stamp to muzzle exit for a shot fired now. Not spinning: lead
    the tracked panel, fire now. Spinning: lead a point on the
    centre->shooter line half a tick ahead, fire after the delay (< tick_s)
    that lands on the next quarter-turn alignment, else None. lead=False
    aims at the current estimate. Returns (aim_pos, delay_s or None).
    """
    xc, yc, zc, vx, vy, vz, yaw, w, r = (float(v) for v in state)
    if not lead:
        horizon_s, tick_s = 0.0, 0.0
    ahead = horizon_s + (tick_s / 2.0 if spinning else 0.0)
    cx, cy, cz = xc + vx * ahead, yc + vy * ahead, zc + vz * ahead
    yaw_h = yaw + w * ahead
    if spinning:
        bearing = math.atan2(shooter_pos[1] - cy, shooter_pos[0] - cx)
        rf = 0.5 * (r + other_r)
        pos = (cx + rf * math.cos(bearing), cy + rf * math.sin(bearing), cz)
        vel = (vx, vy, vz)
    else:
        # The tracked panel, not the best-facing one: without spin, yaw and
        # radius are unobservable and drift, so only the seen panel is solid.
        pos = (cx + r * math.cos(yaw_h), cy + r * math.sin(yaw_h), cz)
        vel = (vx - r * w * math.sin(yaw_h), vy + r * w * math.cos(yaw_h), vz)

    if not lead:
        return pos, 0.0
    aim, t_flight = solve_intercept(pos, vel, shooter_pos, 0.0, v_muzzle,
                                    iterations=iterations, shooter_vel=shooter_vel)
    if not spinning:
        return aim, 0.0
    if abs(w) < 1e-6:
        return aim, None

    # Quarter-turn phase of the panels against the shooter bearing at the
    # impact of a shot fired now; a hit wants it at 0.
    t_impact = horizon_s + t_flight
    ix, iy = xc + vx * t_impact, yc + vy * t_impact
    bearing_i = math.atan2(shooter_pos[1] - iy, shooter_pos[0] - ix)
    phase = (yaw + w * t_impact - bearing_i) % (math.pi / 2.0)
    to_go = (math.pi / 2.0 - phase) % (math.pi / 2.0) if w > 0.0 else phase
    delay = to_go / abs(w)
    return aim, (delay if delay < tick_s else None)


class LatencyStat:
    """
    Running mean/count of now-detection_stamp latency samples (seconds).

    The repo's first real latency measurement.
    """

    def __init__(self):
        self.count = 0
        self.mean = 0.0

    def add(self, sample_s):
        self.count += 1
        self.mean += (sample_s - self.mean) / self.count
