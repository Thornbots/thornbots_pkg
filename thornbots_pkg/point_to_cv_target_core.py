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


QUARTER_TURN = math.pi / 2.0


def plan_shot(state, radius, z_offset, age_s, shooter_pos, v_muzzle, spinning, tick_s,
              gimbal_lag_s=0.05, firmware_latency_s=0.05, lead=True, iterations=3,
              shooter_vel=(0.0, 0.0, 0.0), chase_settle_s=None, chase_margin_s=0.0,
              accel=(0.0, 0.0, 0.0)):
    """
    Choose an aim point and fire delay against a TargetState armor model.

    state: [xc, yc, zc, vx, vy, vz, yaw, w] in odom at its stamp, age_s old;
    accel: the center's; radius, z_offset: per pair, as in TargetState. Each
    aim holds a tick_s, then the gimbal trails it by gimbal_lag_s; the aim
    targets the middle of that. The fire is timed over firmware_latency_s.
    Not spinning: lead the facing panel, fire now. Spinning: aim on the
    center->shooter line at the arriving pair, fire after the delay that
    lands it there if under tick_s, else None. With chase_settle_s: chase
    the facing panel, fire mid-hold if the panel a shot meets has faced us
    chase_settle_s and will for chase_margin_s more.
    lead=False: aim at the current estimate, fire now. Returns (aim_pos,
    delay_s or None).
    """
    xc, yc, zc, vx, vy, vz, yaw, w = (float(v) for v in state)
    ax, ay, az = (float(v) for v in accel)
    aim_horizon_s = age_s + gimbal_lag_s + tick_s / 2.0
    fire_horizon_s = age_s + firmware_latency_s

    def center(t):
        h = 0.5 * t * t
        return (xc + vx * t + ax * h, yc + vy * t + ay * h, zc + vz * t + az * h)

    def bearing(t):
        c = center(t)
        return math.atan2(shooter_pos[1] - c[1], shooter_pos[0] - c[0])

    def facing(t):
        return round((bearing(t) - (yaw + w * t)) / QUARTER_TURN) % 4

    def panel(k, t):
        c, yaw_k, r = center(t), yaw + w * t + k * QUARTER_TURN, radius[k % 2]
        return (c[0] + r * math.cos(yaw_k), c[1] + r * math.sin(yaw_k), c[2] + z_offset[k % 2])

    def on_line(r, dz):
        def path(t):
            c, b = center(t), bearing(t)
            return (c[0] + r * math.cos(b), c[1] + r * math.sin(b), c[2] + dz)
        return path

    def intercept(path):
        # Fixed point on the target's true path (curved by acceleration and
        # spin), not a straight-line extrapolation: t <- |path(h + t) -
        # muzzle(t)| / v_muzzle.
        t = 0.0
        for _ in range(max(1, iterations)):
            muzzle = [shooter_pos[i] + shooter_vel[i] * t for i in range(3)]
            t = math.dist(path(aim_horizon_s + t), muzzle) / v_muzzle if v_muzzle > 0.0 else 0.0
        return path(aim_horizon_s + t), t

    def facing_panel(t):
        return panel(facing(t), t)

    if not lead:
        return facing_panel(0.0), 0.0
    if not spinning:
        return intercept(facing_panel)[0], 0.0
    if chase_settle_s is not None:
        aim, t_flight = intercept(facing_panel)
        # Leave mid-hold of whichever aim is current then: that aim was
        # solved for this exit, so it is on the panel this shot meets. Skip
        # shots the gimbal is still jumping for, either side of a switch.
        delay = (aim_horizon_s - fire_horizon_s) % tick_s
        t_impact = fire_horizon_s + delay + t_flight
        k = facing(t_impact)
        theta = yaw + w * t_impact + k * QUARTER_TURN - bearing(t_impact)
        theta = (theta + QUARTER_TURN / 2.0) % QUARTER_TURN  # 0 as it starts facing, if w > 0
        since = theta / abs(w) if w > 0.0 else (QUARTER_TURN - theta) / abs(w)
        until = QUARTER_TURN / abs(w) - since
        return aim, (delay if since >= chase_settle_s and until >= chase_margin_s else None)

    _, t_flight = intercept(on_line(0.5 * (radius[0] + radius[1]), 0.0))
    if abs(w) < 1e-6:
        return intercept(on_line(radius[0], z_offset[0]))[0], None

    def to_alignment(t_impact):
        # Time from t_impact until a panel normal points along the line to
        # the shooter.
        phase = (yaw + w * t_impact - bearing(t_impact)) % QUARTER_TURN
        return ((QUARTER_TURN - phase) % QUARTER_TURN if w > 0.0 else phase) / abs(w)

    # Hold the pair until the last shot at it has left the muzzle, then
    # switch: its height is a step the gimbal settles in well under a
    # quarter turn, and a switch any earlier moves the gun under that shot.
    t_pair = age_s + t_flight
    k = facing(t_pair + to_alignment(t_pair))
    aim, _ = intercept(on_line(radius[k % 2], z_offset[k % 2]))
    delay = to_alignment(fire_horizon_s + t_flight)
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
