"""
point_to_cv_target_core.py -- pure intercept-solve math for
point_to_cv_target.py (no rclpy import), unit-tested standalone in
test/test_point_to_cv_target.py. See README.md's
### point_to_cv_target.py Notes for the design rationale.
"""
import math


def solve_intercept(target_pos, target_vel, shooter_pos, tau, v_muzzle,
                     iterations=3, shooter_vel=(0.0, 0.0, 0.0)):
    """Fixed-point time-of-flight solve, no gravity/drag/elevation (Type-C
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


class LatencyStat:
    """Running mean/count of now-detection_stamp latency samples (seconds).
    The repo's first real latency measurement."""

    def __init__(self):
        self.count = 0
        self.mean = 0.0

    def add(self, sample_s):
        self.count += 1
        self.mean += (sample_s - self.mean) / self.count
