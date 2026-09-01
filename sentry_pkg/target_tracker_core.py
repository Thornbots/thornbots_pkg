"""
target_tracker_core.py -- pure numpy logic for target_tracker.py (no rclpy
import) so it's unit-testable standalone, mirroring target_selector_core.py.
See sentry_pkg/README.md's ### target_tracker.py Notes.
"""
import math

import numpy as np


def corrected_centre(panel_center, radius_m):
    """Chassis-centre estimate: push panel_center further along the same
    camera-to-panel ray by radius_m, i.e. approximate the chassis centre as
    sitting directly behind the visible panel along the existing line of
    sight. Same frame as panel_center (camera or odom -- direction-only,
    doesn't care).

    This is deliberately NOT derived from PanelDetection.corners. Corners
    look like they'd give a real plane normal via one cross product, but
    they don't: roi_depth_node.cpp's deprojectDetection() deprojects all 4
    corners at one shared mean_depth_m (the "planar assumption" its own
    comment names), which makes every real detection's quad exactly
    fronto-parallel to the camera by construction -- the cross product of
    two edge vectors in that plane is always exactly the camera's boresight
    axis, carrying zero information about the panel's true tilt, and for
    an off-boresight panel that's a materially different (and wrong)
    direction than "back toward the camera along this panel's own bearing"
    (an earlier version of this function used the corner cross product and
    got exactly this wrong for off-axis panels). cv_target_emulator.py's
    corners *do* encode real tilt (built from the true canted right_dir/
    up_dir), so a corner-based estimate would work in sim and silently
    fail differently on hardware -- a sim/hardware divergence via geometry
    instead of frame_id. Real depth-based plane-fitting for a true normal
    is out of scope (only viable under ~2m, where depth noise is below the
    panel's tilt) -- this radial approximation is the honest fallback, not a
    placeholder for something better later."""
    norm = np.linalg.norm(panel_center)
    if norm < 1e-9:
        return np.array(panel_center, dtype=float)
    direction = np.array(panel_center) / norm
    return panel_center + radius_m * direction


class SpinDetector:
    """Estimates spin rate from class_id handoff timing (the visible panel
    id changing as the robot rotates). Coarse by design: a spinning
    Standard-class robot presents 4 panels 90 degrees apart, so handoffs
    are assumed to occur roughly once per quarter revolution -- this
    conflates true handoff period with quarter-revolution period and does
    not distinguish spin direction, but it's enough to decide the binary
    spin/no-spin branch, which is the only thing that gates behaviour (see
    target_tracker.py). spin_phase is a coarse re-derivation from elapsed
    time since the last handoff, not a real phase-locked estimate."""

    def __init__(self, handoff_timeout_s, min_handoffs, cv_max):
        self.handoff_timeout_s = handoff_timeout_s
        self.min_handoffs = min_handoffs
        self.cv_max = cv_max
        self.reset()

    def reset(self):
        self._last_class_id = None
        self._last_change_t = None
        self._intervals = []

    def update(self, t_sec, class_id):
        """Returns (spinning: bool, spin_hz: float, spin_phase: float)."""
        if self._last_class_id is None:
            self._last_class_id = class_id
            self._last_change_t = t_sec
            return False, 0.0, 0.0

        if class_id != self._last_class_id:
            interval = t_sec - self._last_change_t
            if interval > 0.0:
                self._intervals.append(interval)
                self._intervals = self._intervals[-8:]
            self._last_class_id = class_id
            self._last_change_t = t_sec

        since_last = t_sec - self._last_change_t
        if since_last > self.handoff_timeout_s:
            self._intervals = []
            return False, 0.0, 0.0

        if len(self._intervals) < self.min_handoffs:
            return False, 0.0, 0.0

        mean_interval = sum(self._intervals) / len(self._intervals)
        if mean_interval <= 0.0:
            return False, 0.0, 0.0
        variance = sum((i - mean_interval) ** 2 for i in self._intervals) / len(self._intervals)
        cv = math.sqrt(variance) / mean_interval
        if cv > self.cv_max:
            return False, 0.0, 0.0

        period_s = 4.0 * mean_interval  # 4 panels/revolution, see class docstring
        spin_hz = 1.0 / period_s
        spin_phase = (2.0 * math.pi * (since_last / period_s)) % (2.0 * math.pi)
        return True, spin_hz, spin_phase


class KalmanFilter6D:
    """6-state constant-velocity Kalman filter: [x,y,z,vx,vy,vz], 3-D
    position measurements only (H picks out x,y,z). Isotropic per-axis
    process/measurement noise -- no cross-axis coupling."""

    def __init__(self, initial_pos, t_sec, pos_var):
        self.state = np.array([initial_pos[0], initial_pos[1], initial_pos[2],
                                0.0, 0.0, 0.0])
        # Large initial velocity uncertainty -- the first sample carries no
        # velocity information.
        self.P = np.diag([pos_var, pos_var, pos_var, 4.0, 4.0, 4.0])
        self._t_sec = t_sec

    def predict(self, t_sec, process_noise_accel):
        dt = t_sec - self._t_sec
        self._t_sec = t_sec
        if dt <= 0.0:
            return
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        q = process_noise_accel ** 2
        # Standard discrete white-noise-acceleration process noise per axis.
        Q_block = np.array([[dt ** 4 / 4.0, dt ** 3 / 2.0],
                             [dt ** 3 / 2.0, dt ** 2]]) * q
        Q = np.zeros((6, 6))
        for i in range(3):
            idx = [i, i + 3]
            Q[np.ix_(idx, idx)] = Q_block
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + Q

    def update(self, meas, pos_var):
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        R = np.eye(3) * pos_var
        y = np.array(meas) - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    @property
    def variance(self):
        return np.diag(self.P)
