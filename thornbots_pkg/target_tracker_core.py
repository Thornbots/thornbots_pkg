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
Pure numpy armor-model EKF for target_tracker.py (no rclpy import).

State [xc, yc, zc, vx, vy, vz, yaw, w, r, dz]: chassis centre and velocity
in odom, the yaw of the tracked panel's outward normal, spin rate (rad/s),
that panel's centre-to-panel radius, and its pair's height above the centre
(the other pair sits at -dz). Measurement is one panel position;
h = centre + r * (cos yaw, sin yaw, 0) + (0, 0, dz). The other pair's radius
is kept outside the state and swapped in on an odd handoff, which also flips
dz. See README.md's ### target_tracker.py Notes.
"""
import math

import numpy as np

N_STATE = 10
DZ_MAX = 0.15  # m, clamp on the pair height offset


def _as_cov(pos_var):
    """Accept a scalar variance (isotropic) or a 3x3 covariance."""
    return np.eye(3) * pos_var if np.ndim(pos_var) == 0 else np.asarray(pos_var)


def ray_covariance(panel_pos, camera_pos, depth_std, lateral_std):
    """
    Measurement covariance: depth_std along the camera->panel ray, lateral_std across it.

    Depth error grows with range squared while bearing error stays near one
    pixel, so an isotropic R buries a spinning panel's sideways arc.
    """
    ray = np.asarray(panel_pos, dtype=float) - np.asarray(camera_pos, dtype=float)
    u = ray / (np.linalg.norm(ray) + 1e-9)
    return lateral_std ** 2 * np.eye(3) + (depth_std ** 2 - lateral_std ** 2) * np.outer(u, u)


QUARTER_TURN = math.pi / 2.0
BACK_FACING_COS = -0.3  # association skips panels facing further away than this


def panel_positions(state, other_r):
    """Return [(k, yaw_k, position)] for all 4 panels, k=0 the tracked one."""
    xc, yc, zc = state[0], state[1], state[2]
    yaw, r, dz = state[6], state[8], state[9]
    out = []
    for k in range(4):
        yaw_k = yaw + k * QUARTER_TURN
        r_k, dz_k = (r, dz) if k % 2 == 0 else (other_r, -dz)
        out.append((k, yaw_k, np.array([xc + r_k * math.cos(yaw_k),
                                        yc + r_k * math.sin(yaw_k), zc + dz_k])))
    return out


class ArmorEKF:
    """
    Constant-velocity centre plus constant-rate spin, position-only updates.

    q_accel (m/s^2) and q_yaw_accel (rad/s^2) drive white-noise-acceleration
    process noise; q_radius and q_dz (m/sqrt(s)) let r and dz drift. r is
    clamped to [r_min, r_max] and dz to +-DZ_MAX after every update.
    """

    def __init__(self, panel_pos, camera_pos, t_sec, pos_var, radius,
                 q_accel, q_yaw_accel, q_radius, r_min=0.18, r_max=0.45,
                 spin_prior=0.0, spin_prior_std=8.0, q_dz=0.005, dz_prior_std=0.05):
        self.q_accel = q_accel
        self.q_yaw_accel = q_yaw_accel
        self.q_radius = q_radius
        self.q_dz = q_dz
        self.dz_prior_std = dz_prior_std
        self.r_min = r_min
        self.r_max = r_max
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, radius, 0.0])
        self.P = np.eye(N_STATE)
        self.P[9, 9] = dz_prior_std ** 2
        self.other_r = radius
        self.t_sec = t_sec
        self.n_outliers = 0
        self.last_nis = 0.0
        self.spin_prior = spin_prior
        self.spin_prior_std = spin_prior_std
        self.initial_radius = radius
        self.reacquire(panel_pos, camera_pos, t_sec, pos_var, keep_spin=False)

    def reacquire(self, panel_pos, camera_pos, t_sec, pos_var, keep_spin=True):
        """
        Re-seed centre, velocity and yaw from one panel, assumed to face the camera.

        keep_spin keeps w, both radii and dz (with their variance), since a
        target that jinks rarely changes its spin in the same instant.
        """
        pos_var = _as_cov(pos_var)
        r, dz = self.state[8], self.state[9]
        yaw = math.atan2(camera_pos[1] - panel_pos[1], camera_pos[0] - panel_pos[0])
        w, w_var, r_var, dz_var = self.state[7], self.P[7, 7], self.P[8, 8], self.P[9, 9]
        if not keep_spin:
            w, w_var, r_var = self.spin_prior, self.spin_prior_std ** 2, 0.05 ** 2
            dz, dz_var = 0.0, self.dz_prior_std ** 2
        self.state = np.array([
            panel_pos[0] - r * math.cos(yaw), panel_pos[1] - r * math.sin(yaw),
            panel_pos[2] - dz, 0.0, 0.0, 0.0, yaw, w, r, dz])
        self.P = np.diag([0.0, 0.0, 0.0, 4.0, 4.0, 0.25, 0.5 ** 2, w_var, r_var, dz_var])
        self.P[:3, :3] = pos_var + np.diag([0.01, 0.01, 0.0])
        self.t_sec = t_sec
        self.n_outliers = 0

    def _transition(self, dt):
        F = np.eye(N_STATE)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        F[6, 7] = dt
        Q = np.zeros((N_STATE, N_STATE))
        block = np.array([[dt ** 4 / 4.0, dt ** 3 / 2.0],
                          [dt ** 3 / 2.0, dt ** 2]])
        for i in range(3):
            Q[np.ix_([i, i + 3], [i, i + 3])] = block * self.q_accel ** 2
        Q[np.ix_([6, 7], [6, 7])] = block * self.q_yaw_accel ** 2
        Q[8, 8] = dt * self.q_radius ** 2
        Q[9, 9] = dt * self.q_dz ** 2
        return F, Q

    def predicted(self, t_sec):
        """Return (state, P) extrapolated to t_sec, without mutating the filter."""
        dt = t_sec - self.t_sec
        if dt <= 0.0:
            return self.state.copy(), self.P.copy()
        F, Q = self._transition(dt)
        return F @ self.state, F @ self.P @ F.T + Q

    def predict(self, t_sec):
        self.state, self.P = self.predicted(t_sec)
        self.t_sec = max(self.t_sec, t_sec)

    def associate(self, panel_pos, camera_pos):
        """
        Re-label the state onto the panel nearest panel_pos; return (k, distance).

        Panels facing more than ~107 deg away from the camera are skipped.
        A cut at 90 deg flipped edge-on panels in and out with a 10cm camera
        shift and mis-assigned handoffs for seconds (see README.md). k != 0
        is a handoff: yaw steps by k quarter turns, and an odd k swaps r with
        other_r and flips dz. Call after predict(), before update().
        """
        best = None
        for k, yaw_k, pos in panel_positions(self.state, self.other_r):
            to_cam = math.atan2(camera_pos[1] - pos[1], camera_pos[0] - pos[0])
            if math.cos(yaw_k - to_cam) <= BACK_FACING_COS:
                continue
            d = float(np.linalg.norm(pos - np.asarray(panel_pos)))
            if best is None or d < best[1]:
                best = (k, d)
        if best is None:
            return None, float('inf')
        k = best[0]
        if k:
            self.state[6] += k * QUARTER_TURN
            if k % 2:
                self.state[8], self.other_r = self.other_r, self.state[8]
                self.state[9] = -self.state[9]
                self.P[9, :] *= -1.0
                self.P[:, 9] *= -1.0  # P[9, 9] flips twice, staying put
        return best

    def step(self, panel_pos, camera_pos, t_sec, pos_var, gate_nis=16.3, max_outliers=3):
        """
        Predict, associate and update on one detection; return 'update', 'outlier' or 'reacquire'.

        gate_nis is the chi-square(3) bound on the normalised innovation
        (16.3 = 99.9%). Outliers are skipped; max_outliers in a row re-seed
        the position via reacquire(), keeping the spin estimate.
        """
        self.predict(t_sec)
        self.associate(panel_pos, camera_pos)
        self.last_nis = self.nis(panel_pos, pos_var)
        if self.last_nis > gate_nis:
            self.n_outliers += 1
            if self.n_outliers < max_outliers:
                return 'outlier'
            self.reacquire(panel_pos, camera_pos, t_sec, pos_var)
            return 'reacquire'
        self.n_outliers = 0
        self.update(panel_pos, pos_var)
        return 'update'

    def _h_and_jacobian(self):
        xc, yc, zc = self.state[0], self.state[1], self.state[2]
        yaw, r, dz = self.state[6], self.state[8], self.state[9]
        c, s = math.cos(yaw), math.sin(yaw)
        h = np.array([xc + r * c, yc + r * s, zc + dz])
        H = np.zeros((3, N_STATE))
        H[0, 0] = H[1, 1] = H[2, 2] = H[2, 9] = 1.0
        H[0, 6], H[0, 8] = -r * s, c
        H[1, 6], H[1, 8] = r * c, s
        return h, H

    def nis(self, panel_pos, pos_var):
        h, H = self._h_and_jacobian()
        y = np.asarray(panel_pos) - h
        S = H @ self.P @ H.T + _as_cov(pos_var)
        return float(y @ np.linalg.solve(S, y))

    def update(self, panel_pos, pos_var):
        h, H = self._h_and_jacobian()
        R = _as_cov(pos_var)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ (np.asarray(panel_pos) - h)
        IKH = np.eye(N_STATE) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T  # Joseph form
        self.state[8] = min(max(self.state[8], self.r_min), self.r_max)
        self.state[9] = min(max(self.state[9], -DZ_MAX), DZ_MAX)


class ArmorTracker:
    """
    Bank of ArmorEKFs seeded at different spin rates; the lowest-NIS one leads.

    One EKF started at w=0 locks into a wrong spin (often w~0 with a
    collapsed radius) if its first second of data is poor, e.g. while the
    head slews in. Every hypothesis sees every panel; score is an EWMA of
    its NIS (alpha). A hypothesis scoring worse than the leader by
    reseed_margin for reseed_after_s is re-seeded from the leader's centre
    and yaw, with its own spin prior and fresh radii. See README.md.
    """

    def __init__(self, panel_pos, camera_pos, t_sec, pos_var, radius, q_accel,
                 q_yaw_accel, q_radius, spin_priors=(0.0, 7.0, -7.0, 13.0, -13.0),
                 prior_std=3.0, alpha=0.03, reseed_margin=3.0, reseed_after_s=1.0,
                 switch_margin=1.0, switch_after_s=0.5):
        self.filters = [ArmorEKF(panel_pos, camera_pos, t_sec, pos_var, radius, q_accel,
                                 q_yaw_accel, q_radius, spin_prior=w, spin_prior_std=prior_std)
                        for w in spin_priors]
        self.scores = [3.0] * len(self.filters)  # NIS mean for 3 dof
        self.worse_since = [None] * len(self.filters)
        self.alpha = alpha
        self.reseed_margin = reseed_margin
        self.reseed_after_s = reseed_after_s
        self.lead = 0
        self.switch_margin = switch_margin
        self.switch_after_s = switch_after_s
        self._challenger = None  # (index, since t_sec)

    @property
    def best(self):
        return self.filters[self.lead]

    @property
    def state(self):
        return self.best.state

    @property
    def other_r(self):
        return self.best.other_r

    def predicted(self, t_sec):
        return self.best.predicted(t_sec)

    def step(self, panel_pos, camera_pos, t_sec, pos_var, gate_nis=16.3, max_outliers=3):
        """Advance every hypothesis on one panel; return the leading filter's status."""
        statuses = []
        for i, f in enumerate(self.filters):
            statuses.append(f.step(panel_pos, camera_pos, t_sec, pos_var, gate_nis, max_outliers))
            nis = min(f.last_nis, gate_nis)  # one wild sample shouldn't sink a filter
            self.scores[i] += self.alpha * (nis - self.scores[i])
        # The lead changes only after a challenger beats it by switch_margin
        # for switch_after_s: a noise burst briefly favours a collapsed-radius
        # wrong-sign hypothesis, which is least sensitive to it.
        best_i = int(np.argmin(self.scores))
        lead_score = self.scores[self.lead]
        if best_i == self.lead or self.scores[best_i] > lead_score - self.switch_margin:
            self._challenger = None
        elif self._challenger is None or self._challenger[0] != best_i:
            self._challenger = (best_i, t_sec)
        elif t_sec - self._challenger[1] >= self.switch_after_s:
            self.lead, self._challenger = best_i, None
        best, best_score = self.best, self.scores[self.lead]
        for i, f in enumerate(self.filters):
            if (i == self.lead or (self._challenger and self._challenger[0] == i)
                    or self.scores[i] < best_score + self.reseed_margin):
                self.worse_since[i] = None
                continue
            if self.worse_since[i] is None:
                self.worse_since[i] = t_sec
            elif t_sec - self.worse_since[i] >= self.reseed_after_s:
                f.state = best.state.copy()
                f.state[7] = f.spin_prior
                f.state[8] = f.other_r = f.initial_radius
                f.P = best.P.copy()
                f.P[7, :] = f.P[:, 7] = 0.0
                f.P[7, 7] = f.spin_prior_std ** 2
                f.P[8, :] = f.P[:, 8] = 0.0
                f.P[8, 8] = 0.05 ** 2
                f.t_sec = best.t_sec
                self.scores[i] = best_score + self.reseed_margin / 2.0
                self.worse_since[i] = None
        return statuses[self.lead]
