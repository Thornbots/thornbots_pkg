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

State [pos, vel, acc, yaw, w, r, dz]: chassis centre position, velocity and
acceleration (3-vectors in odom, indexed by POS, VEL, ACC), the yaw of the
tracked panel's outward normal, spin rate (rad/s), that panel's
centre-to-panel radius, and its pair's height above the centre (the other
pair sits at -dz). Acceleration is a Singer model: it decays over
accel_tau_s, driven by white jerk. Measurement is one panel position;
h = pos + r * (cos yaw, sin yaw, 0) + (0, 0, dz). The other pair's radius
is kept outside the state and swapped in on an odd handoff, which also flips
dz. See README.md's ### target_tracker.py Notes.
"""
import math

import numpy as np

POS, VEL, ACC = slice(0, 3), slice(3, 6), slice(6, 9)
YAW, W, R, DZ = 9, 10, 11, 12
N_STATE = 13
DZ_MAX = 0.15  # m, clamp on the pair height offset
I3 = np.eye(3)


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


def _offset(yaw, r, dz):
    """Panel position relative to the centre."""
    return np.array([r * math.cos(yaw), r * math.sin(yaw), dz])


def panel_positions(state, other_r):
    """Return [(k, yaw_k, position)] for all 4 panels, k=0 the tracked one."""
    out = []
    for k in range(4):
        yaw_k = state[YAW] + k * QUARTER_TURN
        r_k, dz_k = (state[R], state[DZ]) if k % 2 == 0 else (other_r, -state[DZ])
        out.append((k, yaw_k, state[POS] + _offset(yaw_k, r_k, dz_k)))
    return out


class ArmorEKF:
    """
    Manoeuvring centre plus constant-rate spin, position-only updates.

    q_accel (m/s^2) is white-noise acceleration on the centre, q_jerk
    (m/s^3) drives the acceleration states (0 pins them at 0, a
    constant-velocity filter), q_yaw_accel (rad/s^2) the spin rate; q_radius
    and q_dz (m/sqrt(s)) let r and dz drift. r is clamped to [r_min, r_max]
    and dz to +-DZ_MAX after every update. still=True pins velocity,
    acceleration and spin at exactly 0; the centre and yaw random-walk at
    q_still_pos (m/sqrt(s)) and q_still_yaw (rad/sqrt(s)) instead.
    """

    def __init__(self, panel_pos, camera_pos, t_sec, pos_var, radius,
                 q_accel, q_yaw_accel, q_radius, r_min=0.18, r_max=0.45,
                 spin_prior=0.0, spin_prior_std=8.0, q_dz=0.005, dz_prior_std=0.05,
                 q_jerk=0.0, accel_tau_s=0.5, accel_prior_std=6.0, still=False,
                 q_still_pos=0.02, q_still_yaw=0.05):
        self.still = still
        self.q_still_pos = q_still_pos
        self.q_still_yaw = q_still_yaw
        self.q_accel = q_accel
        self.q_jerk = q_jerk
        self.accel_tau_s = accel_tau_s
        self.accel_prior_std = accel_prior_std if q_jerk > 0.0 else 0.0
        self.q_yaw_accel = q_yaw_accel
        self.q_radius = q_radius
        self.q_dz = q_dz
        self.dz_prior_std = dz_prior_std
        self.r_min = r_min
        self.r_max = r_max
        self.state = np.zeros(N_STATE)
        self.state[R] = radius
        self.P = np.eye(N_STATE)
        self.P[DZ, DZ] = dz_prior_std ** 2
        self.other_r = radius
        self.t_sec = t_sec
        self.n_outliers = 0
        self.last_nis = 0.0
        self.last_logdet = 0.0  # log det of the last innovation covariance
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
        panel_pos = np.asarray(panel_pos, dtype=float)
        r, dz = self.state[R], self.state[DZ]
        yaw = math.atan2(camera_pos[1] - panel_pos[1], camera_pos[0] - panel_pos[0])
        w, w_var = self.state[W], self.P[W, W]
        r_var, dz_var = self.P[R, R], self.P[DZ, DZ]
        if not keep_spin:
            w, w_var, r_var = self.spin_prior, self.spin_prior_std ** 2, 0.05 ** 2
            dz, dz_var = 0.0, self.dz_prior_std ** 2
        self.state = np.zeros(N_STATE)
        self.state[POS] = panel_pos - _offset(yaw, r, dz)
        self.state[YAW], self.state[W], self.state[R], self.state[DZ] = yaw, w, r, dz
        self.P = np.zeros((N_STATE, N_STATE))
        self.P[POS, POS] = _as_cov(pos_var) + np.diag([0.01, 0.01, 0.0])
        self.P[VEL, VEL] = np.diag([4.0, 4.0, 0.25])
        self.P[ACC, ACC] = self.accel_prior_std ** 2 * I3
        self.P[YAW, YAW], self.P[W, W] = 0.5 ** 2, w_var
        self.P[R, R], self.P[DZ, DZ] = r_var, dz_var
        self.t_sec = t_sec
        self.n_outliers = 0
        self.pin_still()

    def pin_still(self):
        """On a still filter, zero velocity, acceleration and spin and their covariance."""
        if not self.still:
            return
        for idx in (VEL, ACC, W):
            self.state[idx] = 0.0
            self.P[idx, :] = 0.0
            self.P[:, idx] = 0.0

    def _transition(self, dt):
        F = np.eye(N_STATE)
        F[POS, VEL] = dt * I3
        F[YAW, W] = dt
        Q = np.zeros((N_STATE, N_STATE))
        if self.still:
            Q[POS, POS] = dt * self.q_still_pos ** 2 * I3
            Q[YAW, YAW] = dt * self.q_still_yaw ** 2
            Q[R, R] = dt * self.q_radius ** 2
            Q[DZ, DZ] = dt * self.q_dz ** 2
            return F, Q
        block = np.array([[dt ** 4 / 4.0, dt ** 3 / 2.0],
                          [dt ** 3 / 2.0, dt ** 2]])
        Q[POS, POS] = block[0, 0] * self.q_accel ** 2 * I3
        Q[POS, VEL] = Q[VEL, POS] = block[0, 1] * self.q_accel ** 2 * I3
        Q[VEL, VEL] = block[1, 1] * self.q_accel ** 2 * I3
        Q[np.ix_([YAW, W], [YAW, W])] = block * self.q_yaw_accel ** 2
        Q[R, R] = dt * self.q_radius ** 2
        Q[DZ, DZ] = dt * self.q_dz ** 2
        if self.q_jerk > 0.0:
            # Singer: acc' = -acc / tau + jerk noise; exact F, white-jerk Q.
            alpha = 1.0 / self.accel_tau_s
            decay = math.exp(-alpha * dt)
            F[POS, ACC] = (alpha * dt - 1.0 + decay) / alpha ** 2 * I3
            F[VEL, ACC] = (1.0 - decay) / alpha * I3
            F[ACC, ACC] = decay * I3
            jerk = self.q_jerk ** 2 * np.array([
                [dt ** 5 / 20.0, dt ** 4 / 8.0, dt ** 3 / 6.0],
                [dt ** 4 / 8.0, dt ** 3 / 3.0, dt ** 2 / 2.0],
                [dt ** 3 / 6.0, dt ** 2 / 2.0, dt]])
            blocks = (POS, VEL, ACC)
            for i, bi in enumerate(blocks):
                for j, bj in enumerate(blocks):
                    Q[bi, bj] += jerk[i, j] * I3
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
            self.state[YAW] += k * QUARTER_TURN
            if k % 2:
                self.state[R], self.other_r = self.other_r, self.state[R]
                self.state[DZ] = -self.state[DZ]
                self.P[DZ, :] *= -1.0
                self.P[:, DZ] *= -1.0  # P[DZ, DZ] flips twice, staying put
        return best

    def step(self, panel_pos, camera_pos, t_sec, pos_var, gate_nis=16.3, max_outliers=3,
             facing_std=None):
        """
        Predict, associate and update on one detection; return 'update', 'outlier' or 'reacquire'.

        gate_nis is the chi-square(3) bound on the normalised innovation
        (16.3 = 99.9%). Outliers are skipped; max_outliers in a row re-seed
        the position via reacquire(), keeping the spin estimate.
        facing_std (rad): the panel was seen alone; see update_facing().
        """
        self.predict(t_sec)
        self.associate(panel_pos, camera_pos)
        self.last_nis, self.last_logdet = self._innovation(panel_pos, pos_var)
        if self.last_nis > gate_nis:
            self.n_outliers += 1
            if self.n_outliers < max_outliers:
                return 'outlier'
            self.reacquire(panel_pos, camera_pos, t_sec, pos_var)
            return 'reacquire'
        self.n_outliers = 0
        self.update(panel_pos, pos_var)
        if facing_std:
            self.update_facing(camera_pos, facing_std)
        return 'update'

    def update_facing(self, camera_pos, std):
        """
        Pseudo-measure the tracked panel's yaw as the bearing to the camera, +-std.

        A panel seen alone faces the camera: its neighbours, 90 deg round,
        would otherwise present too. With one panel in view nothing else
        fixes yaw, which random-walks and swings the centre round the panel.
        """
        yaw = self.state[YAW]
        panel = self.state[POS] + _offset(yaw, self.state[R], 0.0)
        bearing = math.atan2(camera_pos[1] - panel[1], camera_pos[0] - panel[0])
        y = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
        S = self.P[YAW, YAW] + std ** 2
        K = self.P[:, YAW] / S
        self.state = self.state + K * y
        self.P = self.P - np.outer(K, self.P[YAW, :])
        self.P = 0.5 * (self.P + self.P.T)

    def _h_and_jacobian(self):
        yaw, r = self.state[YAW], self.state[R]
        c, s = math.cos(yaw), math.sin(yaw)
        h = self.state[POS] + _offset(yaw, r, self.state[DZ])
        H = np.zeros((3, N_STATE))
        H[:, POS] = I3
        H[2, DZ] = 1.0
        H[0, YAW], H[0, R] = -r * s, c
        H[1, YAW], H[1, R] = r * c, s
        return h, H

    def nis(self, panel_pos, pos_var):
        return self._innovation(panel_pos, pos_var)[0]

    def _innovation(self, panel_pos, pos_var):
        """Return (NIS, log det S) of panel_pos against the current prediction."""
        h, H = self._h_and_jacobian()
        y = np.asarray(panel_pos) - h
        S = H @ self.P @ H.T + _as_cov(pos_var)
        return float(y @ np.linalg.solve(S, y)), float(np.linalg.slogdet(S)[1])

    def update(self, panel_pos, pos_var):
        h, H = self._h_and_jacobian()
        R_meas = _as_cov(pos_var)
        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ (np.asarray(panel_pos) - h)
        IKH = np.eye(N_STATE) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R_meas @ K.T  # Joseph form
        self.state[R] = min(max(self.state[R], self.r_min), self.r_max)
        self.state[DZ] = min(max(self.state[DZ], -DZ_MAX), DZ_MAX)


class ArmorTracker:
    """
    Bank of ArmorEKFs seeded at different spin rates, plus a still one; the likeliest leads.

    Every hypothesis sees every panel; score is an EWMA (alpha) of its
    negative log-likelihood, NIS + log det S, so a looser model can't win on
    slack alone. still=True adds a filter with v, a and w pinned at 0 for a
    parked, non-spinning target: it takes the lead at still_margin, and
    loses it once the summed log-likelihood ratio against the best moving
    filter passes still_exit_llr. A hypothesis scoring worse than the leader
    by reseed_margin for reseed_after_s is re-seeded from the leader's centre
    and yaw.
    ekf_kwargs go to every ArmorEKF. See README.md.
    """

    def __init__(self, panel_pos, camera_pos, t_sec, pos_var, radius, q_accel,
                 q_yaw_accel, q_radius, spin_priors=(0.0, 7.0, -7.0, 13.0, -13.0),
                 prior_std=3.0, alpha=0.03, reseed_margin=3.0, reseed_after_s=1.0,
                 switch_margin=1.0, switch_after_s=0.5, still=True, still_margin=0.25,
                 still_exit_llr=15.0, **ekf_kwargs):
        self.filters = [ArmorEKF(panel_pos, camera_pos, t_sec, pos_var, radius, q_accel,
                                 q_yaw_accel, q_radius, spin_prior=w, spin_prior_std=prior_std,
                                 **ekf_kwargs)
                        for w in spin_priors]
        if still:
            self.filters.append(ArmorEKF(panel_pos, camera_pos, t_sec, pos_var, radius,
                                         q_accel, q_yaw_accel, q_radius, spin_prior=0.0,
                                         spin_prior_std=0.0, still=True, **ekf_kwargs))
        self.scores = [None] * len(self.filters)  # set by the first step
        self.worse_since = [None] * len(self.filters)
        self.alpha = alpha
        self.reseed_margin = reseed_margin
        self.reseed_after_s = reseed_after_s
        self.lead = 0
        self.switch_margin = switch_margin
        self.switch_after_s = switch_after_s
        self.still_margin = still_margin
        self.still_exit_llr = still_exit_llr
        self._still_cusum = 0.0
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

    def step(self, panel_pos, camera_pos, t_sec, pos_var, gate_nis=16.3, max_outliers=3,
             facing_std=None):
        """Advance every hypothesis on one panel; return the leading filter's status."""
        statuses, nlls = [], []
        for i, f in enumerate(self.filters):
            statuses.append(f.step(panel_pos, camera_pos, t_sec, pos_var, gate_nis, max_outliers,
                                   facing_std))
            nll = min(f.last_nis, gate_nis) + f.last_logdet  # clamped: one wild sample
            nlls.append(nll)
            if self.scores[i] is None:
                self.scores[i] = nll
            else:
                self.scores[i] += self.alpha * (nll - self.scores[i])
        if self.best.still:
            # A parked target that moves: CUSUM of the per-sample log-likelihood
            # ratio against the best moving filter hands over within a few samples.
            moving = [i for i, f in enumerate(self.filters) if not f.still]
            rival = min(moving, key=lambda i: nlls[i])
            self._still_cusum = max(0.0, self._still_cusum + nlls[self.lead] - nlls[rival])
            if self._still_cusum > self.still_exit_llr:
                self.lead = min(moving, key=lambda i: self.scores[i])
                self._challenger, self._still_cusum = None, 0.0
        # The lead changes only after a challenger beats it by switch_margin
        # for switch_after_s: a noise burst briefly favours a collapsed-radius
        # wrong-sign hypothesis, which is least sensitive to it.
        best_i = int(np.argmin(self.scores))
        lead_score = self.scores[self.lead]
        margin = self.still_margin if self.filters[best_i].still else self.switch_margin
        if best_i == self.lead or self.scores[best_i] > lead_score - margin:
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
                f.state[W] = f.spin_prior
                f.state[R] = f.other_r = f.initial_radius
                f.P = best.P.copy()
                f.P[W, :] = f.P[:, W] = 0.0
                f.P[W, W] = f.spin_prior_std ** 2
                f.P[R, :] = f.P[:, R] = 0.0
                f.P[R, R] = 0.05 ** 2
                f.pin_still()
                f.t_sec = best.t_sec
                self.scores[i] = best_score + self.reseed_margin / 2.0
                self.worse_since[i] = None
        return statuses[self.lead]
