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
Relocalize decision for mcb_relay.py (no rclpy import).

The correction is the offset loc(t) - odom(t) at the localization stamp,
/odom interpolated there, added to where the MCB's odometry will read when
it applies the frame: latest /odom extrapolated past both UART legs and
the MCB's read delay. Sent only when confident: localization's xy std and
the extrapolation's (speed x latency_std_s) combined stay under max_std_m,
and the offset clears error_threshold_m and n_sigma of that std. See
README.md's ### mcb_relay.py.
"""
import bisect
import collections
import math

OdomSample = collections.namedtuple('OdomSample', 't x y vx vy')


class Relocalizer:
    """Buffer /odom and decide when and what to relocalize the MCB to."""

    def __init__(self, error_threshold_m=0.05, n_sigma=3.0, max_std_m=0.02,
                 uart_latency_s=0.005, mcb_read_delay_s=0.002, latency_std_s=0.003,
                 hold_off_s=0.3, history_s=1.0):
        self.error_threshold_m = error_threshold_m
        self.n_sigma = n_sigma
        self.max_std_m = max_std_m
        self.uart_latency_s = uart_latency_s
        self.mcb_read_delay_s = mcb_read_delay_s
        self.latency_std_s = latency_std_s
        self.hold_off_s = hold_off_s
        self.history_s = history_s
        self._odom = collections.deque()
        self._last_sent_t = None

    def add_odom(self, t, x, y, vx, vy):
        """Record one /odom sample; stamps must not go backwards."""
        if self._odom and t < self._odom[-1].t:
            self._odom.clear()  # clock reset (sim restart)
        self._odom.append(OdomSample(t, x, y, vx, vy))
        while self._odom and self._odom[0].t < t - self.history_s:
            self._odom.popleft()

    def odom_at(self, t, max_ahead_s=0.1):
        """(x, y) of /odom at t: interpolated, extrapolated up to max_ahead_s, else None."""
        if not self._odom or t < self._odom[0].t or t > self._odom[-1].t + max_ahead_s:
            return None
        if t > self._odom[-1].t:
            last = self._odom[-1]
            return last.x + last.vx * (t - last.t), last.y + last.vy * (t - last.t)
        times = [s.t for s in self._odom]
        i = bisect.bisect_left(times, t)
        b = self._odom[i]
        if i == 0 or b.t == t:
            return b.x, b.y
        a = self._odom[i - 1]
        f = (t - a.t) / (b.t - a.t)
        return a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)

    def decide(self, loc_t, loc_x, loc_y, loc_var_xy, now):
        """
        Return (x, y, apply_t, error_m, std_m) to send, or None.

        loc_var_xy is the larger of localization's x and y variances (m^2).
        now is the send time on the /odom stamps' clock; apply_t is when the
        MCB should adopt (x, y), on that clock.
        """
        if self._last_sent_t is not None and now - self._last_sent_t < self.hold_off_s:
            return None  # the last one may not have reached /odom yet
        at_loc = self.odom_at(loc_t)
        if at_loc is None:
            return None
        dx, dy = loc_x - at_loc[0], loc_y - at_loc[1]
        error = math.hypot(dx, dy)
        last = self._odom[-1]
        speed = math.hypot(last.vx, last.vy)
        std = math.hypot(math.sqrt(max(loc_var_xy, 0.0)), speed * self.latency_std_s)
        if std > self.max_std_m or error <= max(self.error_threshold_m, self.n_sigma * std):
            return None
        # /odom is stamped on arrival, one UART leg after the MCB read it,
        # and the frame we send lands one leg plus a read delay from now.
        horizon = now - last.t + 2.0 * self.uart_latency_s + self.mcb_read_delay_s
        self._last_sent_t = now
        apply_t = now + self.uart_latency_s + self.mcb_read_delay_s
        return (last.x + last.vx * horizon + dx, last.y + last.vy * horizon + dy,
                apply_t, error, std)
