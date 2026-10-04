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
The MCB's odometry axes against REP-105, for the mcb_x_right parameter (no rclpy).

MCBV3 position-based-cv's odometry is x right, y forward of the boot heading;
REP-105 is x forward, y left. Its aim takes CV_TARGET x/y less that odometry
as REP-105, so the aim point goes out shifted to cancel the turn.
see README.md for design rationale
"""


def from_mcb(x, y):
    """MCB odometry (x right, y forward) -> REP-105 (x forward, y left)."""
    return y, -x


def to_mcb(x, y):
    """REP-105 -> MCB odometry axes; the inverse of from_mcb."""
    return -y, x


def aim_to_mcb(tx, ty, ox, oy):
    """
    REP-105 aim point t -> CV_TARGET x/y for an MCB at REP-105 pose o.

    The MCB subtracts its odometry, to_mcb(o), and reads the rest as
    REP-105, so it must get t - o + to_mcb(o).
    """
    mx, my = to_mcb(ox, oy)
    return tx - ox + mx, ty - oy + my
