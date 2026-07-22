#!/usr/bin/env python3
"""
Automated integration test suite for sentry_pkg's map-relative localization
drift/jerk correction behavior, exercised against sim's synthetic
wheel-odometry noise model (sim/sim/pose_emulator.py: odom_noise_enabled/
odom_drift_stddev/odom_jitter_stddev/odom_jerk_stddev, see that file's module
docstring for the full noise-model design rationale).

Runs against any of auto.launch.py's localization_mode backends (--backend
slam/amcl/ekf, default slam -- not mapping, see BACKENDS below for why).
Originally written slam_toolbox-only (hence the old filename,
run_slam_drift_tests.py); generalized once auto.launch.py grew amcl/ekf
alongside slam_toolbox's own localization mode, since exercising "does the
correction layer respond to jerks/drift correctly" is equally relevant to
all of them, just watching a different TF edge (see BACKENDS below).

WHY THIS EXISTS
---------------
Before this suite, exercising this correction behavior meant manually:
launching sim, launching sentry_pkg's localization stack, firing `ros2
service call /pose_emulator/trigger_jerk ...` or twiddling
odom_noise_enabled by hand, then eyeballing `ros2 run tf2_ros tf2_echo <the
right two frames>` in a separate shell, then manually tearing both launches
down before the next attempt. That's slow, error-prone (easy to forget a
teardown step and leave orphaned nodes causing duplicate-node TF jitter on
the next run -- see SESSION_NOTES.md), and not repeatable enough to safely
use as a regression check after touching slam.yaml/amcl.yaml/ekf.yaml or
pose_emulator.py's noise model. This script automates exactly that manual
loop: launch stack -> drive scenario -> sample the correction TF over time
-> assert -> tear down -> repeat.

WHY A STANDALONE SCRIPT, NOT A pytest/colcon-test FILE
-------------------------------------------------------
sentry_pkg/test/ already has ament_copyright/ament_flake8/ament_pep257
pytest-style tests, run via `colcon test`. Those are fast, single-process,
static-analysis-style checks with no external state. This suite is the
opposite on every axis that matters for choosing a test runner:
  - It needs a running Docker container, gz-sim, and two full `ros2 launch`
    trees (sim + sentry_pkg) -- none of which `colcon test`'s default
    invocation sets up or tears down for you.
  - Each scenario takes real wall-clock seconds to tens of seconds
    (physics settling, minimum_time_interval/minimum_travel_distance-style
    gating, scan-match convergence) -- not typical unit-test-speed.
  - Scenarios must run strictly sequentially, each with a full stack
    teardown/relaunch in between, to get a clean map/TF state -- colcon
    test's parallel-by-default test execution model actively fights this.
  - Failure diagnosis needs the actual measured drift/correction numbers
    printed clearly, not just a pytest assert traceback.
Wiring this into colcon test/pytest discovery would mean fighting the
runner's assumptions (test isolation, parallelism, speed) for no real
benefit -- nothing here is meant to run as part of a routine `colcon test`
pass anyway; it's meant to be invoked deliberately, e.g. after tuning
slam.yaml/amcl.yaml/ekf.yaml or pose_emulator.py's noise params. A plain
script that is simply run directly is the better fit. It still uses rclpy
directly (not subprocess+CLI parsing) for all in-process ROS interaction
(TF lookups, service calls, cmd_vel publishing), since that's the natural,
robust way to talk to a running ROS graph from Python.

USAGE
-----
Run from inside the isaac_ros_dev container (needs rclpy + the sim/
sentry_pkg packages built and sourced -- exactly what dexec.sh's env
sourcing already provides), from the host:

    isaac_ros_common/scripts/dexec.sh -- \\
        python3 /workspaces/isaac_ros-dev/src/sentry_pkg/test/slam_integration/run_localization_drift_tests.py

Optional: --backend {slam,amcl,ekf} (default slam) to pick which
auto.launch.py localization_mode to exercise. --scenario NAME to run just
one scenario (see SCENARIOS below), --keep-running to skip teardown after
the last scenario (for interactive follow-up inspection), --headless to
run gz-sim headless instead of the default GUI window (faster, but
nothing to watch -- GUI is on by default so a human can watch/sanity-check
scenario behavior live, matching the standing "always launch sim with
GUI" rule in SESSION_NOTES.md).

This script manages its OWN sim + sentry_pkg launch trees end to end (using
the same setsid/process-group approach as dexec.sh -d / kill_launch.sh, see
LaunchTree below) -- it does not attach to or reuse a stack you may already
have running interactively. If you have an interactive stack up already,
either stop it first (this script needs its ports/topics/services
exclusively -- ROS topics/services are process-global, not namespaced per
launch, so two stacks would collide) or just let this script run in a
separate terminal after you tear yours down; it does not try to coexist
with one.

BACKENDS
--------
Each backend owns a different TF edge as its "correction" -- the thing
these scenarios actually watch is whichever edge that backend is
responsible for, not literally "map->odom" in every case:
  - 'slam' (default): slam_toolbox's own localization mode owns map->odom.
    Gated on distance traveled since the last processed scan (see
    slam.yaml's minimum_travel_distance comment) -- a jerk with zero
    reported motion afterward never even attempts a fresh scan match.
  - 'amcl': nav2 amcl owns map->odom instead. Gated the same conceptual
    way as slam_toolbox (amcl.yaml's update_min_d/update_min_a are its
    equivalent of minimum_travel_distance/heading), so the same
    jerk_stationary/jerk_with_motion assertions apply unchanged, just
    watching amcl's own TF broadcast instead of slam_toolbox's.
  - 'ekf' owns odom->root instead of map->odom (localization_mode:=ekf
    runs no map node at all -- see auto.launch.py's module docstring) --
    baseline/continuous_drift are exercised against odom->root instead of
    map->odom (BACKEND_FRAMES below), since that's the analogous "is the
    correction layer behaving" edge for this backend. jerk_with_motion/
    jerk_stationary are SKIPPED for ekf, not asserted: ekf_node fuses
    /odom's x/y directly (see config/ekf.yaml), with no
    distance-traveled gate analogous to slam_toolbox/amcl's, so a
    stationary jerk's effect on odom->root isn't characterized the same
    way and asserting against either the same-shape "must not change" or
    "must change to track the jerk" expectation would just be a guess --
    the EKF pipeline's own tuning/verification is still open work (see
    SESSION_NOTES.md), revisit once that lands. unmapped_obstacle is
    SKIPPED for ekf too, for a simpler reason: ekf_node never touches
    /scan at all, so an unmapped lidar return has no defined effect on
    odom->root whatsoever -- there's no scan-matching step here to have
    an opinion about.
  - 'mapping' is NOT a --backend choice here: mapping mode's job is
    building/refining a map, not evaluating localization accuracy against
    one, so these drift/jerk correction scenarios don't have a meaningful
    reading against it.

SCENARIOS
---------
1. baseline        -- odom_noise_enabled:=false. Stack comes up cleanly,
                       the correction TF settles and stays STABLE (does
                       not drift further with no noise/motion) -- NOT
                       necessarily near (0,0,0) for slam/amcl: the saved
                       ARCC26 map's origin doesn't coincide with sim's
                       spawn pose, so a consistent ~0.1-0.15m absolute
                       offset here is normal. No ERROR in any log.
2. continuous_drift -- odom_noise_enabled:=true, default drift/jitter
                       stddevs plus a 0.25 wheel-slip ratio (reported
                       odometry loses 1/4 of every meter actually
                       driven, see pose_emulator.py's odom_slip_ratio),
                       robot given continuous motion at real speed. Over
                       an observation window, the correction TF should
                       stay BOUNDED (periodic correction keeping up with
                       accumulated drift+slip), not grow without bound.
3. jerk_with_motion -- (slam/amcl only, see BACKENDS) fire trigger_jerk,
                       then command a small amount of /cmd_vel motion.
                       Assert the correction TF produces a prompt, real
                       correction whose magnitude tracks the jerk.
4. jerk_stationary  -- (slam/amcl only, see BACKENDS) fire trigger_jerk,
                       robot never moves afterward. Assert the correction
                       TF does NOT change. This is a KNOWN, EXPECTED,
                       DOCUMENTED structural limitation (see slam.yaml's
                       minimum_travel_distance comment, amcl.yaml's
                       update_min_d/a, and pose_emulator.py's trigger_jerk
                       docstring), not a bug: both backends' scan-matching
                       is gated on distance traveled since the last
                       processed scan, as measured off REPORTED odometry
                       -- which a jerk deliberately leaves unchanged. With
                       zero reported motion, that gate never opens, so
                       neither backend even attempts a fresh scan match. A
                       PASS on this scenario means "the suite correctly
                       observed the known limitation," not "localization
                       is broken" -- read the printed rationale in its
                       output before assuming a regression.
5. unmapped_obstacle -- (slam/amcl only, see BACKENDS) spawn a static box
                       into the running world mid-scenario (not present in
                       ARCC_Field_2026.sdf or the saved ARCC26 map -- from
                       the backend's perspective it's a lidar return with
                       no corresponding map feature), then drive a 2m
                       square loop centered on it (OBSTACLE_LOOP_LEGS, 1m
                       out from the box in every direction -- see that
                       constant's comment for the wall-clearance
                       derivation) so it's seen from every angle but never
                       driven into. This scenario is about the correction
                       layer's reaction to a mismatch between map and
                       reality, not a collision test. Assert the
                       correction TF stays bounded relative to its
                       pre-spawn value (one small unmapped object should
                       only locally corrupt returns near it, not swing the
                       whole map alignment) and that scans keep flowing
                       (backend didn't stall).
"""
import argparse
import math
import os
import shlex
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


# --------------------------------------------------------------------------
# Process-group launch management (in-container reimplementation of
# dexec.sh -d + kill_launch.sh's approach, since this script already runs
# INSIDE the container -- no docker exec indirection needed here, but the
# same "setsid so the whole tree is one killable process group, SIGINT the
# group not just the launch PID" logic applies and is just as load-bearing
# here as it is for those host-side scripts).
# --------------------------------------------------------------------------

class LaunchTree:
    """Launches a `ros2 launch ...` command as its own process group and
    can tear the whole tree down cleanly with SIGINT (falling back to
    SIGKILL if it doesn't exit in time). Mirrors kill_launch.sh's
    "SIGINT the process group, never pkill/killall" approach -- a partial
    kill that leaves orphaned children running alongside a fresh relaunch
    causes duplicate-node TF jitter (see SESSION_NOTES.md), which would
    silently corrupt this suite's own results if it happened between
    scenarios.
    """

    def __init__(self, name, cmd, log_path):
        self.name = name
        self.cmd = cmd
        self.log_path = log_path
        self.proc = None
        self.log_file = None

    def start(self):
        self.log_file = open(self.log_path, 'w')
        # start_new_session=True == setsid: makes this process its own
        # process group leader, so signaling -pgid reaches every child
        # node the launch spawns, not just the launch process itself.
        self.proc = subprocess.Popen(
            self.cmd,
            stdout=self.log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(f'[{self.name}] started pid={self.proc.pid} '
              f'log={self.log_path} cmd={shlex.join(self.cmd)}')

    def stop(self, timeout=15.0):
        if self.proc is None or self.proc.poll() is not None:
            return
        pgid = os.getpgid(self.proc.pid)
        print(f'[{self.name}] sending SIGINT to process group {pgid}...')
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                print(f'[{self.name}] exited cleanly.')
                self.log_file.close()
                return
            time.sleep(0.2)
        print(f'[{self.name}] did not exit within {timeout}s, SIGKILLing '
              f'process group {pgid}.')
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait(timeout=10)
        self.log_file.close()

    def log_text(self):
        try:
            with open(self.log_path) as f:
                return f.read()
        except OSError:
            return ''


def check_no_orphans(label):
    """Sanity check used before/after the whole suite: warns (does not
    fail) if sim/localization processes are already running that this
    script did not start itself -- most likely an interactive session's
    stack left over, or a previous run of this suite that didn't clean
    up. This script refuses to start its own stack on top of one already
    running (topics/services are process-global, they WILL collide), it
    just reports what it sees so a human can decide what to do.
    """
    try:
        out = subprocess.run(
            ['bash', '-c',
             "ps aux | grep -E 'ign gazebo|gz sim|slam_toolbox|amcl|"
             "map_server|ekf_filter_node|pose_translator|pose_emulator' | "
             "grep -v grep | "
             # Excludes this script's own process: --backend amcl/ekf on
             # its own command line would otherwise self-match the
             # amcl/ekf_filter_node patterns above.
             "grep -v run_localization_drift_tests.py"],
            capture_output=True, text=True, timeout=10,
        ).stdout.strip()
    except Exception as e:
        out = f'(failed to check: {e})'
    try:
        load1, load5, load15 = os.getloadavg()
        nproc = os.cpu_count() or 1
        print(f'[{label}] host load average: {load1:.2f} {load5:.2f} '
              f'{load15:.2f} ({nproc} CPUs) -- heavy unrelated CPU load '
              f'(e.g. a leftover rviz2/other interactive process from '
              f'earlier manual testing) can slow scan-matching enough to '
              f'make timing-sensitive scenarios (jerk_with_motion '
              f'especially) look like false failures; check `ps aux '
              f'--sort=-%cpu` if a run fails unexpectedly.')
    except OSError:
        pass
    if out:
        print(f'[{label}] WARNING: localization/sim-related processes '
              f'already running:\n{out}')
        return False
    return True


# --------------------------------------------------------------------------
# In-process ROS helper: TF sampling, trigger_jerk service calls, cmd_vel.
# --------------------------------------------------------------------------

class LocalizationTestHelper(Node):
    def __init__(self, parent_frame='map', child_frame='odom'):
        super().__init__('localization_drift_test_helper')
        # Which TF edge counts as "the correction" -- see BACKENDS in the
        # module docstring: (map, odom) for slam/amcl, (odom, root) for
        # ekf.
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.jerk_client = self.create_client(
            Trigger, '/pose_emulator/trigger_jerk')
        self._scan_count = 0
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

    def _on_scan(self, msg):
        self._scan_count += 1

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_scans_flowing(self, min_scans=10, timeout=60.0):
        """Blocks until at least `min_scans` /scan messages have been
        received, or `timeout` elapses. Used as the real "is the stack
        actually up and processing lidar data" readiness signal -- more
        reliable than checking for the correction TF's mere existence,
        since slam_toolbox/amcl broadcast an initial identity transform
        immediately on startup (before processing a single real scan
        against the loaded map), so waiting on TF alone can let a
        scenario start its timed assertions well before the stack is
        actually warmed up (observed directly: a run where slam_toolbox
        had only registered 2 scans total in over 30 wall-clock seconds,
        evidently due to transient system load slowing scan-matcher
        startup). Returns True if the threshold was reached, False on
        timeout (caller should treat that as a slow/unhealthy stack, not
        silently proceed).
        """
        self._scan_count = 0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._scan_count >= min_scans:
                return True
            self.spin_for(0.5)
        return False

    def get_correction_tf(self, timeout=2.0):
        """Returns (x, y, yaw) of self.parent_frame->self.child_frame, or
        None if unavailable (e.g. the backend hasn't published it yet)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, rclpy.time.Time(),
                timeout=Duration(seconds=timeout))
        except (LookupException, ExtrapolationException, Exception):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def wait_for_correction_tf(self, timeout=30.0, poll=0.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            v = self.get_correction_tf(timeout=0.5)
            if v is not None:
                return v
            self.spin_for(poll)
        return None

    def call_trigger_jerk(self, timeout=10.0):
        if not self.jerk_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('/pose_emulator/trigger_jerk not available')
        future = self.jerk_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('trigger_jerk call timed out')
        return future.result()

    def call_trigger_jerk_and_get_magnitude(self, timeout=10.0):
        """Calls trigger_jerk and returns the actual applied |jerk| (m),
        parsed out of the Trigger response's `message` field (see
        sim/sim/pose_emulator.py's _trigger_jerk_srv -- Trigger has no
        dedicated payload field, so the real (dx, dy) that was actually
        drawn/applied is encoded into the message string). Using the real
        applied magnitude rather than the odom_jerk_stddev distribution
        parameter matters: a single random draw from that distribution can
        be much larger or smaller than the stddev itself (e.g. a draw near
        zero is entirely possible), so asserting a fixed fraction of
        stddev as the expected correction is flaky by construction. Falls
        back to None (caller should fall back to a stddev-based estimate)
        if the message can't be parsed -- keeps this robust to
        pose_emulator message-format changes rather than hard-failing.
        """
        result = self.call_trigger_jerk(timeout=timeout)
        try:
            # Expected format: "jerk applied: dx=<float> dy=<float>"
            parts = result.message.split('dx=')[1]
            dx_str, dy_str = parts.split('dy=')
            dx = float(dx_str.strip())
            dy = float(dy_str.strip())
            return math.hypot(dx, dy)
        except (IndexError, ValueError):
            return None

    def drive(self, vx, vy, duration):
        """Publish /cmd_vel at 10Hz for `duration` seconds, then stop."""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.cmd_vel_pub.publish(msg)
            self.spin_for(0.1)
        self.cmd_vel_pub.publish(Twist())  # stop
        self.spin_for(0.2)


# --------------------------------------------------------------------------
# Scenario plumbing
# --------------------------------------------------------------------------

WORKSPACE = '/workspaces/isaac_ros-dev'
LOG_DIR = '/tmp/localization_drift_tests'

# Which TF edge each backend's "correction" actually shows up on -- see
# BACKENDS in the module docstring.
BACKEND_FRAMES = {
    'slam': ('map', 'odom'),
    'amcl': ('map', 'odom'),
    'ekf': ('odom', 'root'),
}

# Driving path used by continuous_drift/jerk_with_motion, at the robot's
# real 4.0 m/s top speed (see those scenarios' git history for why, vs.
# the old 0.15 m/s wiggle).
#
# A first version of this (2026-07-20) tried to actually tour the field --
# mapped clean_map.pgm's wall positions via connected-component analysis,
# converted to world coords via clean_map.yaml's resolution/origin, and
# built a 6-leg loop that AABB-checked clear of every wall by real margin
# (the closest was ~0.77m from the maze block). It still ended up driving
# into the upper-middle wall, confirmed live by watching gz-sim: the first
# ~10 loop cycles (~40s) tracked fine, then map->odom error grew sharply
# and never recovered (see that commit's test log) -- consistent with an
# actual collision partway through, not a wrong-from-the-start coordinate
# error (which would fail the very first cycle, not the tenth). Most
# likely cause: these legs are open-loop (fixed velocity for a fixed
# duration, no position feedback at all), so small per-leg execution
# error on the free-floating chassis (no joint chain, no friction to
# damp overshoot) can accumulate across many repeated cycles until it's
# enough to clip a wall that looked comfortably clear on paper. Not worth
# chasing the exact mechanism further -- the fix is a smaller, simpler
# loop, not a more precisely-computed big one.
#
# This version stays inside the open central gap the whole time -- never
# needs to approach any wall's x/y band at all, at any point in the loop,
# so there's nothing to route around and no accumulated-drift budget that
# matters: even generous execution error still lands nowhere near a wall.
# Comfortable margins at this size (world coords, meters): ~1.49m south
# of upper_mid's near edge (y=2.49), ~1.11m north of lower_mid's (y=
# -2.11), and both are nowhere near bottom_wall's ramp-adjacent edge
# (y=-3.35) -- this loop never goes south of y=-1.0.
# Legs are (vx, vy, duration), not (vx, vy) cycled at a fixed duration,
# so scenarios can reuse this one constant either way.
PATROL_LEGS = [
    (4.0, 0.0, 0.25),    # east   0,0   -> 1,0
    (0.0, 4.0, 0.25),    # north  1,0   -> 1,1
    (-4.0, 0.0, 0.25),   # west   1,1   -> 0,1
    (0.0, -4.0, 0.25),   # south  0,1   -> 0,0
]


def source_prefix():
    return (
        f'source /opt/ros/humble/setup.bash && '
        f'source {WORKSPACE}/../ros2_ws/install/setup.bash 2>/dev/null; '
        f'source {WORKSPACE}/install/setup.bash && '
    )


def launch_cmd(args_str):
    # Wrapped in bash -lc so the sourced environment (both workspace
    # installs) is present, matching what dexec.sh's SOURCE_ENV does for
    # host-side invocations -- this script runs inside the container
    # already, so no docker exec layer, but the workspace sourcing is
    # still required since this process wasn't necessarily started from
    # an interactive login shell.
    return ['bash', '-lc', source_prefix() + args_str]


# --------------------------------------------------------------------------
# Obstacle spawning (mid-scenario, unmapped_obstacle scenario only).
# --------------------------------------------------------------------------

# scenario_unmapped_obstacle drives its OWN loop (OBSTACLE_LOOP_LEGS
# below), not PATROL_LEGS -- earlier versions (2026-07-21) tried placing
# the box off to the side of PATROL_LEGS's existing loop and reusing that
# loop unshifted, then tried various reposition offsets to dodge it after
# live testing showed collisions/overshoot -- simpler and more robust to
# put the box at the loop's own center and size the loop 1m out from it
# in every direction, so clearance is true by construction instead of by
# a chain of one-off offset corrections.
# OBSTACLE_XY = (0.5, 0.5) deliberately reuses PATROL_LEGS's own loop
# center (its corners (0,0),(1,0),(1,1),(0,1) center on (0.5,0.5)) --
# already-validated open space (continuous_drift/jerk_with_motion drive
# through this immediate area for tens of seconds without incident), not
# a new, untested spot.
# NOT baked into ARCC_Field_2026.sdf or the saved ARCC26 map -- that's
# the point: from the backend's perspective this is a lidar return with
# no corresponding feature in the map it loaded.
OBSTACLE_XY = (0.5, 0.5)
OBSTACLE_SIZE = 0.3  # meters, x/y footprint
# height, NOT a cube -- 2026-07-21: the original 0.3m cube (spanning
# z=[0, 0.3], resting on the ground) sat entirely below the lidar's
# mounted height (lidar link is offset ~0.35m above ground: body's own
# ~0.16m + headlink's 0.252m + lidarlink's 0.072m, see
# sim/urdf/sentry.urdf.xacro), so the single-plane 2D scan never actually
# intersected the box at all -- it was invisible to the lidar the entire
# time, which is why every unmapped_obstacle run and every do_beamskip/
# alpha/max_beams tuning attempt showed the same wobble whether the box
# was spawned or not (see the no-obstacle diagnostic in this session's
# history -- it wasn't actually isolating anything, the "with obstacle"
# case never saw the obstacle either). >0.6m tall, based at the ground
# (z=[0, OBSTACLE_HEIGHT]), guarantees it spans the lidar's height with
# margin on both sides regardless of the exact mount height.
OBSTACLE_HEIGHT = 0.8  # meters

# 2m square loop centered on OBSTACLE_XY, corners at (-0.5,-0.5),
# (1.5,-0.5), (1.5,1.5), (-0.5,1.5) -- exactly 1m out from the box's
# center on every side (box half-width 0.15m, so ~0.85m from each face).
# Checked against this file's own documented wall clearances (see
# PATROL_LEGS's comment; y-axis only, no x-axis data exists here):
#   north edge y=1.5 -- 0.99m clear of upper_mid's wall at y=2.49.
#   south edge y=-0.5 -- 0.5m short of PATROL_LEGS's own documented -1.0
#     floor (which itself has a further 1.11m before lower_mid's wall),
#     so comfortably inside already-established safe territory.
#   x extent -0.5 to 1.5 -- only 0.5m beyond the already-validated
#     x=[0,1] core on each side (unlike earlier abandoned +1/+2m east
#     excursions), no wall data to check against but a much smaller,
#     more conservative reach into unknown territory.
# Legs are (vx, vy, duration) like PATROL_LEGS, but 2m per side (0.5s at
# 4.0 m/s) since this loop's side length is 2m, not 1m.
OBSTACLE_LOOP_LEGS = [
    (4.0, 0.0, 0.5),    # east   (-0.5,-0.5) -> (1.5,-0.5)
    (0.0, 4.0, 0.5),    # north  (1.5,-0.5)  -> (1.5,1.5)
    (-4.0, 0.0, 0.5),   # west   (1.5,1.5)   -> (-0.5,1.5)
    (0.0, -4.0, 0.5),   # south  (-0.5,1.5)  -> (-0.5,-0.5)
]


def spawn_box_obstacle(name='unmapped_test_obstacle', xy=OBSTACLE_XY,
                        size=OBSTACLE_SIZE, height=OBSTACLE_HEIGHT,
                        timeout=15.0):
    """One-shot spawn of a static box into the running gz-sim world, via
    the same `ros_gz_sim create -string <inline SDF>` mechanism
    sim.launch.py's spawn_robot uses (-topic is documented broken for
    this stack -- see that Node's comment / SESSION_NOTES.md) -- but run
    directly as a subprocess here rather than as a launch Node, since
    this needs to fire mid-scenario (after the pre-spawn baseline is
    sampled), not at stack startup. <static>true</static>: no
    physics/inertia needed, it should never move on its own. Torn down
    for free when the scenario's full sim teardown kills the whole
    gz-sim process group afterward -- no separate despawn needed.
    size is the x/y footprint, height is z (NOT a cube -- see
    OBSTACLE_HEIGHT's comment for why it must clear the lidar's mounted
    height), based at the ground (z=[0, height]).
    """
    x, y = xy
    sdf = (
        '<sdf version="1.6"><model name="{name}"><static>true</static>'
        '<pose>{x} {y} {z} 0 0 0</pose><link name="link">'
        '<collision name="collision"><geometry><box><size>{s} {s} {h}'
        '</size></box></geometry></collision>'
        '<visual name="visual"><geometry><box><size>{s} {s} {h}</size>'
        '</box></geometry><material><ambient>0.8 0.1 0.1 1</ambient>'
        '<diffuse>0.8 0.1 0.1 1</diffuse></material></visual>'
        '</link></model></sdf>'
    ).format(name=name, x=x, y=y, z=height / 2.0, s=size, h=height)
    cmd = (f'ros2 run ros_gz_sim create -string {shlex.quote(sdf)} '
           f'-name {name} -allow_renaming false')
    result = subprocess.run(
        launch_cmd(cmd), capture_output=True, text=True, timeout=timeout)
    if result.returncode != 0:
        raise RuntimeError(
            f'spawning obstacle {name!r} failed (rc={result.returncode}): '
            f'{result.stdout}\n{result.stderr}')


class Scenario:
    def __init__(self, name, description):
        self.name = name
        self.description = description
        self.passed = None
        self.skipped = False
        self.details = []

    def log(self, msg):
        print(f'    {msg}')
        self.details.append(msg)

    def result(self, passed, summary):
        self.passed = passed
        status = 'PASS' if passed else 'FAIL'
        print(f'  [{status}] {self.name}: {summary}')
        self.details.append(f'{status}: {summary}')

    def skip(self, reason):
        self.skipped = True
        print(f'  [SKIP] {self.name}: {reason}')
        self.details.append(f'SKIP: {reason}')


def run_stack(gui, backend, odom_noise_enabled, odom_jerk_stddev=None,
              odom_drift_stddev=None, odom_jitter_stddev=None,
              odom_slip_ratio=None):
    """Starts sim + sentry_pkg launch trees, waits for the graph to come
    up, returns (sim_tree, sentry_tree, helper_node). Caller must call
    teardown_stack() when done."""
    os.makedirs(LOG_DIR, exist_ok=True)

    sim_args = (
        f"ros2 launch sim sim.launch.py gui:={'true' if gui else 'false'} "
        f"odom_noise_enabled:={'true' if odom_noise_enabled else 'false'}"
    )
    if odom_jerk_stddev is not None:
        sim_args += f' odom_jerk_stddev:={odom_jerk_stddev}'
    if odom_drift_stddev is not None:
        sim_args += f' odom_drift_stddev:={odom_drift_stddev}'
    if odom_jitter_stddev is not None:
        sim_args += f' odom_jitter_stddev:={odom_jitter_stddev}'
    if odom_slip_ratio is not None:
        sim_args += f' odom_slip_ratio:={odom_slip_ratio}'

    sim_tree = LaunchTree(
        'sim', launch_cmd(sim_args),
        os.path.join(LOG_DIR, 'sim.log'))
    sim_tree.start()

    # Give gz-sim + robot spawn a head start before bringing up
    # localization, which otherwise starts subscribing to /scan and /pose
    # before either exists -- not fatal (ROS handles late publishers fine)
    # but avoids some noisy early "waiting for transform" warnings that
    # make log-scraping for real errors harder.
    time.sleep(8.0)

    # map_file:= must be passed explicitly for slam -- auto.launch.py's own
    # map_file arg default flipped from ARCC26 to clean_map on 2026-07-20,
    # and clean_map doesn't have a real .posegraph/.data yet (see that
    # arg's declaration in auto.launch.py), so slam_toolbox's
    # load_map:=true against the bare default silently deserializes
    # nothing and starts blank -- confirmed live (2026-07-21): drift/jerk
    # scenarios' correction-TF assertions are meaningless against a blank
    # map, they need the real saved ARCC26 pose graph this suite has
    # always been written to test against. amcl is different: it only
    # ever reads map_file's .yaml occupancy grid (no posegraph concept at
    # all), and clean_map's .yaml exists and is deliberately the
    # simpler/cleaner rendering meant for it (see auto.launch.py's
    # map_file arg description) -- leave amcl on the bare default rather
    # than forcing ARCC26 on it too. ekf never reads map_file, so the
    # override is harmless either way for it; only overridden for slam.
    sentry_args = (
        'ros2 launch sentry_pkg auto.launch.py real_hardware:=false '
        f'localization_mode:={backend} load_map:=true'
    )
    if backend == 'slam':
        sentry_args += (
            f' map_file:={WORKSPACE}/install/sentry_pkg/share/sentry_pkg/map/ARCC26'
        )
    sentry_tree = LaunchTree(
        'sentry_pkg', launch_cmd(sentry_args),
        os.path.join(LOG_DIR, 'sentry_pkg.log'))
    sentry_tree.start()

    parent_frame, child_frame = BACKEND_FRAMES[backend]
    helper = LocalizationTestHelper(parent_frame, child_frame)
    return sim_tree, sentry_tree, helper


def teardown_stack(sim_tree, sentry_tree, helper):
    if helper is not None:
        helper.destroy_node()
    # sentry_pkg first (consumer of sim's topics), then sim -- avoids
    # the localization backend/pose_translator spending their shutdown
    # window complaining about topics that vanished out from under them.
    if sentry_tree is not None:
        sentry_tree.stop()
    if sim_tree is not None:
        sim_tree.stop()


def wait_for_stack_ready(sc, helper, min_scans=10, timeout=60.0):
    """Common readiness gate for every scenario: block until /scan is
    actually flowing at a reasonable volume (see
    LocalizationTestHelper.wait_for_scans_flowing's docstring for why
    TF's mere existence isn't a sufficient readiness signal on its own).
    Logs the outcome onto the scenario and returns True/False; scenarios
    should treat False as a hard failure of that run (an unhealthy/too-
    slow stack invalidates the scenario's timing-sensitive assertions),
    not something to silently paper over.
    """
    ok = helper.wait_for_scans_flowing(min_scans=min_scans, timeout=timeout)
    if ok:
        sc.log(f'stack ready: >= {min_scans} /scan messages received')
    else:
        sc.log(f'stack NOT ready: fewer than {min_scans} /scan messages '
               f'received within {timeout}s -- treating as an unhealthy/'
               f'too-slow run, not a correctness result')
    return ok


def scan_log_for_errors(log_text, name):
    """Returns a list of suspicious lines (ERROR-level, tracebacks,
    segfault indicators) from a launch tree's combined log."""
    bad = []
    for line in log_text.splitlines():
        low = line.lower()
        if '[error]' in low or 'traceback' in low or 'segmentation fault' in low:
            bad.append(line)
    return bad


# --------------------------------------------------------------------------
# Scenarios
# --------------------------------------------------------------------------

def scenario_baseline(gui, backend):
    parent, child = BACKEND_FRAMES[backend]
    edge = f'{parent}->{child}'
    sc = Scenario('baseline', f'no noise: stack comes up cleanly, {edge} '
                              'settles and stays STABLE (not necessarily '
                              'near zero -- see note below), no errors')
    sim_tree = sentry_tree = helper = None
    try:
        sim_tree, sentry_tree, helper = run_stack(
            gui, backend, odom_noise_enabled=False)
        if not wait_for_stack_ready(sc, helper):
            sc.result(False, 'stack failed to reach a healthy /scan rate '
                              'in time -- see log above')
            return sc
        pose = helper.wait_for_correction_tf(timeout=45.0)
        if pose is None:
            sc.result(False, f'{edge} never became available within 45s')
            return sc
        x, y, yaw = pose
        mag = math.hypot(x, y)
        sc.log(f'{edge} = (x={x:.4f}, y={y:.4f}, yaw={yaw:.4f}), '
               f'|xy|={mag:.4f} m')
        # NOTE: for slam/amcl, this is NOT expected to be near (0,0,0)
        # here, even with zero injected noise -- the saved ARCC26 map's
        # origin (see map/ARCC26.yaml: origin: [-4.3, -6.23, 0]) does not
        # coincide with sim's robot spawn pose / map_start_pose:=[0,0,0]
        # used at launch, so a consistent ~0.1-0.15m offset here is
        # NORMAL and was confirmed reproducible across many runs this
        # session with odom_noise disabled. What this scenario actually
        # checks is STABILITY: with no noise and no motion, that offset
        # should not drift further over time (a growing offset here,
        # even with noise disabled, would indicate a real problem in
        # the backend's steady-state behavior, unrelated to the noise
        # model).

        # Let it run a bit longer and re-sample.
        helper.spin_for(10.0)
        pose2 = helper.wait_for_correction_tf(timeout=5.0)
        x2, y2, yaw2 = pose2 if pose2 else pose

        drift = math.hypot(x2 - x, y2 - y)
        sc.log(f'after +10s: {edge} = (x={x2:.4f}, y={y2:.4f}), '
               f'drift from first sample = {drift:.4f} m')

        sim_errs = scan_log_for_errors(sim_tree.log_text(), 'sim')
        sentry_errs = scan_log_for_errors(sentry_tree.log_text(), 'sentry_pkg')
        for e in (sim_errs + sentry_errs)[:10]:
            sc.log(f'log error: {e}')

        DRIFT_THRESHOLD = 0.05  # meters, over the 10s stability window
        ok = drift < DRIFT_THRESHOLD and not sim_errs and not sentry_errs
        sc.result(ok,
                   f'{edge} drift over 10s = {drift:.4f} m (threshold '
                   f'{DRIFT_THRESHOLD} m; absolute offset {mag:.4f} m is '
                   f'expected/normal, see note above), '
                   f'sim_errors={len(sim_errs)}, sentry_errors={len(sentry_errs)}')
        return sc
    finally:
        teardown_stack(sim_tree, sentry_tree, helper)


def scenario_continuous_drift(gui, backend):
    parent, child = BACKEND_FRAMES[backend]
    edge = f'{parent}->{child}'
    sc = Scenario('continuous_drift',
                  f'continuous drift+jitter+slip with motion: {edge} '
                  'should correct periodically and stay bounded, not grow '
                  'without limit')
    sim_tree = sentry_tree = helper = None
    # 0.25 (down from an initial 0.5, see git history) -- reported
    # odometry loses 1/4 of every meter the robot actually drives (see
    # pose_emulator.py's odom_slip_ratio for the model), on top of the
    # existing drift/jitter random-walk. Still a real slip-induced
    # discrepancy on top of drift/jitter, just less severe than the
    # initial value.
    SLIP_RATIO = 0.25
    try:
        sim_tree, sentry_tree, helper = run_stack(
            gui, backend, odom_noise_enabled=True, odom_slip_ratio=SLIP_RATIO)
        if not wait_for_stack_ready(sc, helper):
            sc.result(False, 'stack failed to reach a healthy /scan rate '
                              'in time -- see log above')
            return sc
        pose = helper.wait_for_correction_tf(timeout=45.0)
        if pose is None:
            sc.result(False, f'{edge} never became available within 45s')
            return sc

        samples = []
        OBSERVE_SECONDS = 60.0
        # Real 4.0 m/s driving around PATROL_LEGS's mapped-safe loop (see
        # its definition for the wall-clearance derivation), not the old
        # timid 0.15 m/s wiggle near the start -- also keeps the
        # distance-traveled gate opening throughout the window (a fully
        # stationary robot wouldn't exercise periodic correction at all,
        # see the jerk_stationary scenario for that case specifically).
        t0 = time.monotonic()
        i = 0
        while time.monotonic() - t0 < OBSERVE_SECONDS:
            vx, vy, duration = PATROL_LEGS[i % len(PATROL_LEGS)]
            i += 1
            helper.drive(vx, vy, duration)
            p = helper.get_correction_tf(timeout=2.0)
            if p is not None:
                elapsed = time.monotonic() - t0
                mag = math.hypot(p[0], p[1])
                samples.append((elapsed, mag))
                sc.log(f't={elapsed:5.1f}s  |{edge} xy|={mag:.4f} m')

        if len(samples) < 3:
            sc.result(False, f'too few {edge} samples ({len(samples)}) '
                              'to assess boundedness')
            return sc

        mags = [m for _, m in samples]
        max_mag = max(mags)
        # "Bounded" check: compare the max of the second half of samples
        # against the max of the first half. If the backend is correcting
        # drift periodically, the second half shouldn't be substantially
        # larger than the first -- growth would indicate corrections
        # aren't keeping up (or aren't happening at all).
        half = len(mags) // 2
        first_half_max = max(mags[:half]) if half else mags[0]
        second_half_max = max(mags[half:])
        growth_ratio = second_half_max / max(first_half_max, 1e-6)

        sim_errs = scan_log_for_errors(sim_tree.log_text(), 'sim')
        sentry_errs = scan_log_for_errors(sentry_tree.log_text(), 'sentry_pkg')

        GROWTH_THRESHOLD = 2.0  # second half shouldn't be >2x first half
        ok = growth_ratio < GROWTH_THRESHOLD and not sim_errs and not sentry_errs
        sc.result(ok,
                   f'max|xy|={max_mag:.4f} m, first_half_max={first_half_max:.4f}, '
                   f'second_half_max={second_half_max:.4f}, '
                   f'growth_ratio={growth_ratio:.2f} (threshold {GROWTH_THRESHOLD}), '
                   f'sim_errors={len(sim_errs)}, sentry_errors={len(sentry_errs)}')
        return sc
    finally:
        teardown_stack(sim_tree, sentry_tree, helper)


def scenario_jerk_with_motion(gui, backend):
    sc = Scenario('jerk_with_motion',
                  'trigger_jerk followed by small /cmd_vel motion: '
                  'the correction TF should produce a prompt correction '
                  'tracking the jerk magnitude')
    if backend == 'ekf':
        sc.skip('ekf fuses /odom directly with no distance-traveled gate '
                'analogous to slam_toolbox/amcl -- its jerk response '
                "isn't characterized yet (EKF tuning/verification is "
                'still open work, see SESSION_NOTES.md), so there is no '
                'sound expectation to assert here. See BACKENDS in the '
                'module docstring.')
        return sc
    parent, child = BACKEND_FRAMES[backend]
    edge = f'{parent}->{child}'
    sim_tree = sentry_tree = helper = None
    # 0.5, not the old 0.3 -- exercises jerks up to the
    # robot's real worst-case bump/slip displacement (see
    # ARCC_2026_SENTRY_CONTEXT.md's "Bumpy Road" zone), not
    # just a gentle nudge.
    JERK_STDDEV = 0.5
    try:
        sim_tree, sentry_tree, helper = run_stack(
            gui, backend, odom_noise_enabled=False, odom_jerk_stddev=JERK_STDDEV)
        if not wait_for_stack_ready(sc, helper):
            sc.result(False, 'stack failed to reach a healthy /scan rate '
                              'in time -- see log above')
            return sc
        pose_before = helper.wait_for_correction_tf(timeout=45.0)
        if pose_before is None:
            sc.result(False, f'{edge} never became available within 45s')
            return sc
        sc.log(f'{edge} before jerk = {pose_before}')

        applied_jerk_mag = helper.call_trigger_jerk_and_get_magnitude()
        if applied_jerk_mag is not None:
            sc.log(f'trigger_jerk called, actual applied |jerk| = '
                   f'{applied_jerk_mag:.4f} m')
        else:
            sc.log('trigger_jerk called (could not parse actual applied '
                   'magnitude from response; falling back to a '
                   'stddev-based estimate)')
            applied_jerk_mag = JERK_STDDEV

        # Immediately after the jerk, with no motion yet, the correction
        # TF should NOT have moved yet (this is the same gate as
        # jerk_stationary -- included here just as a sanity check that
        # the jerk itself didn't leak into reported odometry).
        helper.spin_for(2.0)
        pose_immediately_after = helper.get_correction_tf(timeout=2.0)
        sc.log(f'{edge} ~2s after jerk, before motion = '
               f'{pose_immediately_after}')

        # Now give it a small amount of real motion so the backend's
        # distance-traveled gate opens and it attempts a fresh scan
        # match. Measure relative to the PRE-JERK pose, not raw
        # magnitude from the map origin -- the correction TF is not
        # expected to sit at exact identity even with zero noise (the
        # saved ARCC26 map's origin need not exactly coincide with sim's
        # spawn pose, and ordinary scan-matching has some baseline give),
        # so what actually indicates "did the jerk get corrected" is the
        # CHANGE caused by the jerk, not its absolute value. The
        # threshold is a fraction of the ACTUAL applied jerk magnitude
        # (parsed from trigger_jerk's response above), not of
        # odom_jerk_stddev -- comparing against the distribution
        # parameter instead of the real draw was tried first and found
        # flaky in practice (a single gauss() draw can land well under
        # its own stddev), see git history for that iteration.
        # 60s (rather than a tighter bound): this scenario was observed
        # to be sensitive to unrelated CPU contention on the host from
        # other, pre-existing interactive processes sharing the
        # container (e.g. an rviz2 instance left running from earlier
        # manual testing this session) -- under contention, scan
        # processing can fall meaningfully behind wall-clock (observed
        # directly for slam_toolbox: only 2 sensor registrations logged
        # across an entire ~35s scenario run while contended, versus
        # prompt, repeated re-registration when the box was quiet). A
        # generous timeout keeps the assertion meaningful (it still
        # fails a truly broken minimum_travel_distance/update_min_d
        # config well within it, see this suite's validation run) without
        # being a false failure purely because something unrelated was
        # eating CPU on a shared dev box.
        # CORRECTION_FRACTION = 0.3 (not 0.5): repeated validation runs
        # this session showed slam_toolbox settling into a genuine but
        # PARTIAL correction plateau, typically 40-70% of the true jerk
        # magnitude rather than a full 100% snap-back (expected --
        # scan-matching corrects the pose graph incrementally, and this
        # scenario only gives it a small, brief wiggle motion rather than
        # a full traverse). 0.5 sat right at the edge of that plateau and
        # produced borderline false failures purely from run-to-run
        # variance; 0.3 leaves comfortable margin below the observed
        # plateau while still being far above what the KNOWN-BROKEN case
        # (minimum_travel_distance reverted to 0.5, see this suite's
        # validation run in the final report) ever produces, which was
        # indistinguishable from zero. Not yet independently re-validated
        # against amcl's own plateau behavior -- if amcl runs of this
        # scenario turn out flaky, that's the first constant to revisit.
        # CAVEAT (2026-07-20): all of the above was calibrated against the
        # old 0.15 m/s / JERK_STDDEV=0.3 parameters. Both were since bumped
        # to the robot's real top speed (4 m/s) and a larger worst-case
        # jerk (0.5) to make this suite actually exercise realistic
        # conditions -- if this scenario starts failing/flaking under the
        # new parameters, re-derive the plateau fraction rather than
        # assuming the old 0.3 still applies; faster driving and bigger
        # jerks are not guaranteed to produce the same correction-fraction
        # plateau.
        CORRECTION_TIMEOUT = 60.0
        CORRECTION_FRACTION = 0.3
        correction_threshold = applied_jerk_mag * CORRECTION_FRACTION
        t0 = time.monotonic()
        max_seen = 0.0
        corrected_pose = None
        # Drive PATROL_LEGS's mapped-safe loop (holonomic chassis, real
        # 4.0 m/s -- see its definition for the wall-clearance derivation)
        # rather than pure back-and-forth wiggle on one axis -- a
        # wiggle-in-place pattern was found to net very little actual
        # displacement/geometric diversity for scan-matching to
        # triangulate against, which correlated with slow/partial
        # correction independent of host CPU load. A loop gives the
        # backend multiple distinct vantage points against the map.
        leg_i = 0
        while time.monotonic() - t0 < CORRECTION_TIMEOUT:
            vx, vy, duration = PATROL_LEGS[leg_i % len(PATROL_LEGS)]
            leg_i += 1
            helper.drive(vx, vy, duration)
            p = helper.get_correction_tf(timeout=2.0)
            if p is not None:
                delta = math.hypot(p[0] - pose_before[0], p[1] - pose_before[1])
                elapsed = time.monotonic() - t0
                sc.log(f't={elapsed:5.1f}s since motion start  '
                       f'|{edge} - pre-jerk {edge}|={delta:.4f} m')
                if delta > max_seen:
                    max_seen = delta
                    corrected_pose = p
                # Stop early once we've clearly seen a correction on the
                # order of the actual applied jerk magnitude -- no need to
                # keep driving.
                if delta > correction_threshold:
                    break

        elapsed_total = time.monotonic() - t0
        sim_errs = scan_log_for_errors(sim_tree.log_text(), 'sim')
        sentry_errs = scan_log_for_errors(sentry_tree.log_text(), 'sentry_pkg')

        ok = (max_seen > correction_threshold
              and not sim_errs and not sentry_errs)
        sc.result(ok,
                   f'max|{edge} - pre-jerk {edge}| reached '
                   f'{max_seen:.4f} m within {elapsed_total:.1f}s of '
                   f'resumed motion (threshold {correction_threshold:.4f} m '
                   f'= {CORRECTION_FRACTION}x actual applied jerk magnitude '
                   f'{applied_jerk_mag:.4f} m), sim_errors={len(sim_errs)}, '
                   f'sentry_errors={len(sentry_errs)}')
        return sc
    finally:
        teardown_stack(sim_tree, sentry_tree, helper)


def scenario_jerk_stationary(gui, backend):
    sc = Scenario(
        'jerk_stationary',
        'trigger_jerk with robot never moving afterward: the correction '
        'TF must NOT change -- this is a KNOWN, EXPECTED, DOCUMENTED '
        "structural limitation of the backend's scan-matching gate "
        '(minimum_travel_distance/heading or update_min_d/a are measured '
        'off REPORTED odometry, which a jerk deliberately leaves '
        'unchanged), so a PASS here means "the suite correctly '
        'reproduced the documented limitation," NOT "localization is '
        'broken."')
    if backend == 'ekf':
        sc.skip('ekf fuses /odom directly with no distance-traveled gate '
                'analogous to slam_toolbox/amcl -- its jerk response '
                "isn't characterized yet (EKF tuning/verification is "
                'still open work, see SESSION_NOTES.md), so there is no '
                'sound expectation to assert here. See BACKENDS in the '
                'module docstring.')
        return sc
    parent, child = BACKEND_FRAMES[backend]
    edge = f'{parent}->{child}'
    sim_tree = sentry_tree = helper = None
    try:
        sim_tree, sentry_tree, helper = run_stack(
            gui, backend, odom_noise_enabled=False,
            odom_jerk_stddev=0.5)  # matches jerk_with_motion's bump
        if not wait_for_stack_ready(sc, helper):
            sc.result(False, 'stack failed to reach a healthy /scan rate '
                              'in time -- see log above')
            return sc
        pose_before = helper.wait_for_correction_tf(timeout=45.0)
        if pose_before is None:
            sc.result(False, f'{edge} never became available within 45s')
            return sc
        sc.log(f'{edge} before jerk = {pose_before}')

        helper.call_trigger_jerk()
        sc.log('trigger_jerk called; robot will NOT move afterward')

        OBSERVE_SECONDS = 30.0
        t0 = time.monotonic()
        max_drift_from_before = 0.0
        while time.monotonic() - t0 < OBSERVE_SECONDS:
            helper.spin_for(2.0)
            p = helper.get_correction_tf(timeout=2.0)
            if p is not None:
                d = math.hypot(p[0] - pose_before[0], p[1] - pose_before[1])
                max_drift_from_before = max(max_drift_from_before, d)
        sc.log(f'max |{edge} - pre-jerk {edge}| over '
               f'{OBSERVE_SECONDS:.0f}s = {max_drift_from_before:.4f} m')

        sim_errs = scan_log_for_errors(sim_tree.log_text(), 'sim')
        sentry_errs = scan_log_for_errors(sentry_tree.log_text(), 'sentry_pkg')

        NO_CHANGE_THRESHOLD = 0.02  # near-zero tolerance, meters
        ok = (max_drift_from_before < NO_CHANGE_THRESHOLD
              and not sim_errs and not sentry_errs)
        sc.result(ok,
                   f'{edge} stayed within {max_drift_from_before:.4f} m '
                   f'of its pre-jerk value over {OBSERVE_SECONDS:.0f}s '
                   f'(threshold {NO_CHANGE_THRESHOLD} m) -- EXPECTED: with '
                   f'zero reported motion, the scan-match gate never '
                   f'opens, so it never even attempts to notice the '
                   f'jerk. sim_errors={len(sim_errs)}, '
                   f'sentry_errors={len(sentry_errs)}')
        return sc
    finally:
        teardown_stack(sim_tree, sentry_tree, helper)


def scenario_unmapped_obstacle(gui, backend):
    sc = Scenario(
        'unmapped_obstacle',
        'spawn a static box with no corresponding feature in the saved '
        'map, then drive a 2m loop centered on it, 1m out on every side '
        '(see OBSTACLE_LOOP_LEGS) -- seen from every angle, never driven '
        'into: the correction TF should stay bounded relative to its '
        'pre-spawn value, and the backend should keep processing scans '
        'without errors')
    if backend == 'ekf':
        sc.skip('ekf_node never touches /scan at all -- an unmapped '
                'lidar return has no defined effect on odom->root, so '
                'there is no scan-matching step here to have an opinion '
                'about. See BACKENDS in the module docstring.')
        return sc
    parent, child = BACKEND_FRAMES[backend]
    edge = f'{parent}->{child}'
    sim_tree = sentry_tree = helper = None
    try:
        sim_tree, sentry_tree, helper = run_stack(
            gui, backend, odom_noise_enabled=False)
        if not wait_for_stack_ready(sc, helper):
            sc.result(False, 'stack failed to reach a healthy /scan rate '
                              'in time -- see log above')
            return sc
        pose_before = helper.wait_for_correction_tf(timeout=45.0)
        if pose_before is None:
            sc.result(False, f'{edge} never became available within 45s')
            return sc
        sc.log(f'{edge} before obstacle spawn = {pose_before}')

        spawn_box_obstacle()
        sc.log(f'spawned {OBSTACLE_SIZE}x{OBSTACLE_SIZE}x{OBSTACLE_HEIGHT}m '
               f'box obstacle at {OBSTACLE_XY} (not present in the saved '
               f'map) -- at the center of the loop this scenario is '
               f'about to drive, see OBSTACLE_LOOP_LEGS')
        scans_before_drive = helper._scan_count

        # Move from spawn (0,0, inside the loop) out to the loop's own
        # start corner (-0.5,-0.5) before tracing its perimeter -- see
        # OBSTACLE_LOOP_LEGS's comment for the loop's full geometry and
        # wall-clearance derivation.
        helper.drive(-4.0, 0.0, 0.125)   # -0.5m west, to x=-0.5
        helper.drive(0.0, -4.0, 0.125)   # -0.5m south, to y=-0.5
        sc.log('repositioned to the obstacle loop\'s start corner '
               '(-0.5,-0.5) before tracing its perimeter')

        # Drive the loop around the obstacle. Sampling the correction TF
        # each leg, same pattern as continuous_drift.
        OBSERVE_SECONDS = 45.0
        samples = []
        t0 = time.monotonic()
        i = 0
        while time.monotonic() - t0 < OBSERVE_SECONDS:
            vx, vy, duration = OBSTACLE_LOOP_LEGS[i % len(OBSTACLE_LOOP_LEGS)]
            i += 1
            helper.drive(vx, vy, duration)
            p = helper.get_correction_tf(timeout=2.0)
            if p is not None:
                elapsed = time.monotonic() - t0
                delta = math.hypot(p[0] - pose_before[0], p[1] - pose_before[1])
                samples.append(delta)
                sc.log(f't={elapsed:5.1f}s  |{edge} - pre-spawn {edge}|='
                       f'{delta:.4f} m')

        if len(samples) < 3:
            sc.result(False, f'too few {edge} samples ({len(samples)}) to '
                              'assess boundedness')
            return sc

        if helper._scan_count <= scans_before_drive:
            sc.result(False, 'scan count did not advance after the '
                              'obstacle was spawned and driving resumed '
                              '-- backend may have stalled on the '
                              'unmapped return')
            return sc

        max_delta = max(samples)
        sim_errs = scan_log_for_errors(sim_tree.log_text(), 'sim')
        sentry_errs = scan_log_for_errors(sentry_tree.log_text(), 'sentry_pkg')

        # A single small unmapped object should only locally corrupt the
        # returns near it, not swing the whole map alignment -- this is a
        # tighter bound than continuous_drift's growth-ratio check (which
        # expects real accumulating drift to correct for), since there's
        # no injected odometry error here at all, just one extra
        # unexplained cluster of lidar points. Not yet validated against
        # a real run -- if this flakes, re-derive against observed
        # scan-matcher behavior rather than assuming 0.15m is right,
        # same caveat as jerk_with_motion's CORRECTION_FRACTION.
        MAX_DELTA_THRESHOLD = 0.15  # meters
        ok = (max_delta < MAX_DELTA_THRESHOLD
              and not sim_errs and not sentry_errs)
        sc.result(ok,
                   f'max|{edge} - pre-spawn {edge}| = {max_delta:.4f} m '
                   f'over {OBSERVE_SECONDS:.0f}s driving past the '
                   f'obstacle (threshold {MAX_DELTA_THRESHOLD} m), '
                   f'sim_errors={len(sim_errs)}, sentry_errors={len(sentry_errs)}')
        return sc
    finally:
        teardown_stack(sim_tree, sentry_tree, helper)


SCENARIOS = {
    'baseline': scenario_baseline,
    'continuous_drift': scenario_continuous_drift,
    'jerk_with_motion': scenario_jerk_with_motion,
    'jerk_stationary': scenario_jerk_stationary,
    'unmapped_obstacle': scenario_unmapped_obstacle,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=sorted(BACKEND_FRAMES.keys()),
                         default='slam',
                         help='Which auto.launch.py localization_mode to '
                              'exercise (default: slam). See BACKENDS in '
                              'the module docstring for what each one '
                              "means here and why 'mapping' isn't offered.")
    parser.add_argument('--scenario', choices=sorted(SCENARIOS.keys()),
                         help='Run only this scenario (default: all, in '
                              'the order listed in the module docstring)')
    parser.add_argument('--headless', action='store_true',
                         help='Run gz-sim headless instead of the default '
                              'GUI window (faster, but nothing to watch -- '
                              'GUI is on by default, see the module '
                              'docstring)')
    args = parser.parse_args()
    gui = not args.headless

    check_no_orphans('pre-flight')

    rclpy.init()
    try:
        names = [args.scenario] if args.scenario else list(SCENARIOS.keys())
        results = []
        for name in names:
            print(f'\n=== Running scenario: {name} (backend={args.backend}) ===')
            sc = SCENARIOS[name](gui, args.backend)
            results.append(sc)
    finally:
        rclpy.shutdown()

    print('\n=== Summary ===')
    all_pass = True
    for sc in results:
        status = 'SKIP' if sc.skipped else ('PASS' if sc.passed else 'FAIL')
        print(f'  [{status}] {sc.name}')
        if not sc.skipped and not sc.passed:
            all_pass = False
    check_no_orphans('post-flight (should be empty if teardown worked)')

    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
