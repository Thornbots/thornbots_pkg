# sentry_pkg

Hardware interface and robot description for the Thornbots ARC 2026
Sentry robot. Gets `/pose` (real hardware or `sim`) and `/scan` onto the
ROS graph, owns the robot description (`urdf/` + `robot_state_publisher`),
and republishes whatever `odom->root` pose `sentry_localization` computes.
See `sentry_localization/README.md` for the actual localization backends
(SLAM/AMCL/EKF), and the repo-level `ARCC_2026_SENTRY_CONTEXT.md` for the
broader project context.

## What it owns

- `pose_translator`: sole consumer of `/pose`. Publishes `/odom` (raw,
  uncorrected wheel odometry) and `/joint_states`, on the same code path
  whether `/pose` comes from hardware or `sim/pose_emulator.py`. Broadcasts
  no TF itself.
- `odom_tf_broadcaster`: subscribes `/localization/odom` (published by
  `sentry_localization` regardless of backend) and broadcasts the
  `odom->root` TF from it, so this package never needs to know which
  `localization_mode` is running.
- `mcb_relay`: the *only* node allowed to publish or subscribe directly on
  `dji_serial_bridge_node`'s topics, reshaping each upstream package's
  output into what the bridge expects. Three legs: `relocalize`
  (drift-gated `(x, y)` correction), `cv_target`, and `fire_command`. Only
  launched with `real_hardware:=true`. See the `mcb_relay.py` note below.
- `target_selector`: replaces the old C++ `detection_picker_node`. Reads
  `roi_depth_query`'s `/cv/panel_detections` (all detections, post-depth
  3D), applies team filtering, 3D robot grouping, robot-level hysteresis
  and a per-frame pick, then republishes the winner as a singular
  `PanelDetection` on `/cv/panel_detection`. Own `enable_target_selector`
  toggle.
- `target_tracker`: estimates the tracked robot's spin-centre
  position/velocity in `odom` from that pick, publishing `TargetState` on
  `/cv/target_state`. Own `enable_target_tracker` toggle.
- `point_to_cv_target`: turns `/cv/target_state` into the root-frame
  `CVTarget` on `/cv/target`, optionally with an intercept/lead solve
  (`lead_enabled`). `/cv/panel_detection` still supplies confidence, the
  staleness watchdog and the `/cv/panel_polygon` corners; a zero-confidence
  `CVTarget` goes out when the target goes stale or TF fails. Independent
  of `real_hardware` (own `enable_cv_target_bridge` toggle), since
  `/cv/target` feeds both `mcb_relay` and `sim`'s `cv_head_aim`.
- Its own `robot_state_publisher`, off `urdf/sentry.urdf.xacro`.
- Includes `sentry_localization`'s launch file for whichever backend
  `localization_mode` selects.


## Node/topic pipeline

```
/pose --[pose_translator]--> /odom --> sentry_localization --> /localization/odom --[odom_tf_broadcaster]--> odom->root TF
                          \-> /joint_states --[robot_state_publisher]--> rest of TF tree
/scan ------------------------------> sentry_localization (map->odom TF owned directly by slam_toolbox/amcl there)

/localization/odom vs /odom            --[mcb_relay, drift-gated]-------> dji_serial_bridge_node (~/relocalize) --> UART --> MCB
/cv/panel_detections --[target_selector]--> /cv/panel_detection --[target_tracker]--> /cv/target_state
/cv/target_state (position) + /cv/panel_detection (confidence, liveness, corners) --[point_to_cv_target]--\
                                                                    /cv/panel_polygon (rviz/foxglove) <---/
                                                                    /cv/target (root frame) <-------------/
/cv/target --[mcb_relay]--> dji_serial_bridge_node (~/cv_target) --> UART --> MCB
           \-[sim's cv_head_aim]--> /head_pan_cmd, /head_pitch_cmd (sim only, see sim/README.md)
```

## Prerequisites

Run everything below **inside the Isaac ROS dev container** (see the
`isaac-ros-docker` skill for how to launch/attach it), with
the workspace built and sourced:

```bash
isaac_ros_common/scripts/dexec.sh -- bash -c \
  "cd /workspaces/isaac_ros-dev && colcon build --packages-select sentry_pkg sentry_localization && source install/setup.bash"
```

`dexec.sh` already handles env sourcing correctly.
Prefer it over hand-rolled `docker exec`.

## Launching

Everything goes through one launch file, `auto.launch.py`. It includes
`sentry_localization`'s `localization.launch.py` itself, so you don't
need to launch that package separately:

```bash
# Against real hardware (default): also launches dji_serial_bridge_node
# (/pose from the Type-C board's serial link) and sllidar_ros2 (/scan from
# the RPLIDAR A2M8), and runs on wall-clock time.
ros2 launch sentry_pkg auto.launch.py

# Against sim instead (run `ros2 launch sim sim.launch.py` first; it
# provides /pose via pose_emulator.py and /scan itself):
ros2 launch sentry_pkg auto.launch.py real_hardware:=false
```

`real_hardware` also drives `use_sim_time`; there's no separate arg for
it (false/wall-clock when `real_hardware:=true`, true when it's `false`,
since that's exactly when sim's `/clock` exists to use).

### `localization_mode` / `use_ekf`: pick the map->odom owner and the odom->root source

Both forwarded straight through to `sentry_localization`; see
`sentry_localization/README.md` for the full tables and rationale.
`localization_mode` (`amcl` default / `slam` / `mapping` / `none`) picks
who owns `map->odom`; `use_ekf` (default `false`) independently picks
whether `odom->root` is EKF-fused, layerable on top of any
`localization_mode`.

```bash
ros2 launch sentry_pkg auto.launch.py real_hardware:=false localization_mode:=amcl
ros2 launch sentry_pkg auto.launch.py real_hardware:=false localization_mode:=mapping load_map:=false
ros2 launch sentry_pkg auto.launch.py real_hardware:=false localization_mode:=none use_ekf:=true
```

### Other useful args

- `map_file`, `load_map`, `odom_frame` are forwarded to
  `sentry_localization`; see its README for what each controls.
- `lidar_serial_port` / `lidar_baudrate` (defaults `/dev/ttyUSB0` /
  `115200`) are RPLIDAR A2M8 serial settings, only used when
  `real_hardware:=true`. Owned by `sentry_pkg` since it owns the hardware
  drivers.

Full argument docs live in the module docstrings at the top of
`launch/auto.launch.py` (this package) and
`sentry_localization/launch/localization.launch.py` (the included
localization stack).

## Visualizing

This package ships no rviz config; `sim` owns them, and `sim.launch.py`
runs rviz2 itself unless `rviz:=false`. To point one at a hardware run:

```bash
isaac_ros_common/scripts/dexec.sh -- rviz2 -d install/sim/share/sim/rviz/config.rviz
```

## Nodes (`sentry_pkg/`)

- `pose_translator.py`: `/pose` to `/odom` + `/joint_states`.
- `odom_tf_broadcaster.py`: `/localization/odom` to `odom->root` TF.
- `lidar_self_filter.py`: `/scan_raw` to `/scan`, blanking the head's fixed
  blind sector.
- `mcb_relay.py`: sole relay onto `dji_serial_bridge_node`'s topics.
- `target_selector.py`: `/cv/panel_detections` to the singular
  `/cv/panel_detection`.
- `target_tracker.py`: `/cv/panel_detection` to `/cv/target_state`
  (`TargetState`).
- `point_to_cv_target.py`: `/cv/target_state` to `/cv/target` (`CVTarget`,
  root frame) + `/cv/panel_polygon` (`PolygonStamped`).

The last four have pure-logic halves (`target_selector_core.py`,
`target_tracker_core.py`, `point_to_cv_target_core.py`) with no rclpy
import, so they unit-test without a live graph. See "What it owns" above
and the `## Notes` sections below.

## Testing

`test/` unit-tests the `*_core.py` halves against synthetic inputs (no
rclpy, no live topics): `test_target_selector.py` covers
scoring/centrality/grouping/hysteresis, `test_target_tracker.py` the spin
detector/KF/radial correction, `test_point_to_cv_target.py` the intercept
solve and latency stat. Run with `python3 -m pytest test/`, or via `colcon
test --packages-select sentry_pkg`, which also runs the standard
`ament_copyright`/`ament_flake8`/`ament_pep257` checks.

The localization drift/jerk-correction integration suite lives in
`sim/test/localization/run_localization_drift_tests.py` and launches this
package's `auto.launch.py`; see `sim/README.md`'s Testing section.

## Cleaning up

Always fully stop a launch tree (`isaac_ros_common/scripts/kill_launch.sh
<pid>`, not a bare `pkill`) before relaunching. A half-killed tree leaves
duplicate nodes publishing TF, which causes jitter in the next run.

## Notes

Design rationale lives here so the in-code comments can stay short.

### pose_translator.py

Nobody has measured the odom covariance yet. The placeholder is 1cm stddev
on position/velocity with every other field zero. What matters is that it is
non-zero: at all-zero, `robot_localization`'s EKF has no signal that this
source's absolute x/y is any more or less trustworthy than `/scan_odom`'s, so
it cannot weight rf2o's scan-matched estimate more heavily even when it
should. 1cm is a reasonable order of magnitude for per-sample encoder noise
to start from. Unset fields (z/roll/pitch, and yaw, since this chassis is
holonomic and never reports real orientation) stay 0, which is fine because
`odom0_config` in `ekf.yaml` excludes them from fusion.

### target_selector.py

Ports `detection_picker_node.cpp`'s scoring (score = confidence +
`center_weight`*centrality + `priority_class_bonus` for
`priority_class_ids`, `min_score` gating on raw confidence only), with two
deliberate departures:

Centrality is redefined for 3D. The old picker measured pixel distance from
the image centre, which is meaningless post-depth. `centrality_3d()` measures
bearing off the camera's +X axis instead: 1.0 at boresight, clamped to 0.0 at
`centrality_max_angle_rad` (default 45 degrees, about half the camera's ~87
degree horizontal FOV). A point behind the camera (`x<=0`) scores 0 rather
than hitting an undefined `atan2`.

Grouping uses single-linkage clustering at `panel_group_radius_m` (default
0.4), chosen over centroid linkage. Adjacent panels of one robot sit
`hypot(0.30, 0.24) = 0.384m` apart (opposite pairs 0.48-0.60m), so 0.4m links
adjacent pairs and transitivity reaches all four even though only 1-2 are
usually visible. The known failure mode is two robots whose *nearest* panels
fall within 0.4m merging into one cluster; nobody has fixed that, and it is
worth revisiting if it shows up in practice. Centroid linkage trades it for
the opposite failure, splitting one spinning robot's panels as its centroid
wanders.

Clustering runs on camera-frame (x,y,z) directly. That's valid because every
panel in one `PanelDetectionArray` shares a camera pose, so camera-frame
Euclidean distance equals true metric distance.

Hysteresis runs at robot level rather than panel level. Panels of a spinning
robot legitimately vanish every 0.5-1s (145 degree exposure cone, 1-2Hz
spin), so panel stickiness would delay every *correct* handoff.
`RobotHysteresis` tracks the incumbent robot's last centroid; each frame the
cluster nearest that centroid is the incumbent's continuation, and a
challenger must beat it by `switch_margin` for `switch_hold_frames`
consecutive frames before `robot_track_id` changes. Acquisition is immediate
(no incumbent, or the nearest match exceeds `gate_radius_m`); only switching
between two simultaneously-visible candidates is delayed.

No predicted-centre association is wired in, even though `target_tracker`
now exists. The original design had `target_selector` subscribe the
predicted centre to bridge single-panel handoffs, where grouping alone can't
tell a spinning robot's one visible panel changed. But `/cv/target_state` is
in `odom` while this clusters in camera frame deliberately, to avoid a TF
lookup per frame. Wiring it in means either TF-transforming the prediction
into camera frame each frame or moving clustering into `odom`. Both are real
design changes rather than small additions, so this stays optional. Today's
stand-in is `RobotHysteresis`'s own last-centroid continuation, a zero-order
hold without the velocity term: good enough for the common case, weaker
across long handoff gaps.

### target_tracker.py

Consumes `target_selector`'s per-frame pick, estimates the tracked robot's
spin-centre in `odom`, publishes `TargetState` on `/cv/target_state`. Pure
logic in `target_tracker_core.py`, unit-tested in
`test/test_target_tracker.py`.

Filtering happens in `odom` rather than `root` or camera. `root` translates
with the sentry, so a constant-velocity model there breaks under sentry
acceleration, and camera additionally rotates with the gimbal.
`lookupTransform(odom, camera, detection_stamp + pose_latency_s)` gets both
corrections at once. `pose_latency_s` (0.01s) exists because
`dji_serial_bridge_node.cpp`'s `handle_pose()` stamps `RobotPose` with
`now()` at parse time rather than MCB sample time; shifting the query time
absorbs that bias. Nobody has measured it. 0.01s is a placeholder inside the
documented 3-25ms range, and the real value needs a sweep on hardware.

TF failure is loud. A `TransformException` logs an error and drops the
detection, never falling back to a stale or zero transform. The whole
odom-frame filter rests on this lookup succeeding on hardware, and
degrading silently would let a broken TF tree pass in sim (where
`robot_state_publisher` always happens to be running) and fail invisibly at
the competition.

`SpinDetector` picks the spin/no-spin branch off the timing between
`class_id` changes, not any single interval: it needs `spin_min_handoffs`
(3) roughly equal intervals (coefficient of variation under `spin_cv_max`,
0.35) to call a target spinning, and falls back if `class_id` hasn't changed
in `spin_handoff_timeout_s` (1.5s). `spin_hz` assumes one handoff per
quarter-revolution, so it conflates handoff period with quarter-revolution
period and cannot tell spin direction. That coarseness is fine, because the
only thing gated on it is the binary branch. `spin_phase` is likewise
re-derived from time-since-handoff rather than phase-locked, and nothing
consumes it.

The radial correction deliberately avoids `PanelDetection.corners`. The
spinning branch is a running mean of panel positions corrected for the
exposure-cone bias (only near-facing panels are visible, skewing the raw arc
toward the camera). An earlier version took a plane normal from the panel's 4
corners via one cross product, which is wrong rather than merely unnecessary:
`roi_depth_node.cpp`'s `deprojectDetection()` deprojects all 4 corners at one
shared `mean_depth_m` (its own "planar assumption"), making every real quad
exactly fronto-parallel by construction, so the cross product always comes
out as exactly the camera's boresight axis. For an off-boresight panel that
points somewhere else entirely from "back toward the camera along *this*
panel's bearing." `cv_target_emulator.py`'s corners *do* encode real tilt, so
the corner approach would have worked in sim and silently produced a
different wrong answer on hardware. That is a sim/hardware divergence through
geometry rather than `frame_id`, and a sim-only hit-rate cannot catch it.

`corrected_centre()` instead pushes the panel position further along its own
camera-to-panel ray by `panel_radius_m`, approximating the chassis centre as
sitting directly behind the visible panel. No corners, no normal, no
plane-fitting (out of scope: only viable under ~2m, where depth noise is
below the panel's tilt). Treat it as the honest answer for now, since a
better version needs panel-orientation information the hardware pipeline does
not produce anywhere. `estimator` stays `0` (`running_mean`) always; the
width-refined (`1`) branch is unimplemented and gated behind a verification
pass against the emulator's known panel normal, keeping the running mean if
the refinement doesn't beat it.

`panel_radius_m` (0.27, the mean of 0.30 and 0.24) is a single scalar rather
than per-face. `class_id` encodes team and plate digit but not which face is
visible, so there is no signal to pick between them.

The KF is 6-state constant-velocity with `R` scaled by range:
`meas_noise_base_m + meas_noise_range_coeff * range_m^2`, following the
depth-error model (depth ~z^2, lateral pixel ~z, dominated by z^2 at any
real range). `spin_meas_inflation` defaults to 1.0, since a running mean of
~30 samples is *less* noisy than a single sample and inflating `R` for the
spinning branch would be backwards. It exists for a different, real concern:
the running mean lags a rotating orbit, averaging positions up to
`spin_window_s` old against a centre that is still moving. That is a bias in
the estimate rather than extra variance, so inflating `R` does not fix it
either. It is flagged here rather than silently defaulted to a number the
code cannot justify.

Reset (fresh KF, cleared spin history and window) happens only on a
`robot_track_id` change or a `track_max_gap_s` gap, never on a plain
`class_id` handoff. `valid` goes true after 2 KF updates rather than waiting
for a spin period to converge, since a real engagement can be shorter than
one period and lead must still be available. The consumer weighs the published
`variance`, which stays large right after a reset.

### mcb_relay.py

Only `sentry_pkg` publishes or subscribes directly on
`dji_serial_bridge`'s topics; the bridge stays a pure UART/DJI-protocol
translator with nothing else wired to it. This node reshapes each upstream
package's output into what `dji_serial_bridge_node` expects.

`relocalize` compares `/localization/odom` (`sentry_localization`'s one
guaranteed output, published regardless of `localization_mode`/`use_ekf`)
against `/odom` (the MCB's raw uncorrected wheel odometry). Deliberately
backend-agnostic: two Odometry topics, no TF lookups, no assumption about
which backend runs. Once they drift past `error_threshold_meters` *and* the
chassis is nearly stationary (raw odom speed below `max_move_speed`, so the
correction isn't stale by the time the MCB applies it), it publishes the
localized `(x, y)` as a `Point` on `~/relocalize`, which the bridge packs
into a `RelocalizePayload` so the MCB can reset its odometry origin.

`cv_target` and `fire_command` are straight republishes onto the bridge's
`~/cv_target` and `~/fire_command`.

### lidar_self_filter.py

The lidar is mounted rigidly on the head, so head and lidar always rotate
together. Whatever part of the head blocks the lidar's view sits at a
**fixed angle in the lidar's own frame** regardless of headlink's yaw, even
though the blocked world-frame bearing sweeps around as the head rotates.
That fixed relationship is what makes a static angular filter viable with no
joint-state subscription.

It runs for both sim and hardware. Sim's `gpu_lidar` is a rendering sensor
with no physics collision, so modelling this in the URDF instead (gz-sim's
`visibility_mask`/`visibility_flags`) was unreliable: all-or-nothing per
visual, either seeing straight through the head even where it should
genuinely occlude, or reporting self-hits, with no way to express "block the
beam here without counting it as a detection of the head." Real hardware has
no such trick at all. A software filter with a known blind sector is the one
approach that works for both.

The blind sector is wider than the literal self-hit cluster sim produces. A
solid real head blocks its whole angular footprint, while sim's mesh only
registers a self-hit at its tangent edge against the scan plane. Beams aimed
through the head's bulk pass clean through a thin, non-watertight STL and
"see" whatever is beyond it, which real hardware would never allow. So the
sector is sized to the head's real angular footprint rather than to where
`/scan_raw` happens to show a close return.

Current values (1.0 rad wide): the raw self-hit cluster measured roughly
2.967-3.022 rad, and `blind_angle_end=3.20` lines up with where sim's mesh
lets real wall hits back through (from ~3.024), so that edge is left alone.
`blind_angle_start` is widened well before the cluster's start, down to 2.20,
to approximate the real head's full width. Both numbers are still derived
from the sim mesh, so retune against a real `/scan_raw` capture before
trusting them on hardware.

### point_to_cv_target.py

Turns `target_tracker`'s odom-frame `/cv/target_state` into a root-frame
position on `/cv/target`, with an optional intercept/lead solve. `x/y/z` is
now a position Type-C aims at directly rather than a camera-relative vector.
That semantic change matters more than the byte count (see `CVTarget.msg` and
`ros2_dji_serial_bridge/README.md`'s wire-format history).

The node takes two upstream inputs, split by what they carry.
`target_state_topic` has position/velocity/validity but no confidence field,
so `panel_topic` still
drives confidence caching, the `target_timeout_s` watchdog, and the
`polygon_topic` publish (raw corners for rviz/foxglove). If
`target_state_topic` has never published, `/cv/target` reports zero
confidence even with a live `panel_topic`. This node needs `target_tracker`
in the pipeline, not `target_selector` alone.

Publishing runs on a timer at `cv_target_publish_rate_hz` (30) rather than
per-message, because the tracker runs at detection rate (up to ~60Hz) and
Type-C's PID doesn't need a setpoint that fast. Each tick re-reads the latest cached
state rather than reacting to a subscription.

Three cases per tick in `_compute_aim_point()`:

- No `TargetState` yet, or the TF lookup fails: return `None` and publish
  zero confidence. TF failure logs an `ERROR` (throttled), same reasoning as
  `target_tracker`'s lookup.
- `valid == False`: emit the raw `panel` field, `lead_applied=False`,
  `track_valid=False`. It never extrapolates off an unconverged track, since
  a stub fire-trigger would otherwise shoot at a guess.
- `valid == True`: emit the KF `centre` transformed into root, running
  `solve_intercept()` first if `lead_enabled`. That solve is a 2-3 iteration
  fixed-point time-of-flight loop with no gravity, drag or elevation, since
  Type-C owns those. `lead_enabled` is one param flip between before and
  after for a hit-rate sweep.

tau is the measured `now - detection_stamp` running mean (`LatencyStat`,
updated on every `TargetState`) plus `firmware_latency_s`, a placeholder
that needs measuring on hardware. That running mean is the repo's first real
latency number, and it's logged rather than only used internally.

Frame conversion goes through a TF lookup rather than a fixed axis swap.
`lookup_transform(root_frame, odom_frame, Time())` converts the odom-frame
point into root. For the lead solve a second lookup the other way gives the
shooter's position in odom, and `RobotPose.vel_x/vel_y` rotated by that same
transform gives its velocity, since the sentry keeps moving during flight.
Both use "latest available" rather than the detection stamp, because what
matters is where the shooter is *now*.

### auto.launch.py

`real_hardware:=true` is the default because that's the common case. Against
sim, pass `real_hardware:=false` so it doesn't open the real serial devices
and so it uses sim's `/clock`. That one arg drives `use_sim_time` too.

Everything else this launch file wires up is described in `## What it owns`
above.