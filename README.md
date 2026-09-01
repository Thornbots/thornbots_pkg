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
  uncorrected wheel odometry) and `/joint_states`. Same code path whether
  `/pose` comes from real hardware or `sim/pose_emulator.py`. No longer
  broadcasts any TF itself.
- `odom_tf_broadcaster`: subscribes `/localization/odom` (published by
  `sentry_localization`, regardless of which backend is active) and
  broadcasts the `odom->root` TF from it. This is the "republish" step:
  `sentry_pkg` never needs to know which `localization_mode` is running.
- `mcb_relay`: the *only* node allowed to publish/subscribe directly on
  `dji_serial_bridge_node`'s topics. `dji_serial_bridge` stays a pure
  UART/DJI-protocol translator; anything that wants to send something to the
  Type-C/MCB board goes through this node instead, which reads each upstream
  package's own output and reshapes it into whatever
  `dji_serial_bridge_node` expects:
  - `relocalize`: compares `/localization/odom` (`sentry_localization`'s one
    guaranteed output, the same topic regardless of `localization_mode`, no
    TF lookups, no backend-specific assumptions) against `/odom`
    (`pose_translator`'s raw MCB odometry). Once they've drifted apart past
    a threshold and the chassis is nearly stationary, sends the corrected
    `(x, y)` to `dji_serial_bridge_node`'s `~/relocalize`.
  - `cv_target`: republishes `/cv/target` (`CVTarget`, from
    `point_to_cv_target` below) unchanged onto `dji_serial_bridge_node`'s
    `~/cv_target`.
  - `fire_command`: republishes `/sentry/fire_command` (`FireCommand`)
    unchanged onto `/dji_serial_bridge/fire_command`. Scaffolding on both
    ends: the only publisher today is `point_to_cv_target`'s placeholder
    trigger, and `dji_serial_bridge_node` has no subscription for it yet, so
    nothing reaches the MCB. See `AGENTS.md`'s Open list.
  Only launched alongside `dji_serial_bridge_node` (`real_hardware:=true`).
- `target_selector`: replaces the old C++ `detection_picker_node`. Reads
  `roi_depth_query`'s `/cv/panel_detections` (`PanelDetectionArray`, ALL
  detections, post-depth 3D), applies team filtering + 3D robot grouping +
  robot-level hysteresis + per-frame panel pick, and republishes the winner
  as a singular `PanelDetection` on `/cv/panel_detection`, consumed by
  `target_tracker` (and, for confidence/liveness/polygon, by
  `point_to_cv_target`). Own `enable_target_selector` toggle. See the
  `target_selector.py` note below.
- `target_tracker`: estimates the tracked robot's spin-centre
  position/velocity in `odom` from `target_selector`'s per-frame pick, and
  publishes `TargetState` on `/cv/target_state`. Own `enable_target_tracker`
  toggle. See the `target_tracker.py` note below.
- `point_to_cv_target`: turns `target_tracker`'s `/cv/target_state`
  (odom-frame KF estimate) into the root-frame `CVTarget` published on
  `/cv/target`, optionally with an intercept/lead solve (`lead_enabled`).
  `/cv/panel_detection` still supplies confidence, the staleness watchdog,
  and the `PolygonStamped` on `/cv/panel_polygon` (the panel corners,
  unmodified, for visualization/future firing logic); a zero-confidence
  `CVTarget` goes out when the target goes stale or the TF lookup fails.
  Runs independently of `real_hardware` (its own `enable_cv_target_bridge`
  toggle only) since `/cv/target` is consumed both by `mcb_relay` (real
  hardware) and `sim`'s `cv_head_aim` node (sim).
- Its own `robot_state_publisher`, off `urdf/sentry.urdf.xacro`.
- Includes `sentry_localization`'s launch file to bring up whichever
  localization backend `localization_mode` selects.

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

This package ships no rviz config of its own. `sim` owns them
(`config.rviz`, `cv_target.rviz`), and `sim.launch.py` runs rviz2 itself
unless `rviz:=false`. To point one at a real-hardware run:

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

The localization drift/jerk-correction integration suite now lives in
`sim/test/localization/run_localization_drift_tests.py`
(it launches this package's `auto.launch.py`, which includes
`sentry_localization`). See `sim/README.md`'s Testing
section.

Standard `colcon test`-style checks (`ament_copyright`/`ament_flake8`/
`ament_pep257`) apply via the normal `colcon test --packages-select
sentry_pkg`.

`test/` unit-tests the `*_core.py` halves against synthetic inputs (no
rclpy, no live topics): `test_target_selector.py` covers
scoring/centrality/grouping/hysteresis, `test_target_tracker.py` the
spin detector/KF/radial correction, `test_point_to_cv_target.py` the
intercept solve and latency stat. Run them directly with `python3 -m
pytest test/` or via `colcon test`.

## Cleaning up

Always fully stop a launch tree (`isaac_ros_common/scripts/kill_launch.sh
<pid>`, not a bare `pkill`) before relaunching. A half-killed tree leaves
duplicate nodes publishing TF, which causes jitter in the next run.

## Notes

Design rationale and history trimmed out of in-code comments/docstrings to
keep those short, kept here for anyone who needs the full context.

### pose_translator.py

The odom covariance placeholder (1cm stddev on position/velocity, all other
fields zero) is a first-pass value, not measured/validated (same disclaimer
as `config/ekf.yaml`'s `process_noise_covariance`). Non-zero is the
important part. Left at all-zero, `robot_localization`'s EKF has no signal
that this source's absolute x/y is any more or less trustworthy than
`/scan_odom`'s, so it can't weight rf2o's scan-matched estimate more heavily
even when it should. 1cm stddev is a reasonable per-sample encoder-noise
order of magnitude to start from. Unset fields (z/roll/pitch, and yaw, since
this chassis is holonomic and never reports real orientation) are left at 0,
which is fine since `odom0_config` in `ekf.yaml` excludes them from fusion.

### target_selector.py

Ported from `detection_picker_node.cpp`'s scoring (score = confidence +
`center_weight`*centrality + `priority_class_bonus` added conditionally for
`priority_class_ids`, `min_score` gating on raw confidence only), but two
things changed deliberately rather than being ported byte-for-byte:

- Centrality is redefined for 3D. The old picker measured distance from
  the image centre in pixels (pre-depth, network space). Post-depth, that's
  meaningless, so `centrality_3d()` instead measures bearing off the camera's
  +X (forward) axis, 1.0 at boresight, clamped to 0.0 at `centrality_max_angle_rad`
  or beyond (default 45°, ballpark half of the camera's ~87° horizontal FOV).
  A point behind the camera (`x<=0`) scores 0 rather than hitting an
  undefined `atan2`.
- Grouping is single-linkage clustering at `panel_group_radius_m` (default
  0.4m), chosen explicitly over centroid linkage. Adjacent panels of one
  robot sit `hypot(0.30, 0.24)=0.384m` apart (opposite pairs 0.48-0.60m), so
  0.4m links adjacent pairs and single-linkage transitivity reaches all 4
  panels even though usually only 1-2 are visible at once. That's the
  behaviour we want. The known failure mode is two robots whose *nearest*
  panels happen to fall within 0.4m merging into one cluster. Not fixed
  here; revisit if it shows up in practice (e.g. switch to centroid linkage,
  which trades the opposite failure of legitimately splitting one spinning
  robot's panels across frames as its centroid estimate wanders).

Clustering runs on camera-frame (x,y,z) directly, not a world/odom frame.
This is valid because every panel in one `PanelDetectionArray` shares the same
camera pose, so Euclidean distance in camera frame equals true metric
distance (a rigid transform preserves it).

Hysteresis is at robot level, not panel level. Panels of a spinning robot
legitimately vanish every 0.5-1s (145° exposure cone, 1-2Hz spin), so panel
stickiness would delay every *correct* handoff. `RobotHysteresis` tracks
the incumbent robot's last centroid; each frame, the cluster nearest that
centroid is the incumbent's continuation, and a challenger must beat it by
`switch_margin` for `switch_hold_frames` consecutive frames before the
`robot_track_id` actually changes. Track *acquisition* (no incumbent yet,
or the incumbent's nearest match exceeds `gate_radius_m`) is immediate.
Only switching between two simultaneously-visible candidates is delayed.

No predicted-centre association is wired in yet, even though
`target_tracker.py` now exists. The original plan called for
`target_selector` to subscribe `target_tracker`'s predicted centre to bridge
single-panel handoffs (when a spinning robot shows only one panel and it
changes, grouping alone can't tell it's the same robot).
`target_tracker.py`'s `/cv/target_state` is in the `odom` frame (it needs
`odom` to filter correctly through sentry motion, see its own Notes below),
but `target_selector` clusters in camera frame deliberately, to avoid
needing a TF lookup per frame (every panel in one `PanelDetectionArray`
shares one camera pose, so camera-frame distance is already true metric
distance). Wiring the prediction in would mean either TF-transforming the
predicted centre into camera frame each frame, or moving clustering into
`odom` frame. Both are real design changes, not a small addition, so this
stays an optional improvement and never a requirement. Today's stand-in is
`RobotHysteresis`'s own last-known-centroid continuation (nearest cluster to
the incumbent's last position), a zero-order-hold predictor without
`target_tracker`'s velocity term. It's good enough for the common case but
weaker than a real predicted-centre association during long handoff gaps.

### target_tracker.py

WHERE IT'S GOING: consumes `target_selector`'s per-frame panel pick
(`/cv/panel_detection`), estimates the tracked robot's spin-centre in
`odom`, publishes `dji_serial_bridge/msg/TargetState` on
`/cv/target_state`. Pure logic lives in `target_tracker_core.py` (no
rclpy import), unit-tested in `test/test_target_tracker.py`, same split as
`target_selector`/`target_selector_core`.

Filtering happens in `odom`, not `root` or camera. `root` translates with
the sentry, so a constant-velocity model there breaks under sentry
acceleration; camera additionally rotates with the gimbal.
`lookupTransform(odom, camera, detection_stamp + pose_latency_s)` gets both
corrections in one step. `pose_latency_s` (default 0.01s) exists because
`dji_serial_bridge_node.cpp`'s `handle_pose()` stamps `RobotPose` with
`now()` at parse time, not MCB sample time. Shifting the TF query time is
how that bias gets absorbed. The right value has to be found empirically on
hardware (tune-by-sweep); 0.01s is a placeholder within the documented
3-25ms range, not a measured number.

TF failure is loud, not silent. A `TransformException` (most likely,
`camera_frame` from the detection header isn't in the TF tree, or the
requested stamp isn't covered yet) logs an error and drops the detection. It
never falls back to a stale or zero transform. This is deliberate: the whole
odom-frame filter rests on this lookup succeeding on hardware, and silently
degrading here would let a broken TF tree pass CI in sim (where
`robot_state_publisher` happens to always be running) and then fail
invisibly at the competition.

`SpinDetector` decides the spin/no-spin branch. A spinning target's
visible panel changes as `class_id` cycles; `SpinDetector` looks at the
timing between those changes, not any single interval. It needs
`spin_min_handoffs` (default 3) roughly-equal-length intervals (coefficient
of variation under `spin_cv_max`, default 0.35) before it calls a target
"spinning", and falls back to "not spinning" if `class_id` hasn't changed
in `spin_handoff_timeout_s` (default 1.5s). `spin_hz` assumes exactly one
handoff per quarter-revolution (4 armor panels), which conflates the true
handoff period with the quarter-revolution period and can't tell spin
direction. It's coarse by design, since the only thing gated on it is the
binary branch choice, not `spin_hz`'s absolute accuracy. `spin_phase` is
re-derived from time-since-last-handoff modulo the estimated period, not a
phase-locked estimate, also coarse, and downstream nothing currently
consumes it besides the wire-excluded `TargetState` field itself.

Per-panel radial correction replaces literal orbit averaging, and
deliberately does NOT use `PanelDetection.corners` for it. The spinning
branch is a running mean of panel positions corrected for the exposure-cone
bias: only near-facing panels are visible, which skews the raw arc toward
the camera. An earlier version of this correction used the panel's 4 corners
(a real plane normal via one cross product). That turned out to be dead
weight, not just unnecessary: `roi_depth_node.cpp`'s `deprojectDetection()`
deprojects all 4 corners at one shared `mean_depth_m` (its own comment calls
this the "planar assumption"), which makes every real detection's quad
exactly fronto-parallel to the camera by construction. The cross product of
two edges in that plane is always exactly the camera's boresight axis. For
an off-boresight panel that's a different (and wrong) direction than "back
toward the camera along *this panel's* bearing," not just a less precise
version of the right answer. `cv_target_emulator.py`'s corners *do* encode
real tilt (built from the true canted `right_dir`/`up_dir`), so the corner
approach would have worked in sim and silently produced a different wrong
answer on hardware. That's a sim/hardware divergence via geometry instead of
`frame_id`, and exactly the kind of thing a sim-only hit-rate can't catch.

`corrected_centre()` now just pushes the panel position further along its
own existing camera-to-panel ray by `panel_radius_m`, approximating the
chassis centre as sitting directly behind the visible panel along the
current line of sight. No corners, no normal estimate, no plane-fitting (out
of scope: only viable under ~2m, where depth noise is below the panel's
tilt). This is a deliberately honest fallback, not a stand-in for something
better. The "better" version needs real panel-orientation information the
pipeline doesn't currently produce anywhere on hardware. The spinning branch
still runs a `spin_window_s` (default 0.5s) running mean *of these corrected
points*, to smooth single-frame range noise. `estimator` in `TargetState`
stays `0` (`running_mean`) always. The width-refined (`1`) branch is not
implemented. It stays gated behind a verification pass: a live sim run
against the emulator's known panel normal, keeping the running mean if the
refinement doesn't beat it.

`panel_radius_m` (default 0.27, the mean of `panel_radius_x=0.30` and
`panel_radius_y=0.24`) is a single scalar, not per-panel-face like the
emulator's `radius_x`/`radius_y` split. `PanelDetection.class_id` encodes
team + plate digit, not which face is visible, so there's no signal here
to pick `radius_x` vs `radius_y` from. Approximation, same spirit as
`PANEL_SIZE`'s own hedge in `cv_target_emulator.py`.

The KF is 6-state constant-velocity, with `R` scaled by range.
`meas_noise_base_m + meas_noise_range_coeff * range_m^2` follows the
depth-error model (depth error ~z², lateral pixel error ~z; dominated by the
z² term at any real range). `spin_meas_inflation` defaults to `1.0` (no
inflation), since a `spin_window_s` running mean of ~30 samples is *less*
noisy than a single raw sample, so inflating `R` for the spinning branch by
default would be backwards. It exists as a tunable knob for a different,
real concern: the running mean can lag a rotating orbit (it's averaging
positions from up to `spin_window_s` in the past against a centre that's
still translating/spinning), which is a bias more than a variance and isn't
actually fixed by inflating `R` either. It's flagged here rather than
silently defaulted to a number the code doesn't justify. Reset (fresh KF,
cleared spin history, cleared running-mean window) happens only on a
`robot_track_id` change or a `track_max_gap_s` gap, never on a plain
`class_id` handoff.

`valid` goes true after 2 KF updates, not after a spin period converges. A
real engagement can be shorter than a full spin period, and lead must still
be available then. The consumer (`point_to_cv_target.py`'s intercept solver)
is expected to weigh the published `variance`, which stays large immediately
after a reset.

### mcb_relay.py

Sole relay between `sentry_pkg`/`sentry_localization`/CV and
`dji_serial_bridge_node`'s topics. Per project convention, only
`sentry_pkg` is allowed to publish/subscribe directly on
`dji_serial_bridge`'s topics. `dji_serial_bridge` itself stays a pure
UART/DJI-protocol translator, with no other package wired to it. This node
reads each upstream package's own output and reshapes it into whatever
`dji_serial_bridge_node`'s subscription expects.

`relocalize`: compares `localization_odom_topic` (`/localization/odom`,
`sentry_localization`'s one guaranteed output, published regardless of
`localization_mode`/`use_ekf`) against `raw_odom_topic`
(`/odom`, `sentry_pkg`'s own `pose_translator`, i.e. the MCB's raw
uncorrected wheel odometry). Deliberately backend-agnostic: no TF lookups,
no assumption about which `localization_mode`/`use_ekf` combination is
running, just two Odometry topics. Once they've drifted apart by more than
`error_threshold_meters` *and* the chassis is nearly stationary (raw odom
speed below `max_move_speed`, so the correction isn't stale by the time
the MCB applies it), it publishes the localized `(x, y)` as a `Point` on
`relocalize_output_topic`, which is `dji_serial_bridge_node`'s `~/relocalize`
(default resolved name `/dji_serial_bridge/relocalize`). That node packs it into a
`RelocalizePayload` and sends it over UART so the MCB can reset its own
odometry origin.

`cv_target`: republishes `cv_target_input_topic` (`dji_serial_bridge/msg/
CVTarget`, from the CV pipeline) unchanged onto `cv_target_output_topic`,
which is `dji_serial_bridge_node`'s `~/cv_target` (default resolved name
`/dji_serial_bridge/cv_target`).

### lidar_self_filter.py

The lidar is mounted rigidly on the head (see
`sim/urdf/sentry.urdf.xacro`'s `lidarlink` joint), so head and lidar
always rotate together as one unit. Whatever part of the head's own
structure blocks the lidar's view sits at a FIXED angle in the lidar's own
frame regardless of headlink's current yaw, even though that blocked
WORLD-frame bearing sweeps around as headlink rotates (see
`sim/head_sweep.py`'s docstring: sweeping headlink moves this blind wedge
around the world so SLAM eventually gets full coverage). That fixed
relationship is what makes a static angular filter here viable at all, with no
joint-state subscription needed.

Runs for both sim and real hardware (see `sentry_pkg/launch/auto.launch.py`):
sim's `gpu_lidar` is a rendering sensor with no physics collision to fall
back on, so trying to model this via mesh visibility in the URDF instead
(gz-sim's `visibility_mask`/`visibility_flags`) was unreliable. It could
only ever be all-or-nothing per visual (either the whole head is invisible
to the lidar, seeing straight through it even where it should genuinely
occlude, or fully visible and back to reporting self-hits) and had no way
to express "block the beam here without counting it as a false detection
of the head's own mesh". Real hardware has no such trick available at all
(it's a physical beam). A software filter with a known fixed blind sector
is the one approach that actually works for both.

The blind sector is wider than the literal near-range self-hit cluster
sim's mesh happens to produce. A solid real head blocks its whole real
angular footprint outright, but sim's mesh only registers as a self-hit
right at its tangent edge against the lidar's exact scan plane. Beams
aimed more centrally through the head's bulk can pass clean through a
thin/non-watertight STL without registering any hit at all, "seeing"
whatever real geometry (walls, etc.) sits beyond the head, which the real
hardware's fully-solid head would never let through. So this sector isn't
just "wherever `/scan_raw` shows a close self-hit"; it's sized to the
head's actual real-world angular footprint from the lidar's vantage point.

Current values (1.0 rad wide): the raw self-hit cluster measured roughly
2.967-3.022 rad, and `blind_angle_end=3.20` lines up with where sim's mesh
already lets real wall hits back through (from ~3.024, right after the
cluster), so that edge is left alone. `blind_angle_start` is widened well
before the cluster's own start (2.967), down to 2.20, to approximate the
real head's full width, since sim's mesh only registers a self-hit right
at its tangent edge and lets real wall hits through everywhere else in the
head's true angular footprint. These are still sim-mesh-derived estimates
and will need retuning against a real `/scan_raw` capture before trusting
them on real hardware.

### point_to_cv_target.py

WHERE TO AIM: turns `target_tracker`'s `/cv/target_state` (odom-frame
spin-centre KF estimate) into a root-frame position on `/cv/target` for
`mcb_relay`/sim's `cv_head_aim`, with an optional intercept/lead solve. This
is a frame and content change from the old camera-frame-offset `CVTarget`
(see `CVTarget.msg` and `ros2_dji_serial_bridge/README.md`'s wire-format
history). `x/y/z` is now a position Type-C aims at directly, not a
camera-relative vector, and that's the semantic change that matters, not the
byte count.

Two upstream inputs, split by what they carry. `target_state_topic`
(`dji_serial_bridge/msg/TargetState`, default `/cv/target_state`) has
position/velocity/validity but no confidence field; `panel_topic`
(`dji_serial_bridge/msg/PanelDetection`, default `/cv/panel_detection`)
still drives confidence caching, the `target_timeout_s` liveness watchdog,
and the unchanged `polygon_topic` (`geometry_msgs/PolygonStamped`, the raw
corners for rviz/foxglove) publish. If `target_state_topic` has never
published, `/cv/target` reports zero confidence even if `panel_topic` is
live. This node now depends on `target_tracker` being in the pipeline,
not just `target_selector`.

Publishing runs on a timer at `cv_target_publish_rate_hz` (default 30), not
per-message. The tracker runs at detection rate (up to ~60Hz); Type-C's PID
doesn't need a setpoint that fast. Each tick re-reads the latest cached
`TargetState`/confidence rather than reacting to either subscription
directly.

Three cases per tick, in `_compute_aim_point()`:
- No `TargetState` ever received, or the latest TF lookup fails: `None`,
  caller publishes zero-confidence. TF failure is logged as an `ERROR`
  (throttled), never silently swallowed, same reasoning as
  `target_tracker`'s TF lookup.
- `TargetState.valid == False`: emit the raw `panel` field (unconverged,
  unfiltered position), `lead_applied=False`, `track_valid=False`. Never
  extrapolate off an unconverged track, since a stub fire-trigger would
  otherwise shoot at a guess.
- `TargetState.valid == True`: emit the KF `centre` transformed into root.
  If `lead_enabled` (default `False`) is also set, run
  `point_to_cv_target_core.solve_intercept()` first, a 2-3 iteration
  fixed-point time-of-flight solve, no gravity/drag/elevation (Type-C owns
  those). `lead_enabled` is one param flip between "before" and "after" for
  a hit-rate sweep.

tau (total latency) is the measured `now - detection_stamp` running mean
(`point_to_cv_target_core.LatencyStat`, updated on every `TargetState`
arrival) plus `firmware_latency_s` (a placeholder param, needs measuring on
real hardware). This running mean is the repo's first real latency
number, logged, not just used internally.

Frame conversion is a TF lookup now, not a fixed axis swap.
`lookup_transform(root_frame, odom_frame, Time())` (latest available, not a
future extrapolation) converts the odom-frame KF/panel point into root; for
the lead solve, a second lookup in the other direction (`odom_frame` <-
`root_frame`) gives the shooter's current position in odom, and
`RobotPose.vel_x/vel_y` (subscribed on `robot_pose_topic`, default `/pose`)
rotated into odom by that same transform's orientation gives the shooter's
velocity for the fixed-point solve, since the sentry itself keeps moving
during the flight (filter in odom, emit in root). Both lookups use "latest
available" rather than the detection stamp, since we want where the shooter
physically is *now*, not at detection time.

### auto.launch.py

`real_hardware:=true` is the default because running against real hardware
is the common case. When running against sim instead (`ros2 launch sim
sim.launch.py`, which runs `sim/pose_emulator.py` to publish `/pose` in
the same `dji_serial_bridge/msg/RobotPose` format real hardware sends,
plus raw `/scan` via its own gz bridge), launch this with
`real_hardware:=false` so it doesn't also try to open the real serial
devices, and so it uses sim's `/clock`.

`pose_translator` (fed by `/pose`) turns raw hardware pose into `/odom`
(raw, uncorrected wheel odometry) and `/joint_states`, on the same code path for
sim and real hardware, so there's only one place pose handling can go
wrong. This package also runs its own `robot_state_publisher` off
`sentry_pkg/urdf/sentry.urdf.xacro` (fed by `pose_translator`'s
`/joint_states`) rather than depending on sim's URDF/TF. `sentry_pkg`
owns the whole TF tree itself, and sim only ever provides raw sensor data
through the shared real-hardware-shaped interfaces.

`sentry_pkg` no longer computes `odom->root` itself: `/odom` + `/scan` are
handed to `sentry_localization` (included in the launch file), which
always publishes the localized result on `/localization/odom` regardless
of which backend `localization_mode` selects. `odom_tf_broadcaster`
subscribes that topic and broadcasts the actual `odom->root` TF, so this
package never needs to know which `localization_mode` is active.

`mcb_relay` reshapes each upstream package's own output into what
`dji_serial_bridge_node` expects: `/localization/odom` vs `/odom`
(drift-gated) -> `~/relocalize`, and the CV pipeline's CVTarget ->
`~/cv_target`. Backend-agnostic by construction, with no TF lookups and no
assumption about which `localization_mode` is running. Only launched
alongside `dji_serial_bridge_node` (`real_hardware:=true`).

`point_to_cv_target` converts the vision pipeline's `/cv/panel_detection`
(REP-103 camera frame) into the CVTarget published on `/cv/target`
(position + confidence only) and the `/cv/panel_polygon` PolygonStamped,
publishing a zero-confidence CVTarget if the target goes stale.
Independent of `real_hardware` (its own
`enable_cv_target_bridge` toggle only) since `/cv/target` feeds both
`mcb_relay`'s `cv_target` input (real hardware) and `sim`'s `cv_head_aim`
node (sim), unlike `mcb_relay` itself, which stays gated on
`real_hardware:=true`.
