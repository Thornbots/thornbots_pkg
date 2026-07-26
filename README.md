# sentry_pkg

Hardware interface and robot description for the Thornbots ARC 2026
Sentry robot. Gets `/pose` (real hardware or `sim`) and `/scan` onto the
ROS graph, owns the robot description (`urdf/` + `robot_state_publisher`),
and republishes whatever `odom->root` pose `sentry_localization` computes.
See `sentry_localization/README.md` for the actual localization backends
(SLAM/AMCL/EKF), and the repo-level `SESSION_NOTES.md` /
`ARCC_2026_SENTRY_CONTEXT.md` for the broader project context.

## What it owns

- `pose_translator` — sole consumer of `/pose`. Publishes `/odom` (raw,
  uncorrected wheel odometry) and `/joint_states`. Same code path whether
  `/pose` comes from real hardware or `sim/pose_emulator.py`. No longer
  broadcasts any TF itself.
- `odom_tf_broadcaster` — subscribes `/localization/odom` (published by
  `sentry_localization`, regardless of which backend is active) and
  broadcasts the `odom->root` TF from it. This is the "republish" step —
  `sentry_pkg` never needs to know which `localization_mode` is running.
- `mcb_relay` — the *only* node allowed to publish/subscribe directly on
  `dji_serial_bridge_node`'s topics. `dji_serial_bridge` stays a pure
  UART/DJI-protocol translator; anything that wants to send something to
  the Type-C/MCB board goes through this node instead, which reads each
  upstream package's own output and reshapes it into whatever
  `dji_serial_bridge_node` expects:
  - **relocalize**: compares `/localization/odom` (`sentry_localization`'s
    one guaranteed output — the same topic regardless of `localization_mode`,
    no TF lookups, no backend-specific assumptions) against `/odom`
    (`pose_translator`'s raw MCB odometry). Once they've drifted apart past
    a threshold and the chassis is nearly stationary, sends the corrected
    `(x, y)` to `dji_serial_bridge_node`'s `~/relocalize`.
  - **cv_target**: republishes `/cv/target` (`CVTarget`, from
    `point_to_cv_target` below) unchanged onto `dji_serial_bridge_node`'s
    `~/cv_target`.
  Only launched alongside `dji_serial_bridge_node` (`real_hardware:=true`).
- `point_to_cv_target` — converts the vision pipeline's `/roi_point`
  (`geometry_msgs/PointStamped`, REP-103 camera frame) into the `CVTarget`
  `mcb_relay` expects on `/cv/target`, estimating velocity/acceleration by
  finite-differencing consecutive points and publishing a zero-confidence
  `CVTarget` if the target goes stale. Same `real_hardware` gate, plus its
  own `enable_cv_target_bridge` toggle.
- Its own `robot_state_publisher`, off `urdf/sentry.urdf.xacro`.
- Includes `sentry_localization`'s launch file to bring up whichever
  localization backend `localization_mode` selects.

## Node/topic pipeline

```
/pose --[pose_translator]--> /odom --> sentry_localization --> /localization/odom --[odom_tf_broadcaster]--> odom->root TF
                          \-> /joint_states --[robot_state_publisher]--> rest of TF tree
/scan ------------------------------> sentry_localization (map->odom TF owned directly by slam_toolbox/amcl there)

/localization/odom vs /odom            --[mcb_relay, drift-gated]-------> dji_serial_bridge_node (~/relocalize) --> UART --> MCB
/roi_point --[point_to_cv_target]--> /cv/target --[mcb_relay]-----------> dji_serial_bridge_node (~/cv_target)  --> UART --> MCB
```

## Prerequisites

Run everything below **inside the Isaac ROS dev container** (see the
`isaac-ros-docker` skill / `DOCKER.md` for how to launch/attach it), with
the workspace built and sourced:

```bash
isaac_ros_common/scripts/dexec.sh -- bash -c \
  "cd /workspaces/isaac_ros-dev && colcon build --packages-select sentry_pkg sentry_localization && source install/setup.bash"
```

`dexec.sh` (see `DOCKER.md`) already handles env sourcing correctly —
prefer it over hand-rolled `docker exec`.

## Launching

Everything goes through one launch file, `auto.launch.py` — it includes
`sentry_localization`'s `localization.launch.py` itself, so you don't
need to launch that package separately:

```bash
# Against real hardware (default): also launches dji_serial_bridge_node
# (/pose from the Type-C board's serial link) and sllidar_ros2 (/scan from
# the RPLIDAR A2M8), and runs on wall-clock time.
ros2 launch sentry_pkg auto.launch.py

# Against sim instead (run `ros2 launch sim sim.launch.py` first — it
# provides /pose via pose_emulator.py and /scan itself):
ros2 launch sentry_pkg auto.launch.py real_hardware:=false
```

`real_hardware` also drives `use_sim_time` — there's no separate arg for
it (false/wall-clock when `real_hardware:=true`, true when it's `false`,
since that's exactly when sim's `/clock` exists to use).

### `localization_mode` — pick the whole localization scheme

Forwarded straight through to `sentry_localization`; see
`sentry_localization/README.md` for the full table and rationale of each
value (`slam` default / `mapping` / `amcl` / `ekf`).

```bash
ros2 launch sentry_pkg auto.launch.py real_hardware:=false localization_mode:=amcl
ros2 launch sentry_pkg auto.launch.py real_hardware:=false localization_mode:=mapping load_map:=false
```

### Other useful args

- `map_file`, `load_map`, `odom_frame`, `home_yaw_tolerance` — forwarded
  to `sentry_localization`; see its README for what each controls.
- `lidar_serial_port` / `lidar_baudrate` (defaults `/dev/ttyUSB0` /
  `115200`) — RPLIDAR A2M8 serial settings, only used when
  `real_hardware:=true`. Owned by `sentry_pkg` since it owns the hardware
  drivers.

Full argument docs live in the module docstrings at the top of
`launch/auto.launch.py` (this package) and
`sentry_localization/launch/localization.launch.py` (the included
localization stack).

## Visualizing

```bash
isaac_ros_common/scripts/dexec.sh -- rviz2 -d install/sentry_pkg/share/sentry_pkg/rviz/config.rviz
```

## Nodes (`sentry_pkg/`)

- `pose_translator.py` — `/pose` → `/odom` + `/joint_states`.
- `odom_tf_broadcaster.py` — `/localization/odom` → `odom->root` TF.
- `mcb_relay.py` — sole relay onto `dji_serial_bridge_node`'s topics; see
  "What it owns" above.
- `point_to_cv_target.py` — `/roi_point` → `/cv/target` (`CVTarget`); see
  "What it owns" above.

## Testing

The localization drift/jerk-correction integration suite now lives in
`sim/test/localization/run_localization_drift_tests.py`
(it launches this package's `auto.launch.py`, which includes
`sentry_localization`) — see `sim/README.md`'s Testing
section.

Standard `colcon test`-style checks (`ament_copyright`/`ament_flake8`/
`ament_pep257`) apply via the normal `colcon test --packages-select
sentry_pkg`.

## Cleaning up

Always fully stop a launch tree (`isaac_ros_common/scripts/kill_launch.sh
<pid>`, not a bare `pkill`) before relaunching — a half-killed tree leaves
duplicate nodes publishing TF, which causes jitter in the next run.

## Notes

Design rationale and history trimmed out of in-code comments/docstrings to
keep those short — kept here for anyone who needs the full context.

### pose_translator.py

The odom covariance placeholder (1cm stddev on position/velocity, all
other fields zero) is a first-pass value, not measured/validated (same
disclaimer as `config/ekf.yaml`'s `process_noise_covariance`) — but
non-zero is the important part. Left at all-zero, `robot_localization`'s
EKF has no signal that this source's absolute x/y is any more or less
trustworthy than `/scan_odom`'s, so it can't weight rf2o's scan-matched
estimate more heavily even when it should be — see `SESSION_NOTES.md`'s
2026-07-24 EKF investigation. 1cm stddev is a reasonable per-sample
encoder-noise order of magnitude to start from. Unset fields (z/roll/pitch,
and yaw — this chassis is holonomic and never reports real orientation)
are left at 0, which is fine since `odom0_config` in `ekf.yaml` excludes
them from fusion.

### mcb_relay.py

Sole relay between `sentry_pkg`/`sentry_localization`/CV and
`dji_serial_bridge_node`'s topics. Per project convention, only
`sentry_pkg` is allowed to publish/subscribe directly on
`dji_serial_bridge`'s topics — `dji_serial_bridge` itself stays a pure
UART/DJI-protocol translator, with no other package wired to it. This node
reads each upstream package's own output and reshapes it into whatever
`dji_serial_bridge_node`'s subscription expects.

`relocalize`: compares `localization_odom_topic` (`/localization/odom` —
`sentry_localization`'s one guaranteed output, published by every
`localization_mode`: slam/mapping/amcl/ekf) against `raw_odom_topic`
(`/odom` — `sentry_pkg`'s own `pose_translator`, i.e. the MCB's raw
uncorrected wheel odometry). Deliberately backend-agnostic: no TF lookups,
no assumption about which `localization_mode` is running, just two
Odometry topics. Once they've drifted apart by more than
`error_threshold_meters` *and* the chassis is nearly stationary (raw odom
speed below `max_move_speed`, so the correction isn't stale by the time
the MCB applies it), publishes the localized `(x, y)` as a `Point` on
`relocalize_output_topic` — `dji_serial_bridge_node`'s `~/relocalize`
(default resolved name `/dji_serial_bridge/relocalize`) packs this into a
`RelocalizePayload` and sends it over UART so the MCB can reset its own
odometry origin.

`cv_target`: republishes `cv_target_input_topic` (`dji_serial_bridge/msg/
CVTarget`, from the CV pipeline) unchanged onto `cv_target_output_topic` —
`dji_serial_bridge_node`'s `~/cv_target` (default resolved name
`/dji_serial_bridge/cv_target`).

### lidar_self_filter.py

The lidar is mounted rigidly on the head (see
`sim/urdf/sentry.urdf.xacro`'s `lidarlink` joint), so head and lidar
always rotate together as one unit — whatever part of the head's own
structure blocks the lidar's view sits at a FIXED angle in the lidar's own
frame regardless of headlink's current yaw, even though that blocked
WORLD-frame bearing sweeps around as headlink rotates (see
`sim/head_sweep.py`'s docstring: sweeping headlink moves this blind wedge
around the world so SLAM eventually gets full coverage). That fixed
relationship is what makes a static angular filter here viable at all — no
joint-state subscription needed.

Runs for both sim and real hardware (see `sentry_pkg/launch/auto.launch.py`):
sim's `gpu_lidar` is a rendering sensor with no physics collision to fall
back on, so trying to model this via mesh visibility in the URDF instead
(gz-sim's `visibility_mask`/`visibility_flags`) was unreliable — it could
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
right at its tangent edge against the lidar's exact scan plane — beams
aimed more centrally through the head's bulk can pass clean through a
thin/non-watertight STL without registering any hit at all, "seeing"
whatever real geometry (walls, etc.) sits beyond the head, which the real
hardware's fully-solid head would never let through. So this sector isn't
just "wherever `/scan_raw` shows a close self-hit"; it's sized to the
head's actual real-world angular footprint from the lidar's vantage point.

Current values (1.0 rad wide): the raw self-hit cluster measured roughly
2.967–3.022 rad, and `blind_angle_end=3.20` lines up with where sim's mesh
already lets real wall hits back through (from ~3.024, right after the
cluster) — that edge is left alone. `blind_angle_start` is widened well
before the cluster's own start (2.967), down to 2.20, to approximate the
real head's full width, since sim's mesh only registers a self-hit right
at its tangent edge and lets real wall hits through everywhere else in the
head's true angular footprint. These are still sim-mesh-derived estimates
and will need retuning against a real `/scan_raw` capture before trusting
them on real hardware.

### point_to_cv_target.py

Bridges the vision pipeline's 3-D target estimate to a CVTarget message
for `mcb_relay` — this is the missing link between `roi_depth_query`/
`roi_depth_node` and `mcb_relay`'s `cv_target` input, since only
`sentry_pkg` is allowed to publish the messages that eventually reach
`dji_serial_bridge`.

Subscribes `point_topic` (`geometry_msgs/PointStamped`) — REP-103 camera
body frame (X forward, Y left, Z up), default `"roi_point"`, published by
`roi_depth_query`/`roi_depth_node` — and `confidence_topic`
(`vision_msgs/Detection2D`, optional) — read only for a confidence score
(max of all hypothesis scores, same rule `detection_picker_node` uses),
default `"roi"`.

Publishes `output_topic` (`dji_serial_bridge/msg/CVTarget`) — camera-frame
convention (X right, Y up, Z forward), see `CVTarget.msg`. Default
`"/cv/target"`, matching `mcb_relay`'s `cv_target_input_topic` default.

Frame conversion (REP-103 -> CVTarget convention): `cv.x = -p.y` (right =
-left), `cv.y = p.z` (up = up), `cv.z = p.x` (forward = forward).

Velocity/acceleration: `roi_depth_node` only publishes position. When
`estimate_velocity` is true (default), `v_x/v_y/v_z` and `a_x/a_y/a_z` are
estimated by finite-differencing consecutive `PointStamped` samples (using
the message timestamps, not wall-clock arrival time) and smoothed with a
simple exponential moving average (`velocity_filter_alpha`). This is a
coarse estimate, not a proper tracker/filter — if you already have a
tracked velocity upstream, publish it separately and set
`estimate_velocity:=false` to leave those fields at zero.

Stale-target watchdog: if no new point arrives for `target_timeout_s`, a
single zero-confidence CVTarget is published (so the MCB/gimbal can stop
tracking a ghost target) and the velocity filter resets; publishing
resumes cleanly on the next fresh point.

### auto.launch.py

`real_hardware:=true` is the default because running against real hardware
is the common case. When running against sim instead (`ros2 launch sim
sim.launch.py`, which runs `sim/pose_emulator.py` to publish `/pose` in
the same `dji_serial_bridge/msg/RobotPose` format real hardware sends,
plus raw `/scan` via its own gz bridge), launch this with
`real_hardware:=false` so it doesn't also try to open the real serial
devices, and so it uses sim's `/clock`.

`pose_translator` (fed by `/pose`) turns raw hardware pose into `/odom`
(raw, uncorrected wheel odometry) and `/joint_states` — same code path for
sim and real hardware, so there's only one place pose handling can go
wrong. This package also runs its own `robot_state_publisher` off
`sentry_pkg/urdf/sentry.urdf.xacro` (fed by `pose_translator`'s
`/joint_states`) rather than depending on sim's URDF/TF — `sentry_pkg`
owns the whole TF tree itself, sim only ever provides raw sensor data
through the shared real-hardware-shaped interfaces.

`sentry_pkg` no longer computes `odom->root` itself: `/odom` + `/scan` are
handed to `sentry_localization` (included in the launch file), which
always publishes the localized result on `/localization/odom` regardless
of which backend `localization_mode` selects. `odom_tf_broadcaster`
subscribes that topic and broadcasts the actual `odom->root` TF — so this
package never needs to know which `localization_mode` is active.

`mcb_relay` reshapes each upstream package's own output into what
`dji_serial_bridge_node` expects: `/localization/odom` vs `/odom`
(drift-gated) -> `~/relocalize`, and the CV pipeline's CVTarget ->
`~/cv_target`. Backend-agnostic by construction — no TF lookups, no
assumption about which `localization_mode` is running. Only launched
alongside `dji_serial_bridge_node` (`real_hardware:=true`).

`point_to_cv_target` converts the vision pipeline's `/roi_point` (REP-103
camera frame) into the CVTarget `mcb_relay`'s `cv_target` input expects on
`/cv/target` — estimating velocity/acceleration by finite-differencing
consecutive points, and publishing a zero-confidence CVTarget if the
target goes stale. Same `real_hardware` gate, plus its own
`enable_cv_target_bridge` toggle.
