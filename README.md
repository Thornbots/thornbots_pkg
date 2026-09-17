# thornbots_pkg

Hardware interface, robot description, and CV target selection for the Thornbots
ARC 2026 Sentry. It puts `/pose` (hardware or `sim`) and `/scan` on the graph,
runs `robot_state_publisher` off `urdf/sentry.urdf.xacro`, republishes the
`odom->root` pose from `sentry_localization`, and turns detections into a
root-frame `CVTarget`. Localization backends are in
`sentry_localization/README.md`; game rules are in `../ARCC_2026_SENTRY_CONTEXT.md`.

## Nodes

| Node | In | Out |
| --- | --- | --- |
| `pose_translator` | `/pose` | `/odom` (raw wheel odom), `/joint_states`. No TF. |
| `odom_tf_broadcaster` | `/localization/odom` | `odom->root` TF |
| `lidar_self_filter` | `/scan_raw` | `/scan`, head blind sector blanked |
| `mcb_relay` | `/localization/odom`, `/odom`, `/cv/target`, `/sentry/fire_command` | `dji_serial_bridge_node`'s `~/relocalize`, `~/cv_target`, `~/fire_command` |
| `target_selector` | `/cv/panel_detections`, `/dji_serial_bridge/ref_sys` (team colour) | `/cv/panel_detection` (one pick) |
| `target_tracker` | `/cv/panel_detection` | `/cv/target_state` (`TargetState`, odom frame) |
| `point_to_cv_target` | `/cv/target_state`, `/cv/panel_detection`, `/pose` | `/cv/target` (`CVTarget`, root frame), `/cv/panel_polygon`, `/sentry/fire_command` |

`mcb_relay` is the only node allowed on the bridge's topics, and only launches
with `real_hardware:=true`. `point_to_cv_target` runs in both modes because
`/cv/target` also feeds sim's `cv_head_aim`. The CV nodes each have an enable
arg (`enable_target_selector`, `enable_target_tracker`,
`enable_cv_target_bridge`) and a rclpy-free `*_core.py` half for unit tests.

```
/pose --[pose_translator]--> /odom --> sentry_localization --> /localization/odom --[odom_tf_broadcaster]--> odom->root TF
                          \-> /joint_states --[robot_state_publisher]--> rest of TF tree
/scan_raw (sllidar_node or sim) --[lidar_self_filter]--> /scan --> sentry_localization (map->odom TF owned by slam_toolbox/amcl there)

/localization/odom vs /odom            --[mcb_relay, drift-gated]-------> dji_serial_bridge_node (~/relocalize) --> UART --> MCB
/cv/panel_detections --[target_selector]--> /cv/panel_detection --[target_tracker]--> /cv/target_state
/cv/target_state (position) + /cv/panel_detection (confidence, liveness, corners) --[point_to_cv_target]--\
                                                                    /cv/panel_polygon (rviz/foxglove) <---/
                                                                    /cv/target (root frame) <-------------/
/cv/target --[mcb_relay]--> dji_serial_bridge_node (~/cv_target) --> UART --> MCB
           \-[sim's cv_head_aim]--> /head_pan_cmd, /head_pitch_cmd (sim only, see sim/README.md)
```

## Build and launch

In a container terminal, build:

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select thornbots_pkg sentry_localization
```

In every new terminal, source the overlay:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

The image bakes its own copy of this package into `/workspaces/ros2_ws`, and a
fresh shell sources only that one. Skip the line above and you run the image's
old code instead of your edit. `ros2 pkg prefix thornbots_pkg` should print a
`/workspaces/isaac_ros-dev/` path.

`auto.launch.py` is the only entry point and includes `sentry_localization`'s
launch itself.

```bash
# Real hardware (default): also starts dji_serial_bridge_node and sllidar_ros2, wall-clock time.
ros2 launch thornbots_pkg auto.launch.py

# Sim: start `ros2 launch sim sim.launch.py` first; it provides /pose and /scan.
ros2 launch thornbots_pkg auto.launch.py real_hardware:=false
```

`real_hardware` also sets `use_sim_time` (true when `real_hardware:=false`).
Against sim, it keeps the launch off the real serial devices.

`localization_mode` (`amcl` default, `slam`, `mapping`, `none`) picks the
`map->odom` owner. `use_ekf` (default `false`) picks whether `odom->root` is
EKF-fused, with any mode. Both, plus `map_file`, `load_map` and `odom_frame`,
pass through to `sentry_localization`.

```bash
ros2 launch thornbots_pkg auto.launch.py real_hardware:=false localization_mode:=mapping load_map:=false
ros2 launch thornbots_pkg auto.launch.py real_hardware:=false localization_mode:=none use_ekf:=true
```

`lidar_serial_port` and `lidar_baudrate` (`/dev/ttyUSB0`, `115200`) configure
the RPLIDAR A2M8 on hardware. The docstring at the top of
`launch/auto.launch.py` documents every arg.

This package has no rviz config; `sim` does, and `sim.launch.py` opens rviz2
itself. For a hardware run, build `sim` too and then:

```bash
rviz2 -d $(ros2 pkg prefix sim)/share/sim/rviz/config.rviz
```

Stop a launch with Ctrl+C and wait for every node to exit before relaunching;
don't `pkill` individual nodes. Leftover nodes keep publishing TF and make the
next run jitter. `pgrep -af -- --ros-args` lists every running node and should
come back empty (sim's nodes show up too while `sim.launch.py` runs).

## Testing

The unit tests exercise the `*_core.py` halves on synthetic input and need no
running graph:

```bash
cd /workspaces/isaac_ros-dev/src/thornbots_pkg
python3 -m pytest test/test_target_selector.py test/test_target_tracker.py test/test_point_to_cv_target.py
```

`test_target_selector.py` covers scoring, centrality, grouping and hysteresis;
`test_target_tracker.py` the spin detector, KF and radial correction;
`test_point_to_cv_target.py` the intercept solve and latency stat. `pytest test/`
also picks up the ament copyright, flake8 and pep257 checks, which
`colcon test --packages-select thornbots_pkg` runs too.

The localization drift suite is
`sim/test/localization/run_localization_drift_tests.py`, which launches
`auto.launch.py`; see `sim/README.md`.

## Notes

Design rationale, kept here so in-code comments stay short.

### pose_translator.py

Nobody has measured odom covariance. The placeholder is 1cm stddev on
position and velocity, everything else zero. It has to be non-zero: at zero,
`robot_localization`'s EKF can't weight `/scan_odom` (rf2o) against this
source. Unset fields (z, roll, pitch, and yaw, since the holonomic chassis
never reports orientation) stay 0; `odom0_config` in `ekf.yaml` excludes them.

### target_selector.py

Team colour comes from `RefSysStatus.is_on_blue_team`: on blue it drops class
IDs 0-3, on red 4-7. Until the first `RefSysStatus` arrives (always, in sim
without a referee) it passes every detection through, so it can pick an allied
robot.

Scoring is ported from the old C++ `detection_picker_node`: confidence +
`center_weight`*centrality + `priority_class_bonus` for `priority_class_ids`,
with `min_score` gating raw confidence only. Two things changed.

Centrality is now 3D. Pixel distance from image centre means nothing after
depth, so `centrality_3d()` uses bearing off the camera's +X axis: 1.0 at
boresight, 0.0 at `centrality_max_angle_rad` (45 degrees, about half the ~87
degree HFOV). Points behind the camera (`x<=0`) score 0.

Grouping is single-linkage at `panel_group_radius_m` (0.4). Adjacent panels
on one robot are `hypot(0.30, 0.24) = 0.384m` apart (opposite pairs
0.48-0.60m), so 0.4m links neighbours and transitivity reaches all four.
Two robots whose nearest panels are within 0.4m will merge; nobody has fixed
that. Centroid linkage would instead split one spinning robot as its centroid
wanders. Clustering runs in camera frame, which is metric because every panel
in a `PanelDetectionArray` shares one camera pose.

Hysteresis is per robot, since a spinning robot's panels vanish every
0.5-1s (145 degree exposure cone, 1-2Hz spin) and panel stickiness would
delay correct handoffs. `RobotHysteresis` keeps the incumbent's last centroid;
the nearest cluster continues it, and a challenger must win by `switch_margin`
(0.3) for `switch_hold_frames` (5) frames. Acquisition is immediate when there
is no incumbent or the nearest match is beyond `gate_radius_m`.

The selector doesn't use `target_tracker`'s predicted centre. That would help
single-panel handoffs, but `/cv/target_state` is in `odom` and clustering is
in camera frame to avoid a per-frame TF lookup. Wiring it in means
transforming the prediction every frame or clustering in `odom`, both real
design changes. The last-centroid hold is a zero-order stand-in that is weaker
across long handoff gaps.

### target_tracker.py

The filter runs in `odom`. `root` moves with the sentry, which breaks
constant velocity under acceleration, and camera also rotates with the gimbal.
`lookupTransform(odom, camera, detection_stamp + pose_latency_s)` corrects
both. `pose_latency_s` (0.01, unmeasured, inside the documented 3-25ms range)
offsets `dji_serial_bridge_node`'s `handle_pose()` stamping `RobotPose` at
parse time instead of MCB sample time. Sweep it on hardware.

A missing transform logs an error and drops the detection, with no stale or
zero fallback. In sim `robot_state_publisher` is always up, so a silent
fallback would hide a broken TF tree until competition.

A transform that is only behind is recoverable. When the detection stamp is
newer than the newest TF, `_lookup_camera_tf()` retries at `Time()` and
accepts it within `tf_future_tolerance_s` (0.25), warning each time. Past that
it drops the detection and logs the gap. Bearing error is gap times head slew
rate, so a loose tolerance would trade no output for confident bad aim.

Every TF lookup in `target_tracker` and `point_to_cv_target` is
non-blocking. `/tf` is serviced by the same executor as the detection
callback, and `Buffer.lookup_transform(timeout=...)` sleeps in a wall-clock
loop, so a 50ms wait per ~60Hz detection starved `/tf`. The buffer then fell
0.6-1.7s behind detections that TF itself was ~60ms ahead of, and the tracker
dropped nearly everything (2026-09-17, measured against a separate listener
on the same run). Humble's `TransformListener(spin_thread=True)` doesn't help:
it adds the whole node to a second executor rather than isolating `/tf`.

`SpinDetector` calls a target spinning after `spin_min_handoffs` (3) `class_id`
changes at roughly equal intervals (coefficient of variation under
`spin_cv_max`, 0.35), and drops back after `spin_handoff_timeout_s` (1.5s)
without one. `spin_hz` assumes one handoff per quarter turn and can't tell
direction; that is coarse, but only the spin/no-spin branch depends on it.
`spin_phase` is re-derived from time since handoff and nothing reads it.

The spinning branch takes a running mean of panel positions, corrected for the
exposure-cone bias toward the camera. It ignores `PanelDetection.corners`.
`roi_depth_node.cpp`'s `deprojectDetection()` deprojects all four corners at
one `mean_depth_m`, so every real quad is fronto-parallel and a corner cross
product always returns the boresight axis. `cv_target_emulator.py`'s corners
do carry tilt, so a corner-based normal would pass in sim and be wrong on
hardware, a divergence a sim hit-rate can't catch.

`corrected_centre()` pushes the panel position out along its own camera ray by
`panel_radius_m` (0.27, mean of 0.30 and 0.24; `class_id` doesn't say which
face is visible). Plane fitting only works under ~2m, and the hardware
pipeline produces no panel orientation, so this stays. `estimator` is always
`0` (`running_mean`). The width-refined `1` branch is unimplemented and must
beat the running mean against the emulator's known panel normal before it
replaces it.

The KF is 6-state constant velocity with range-scaled `R`:
`meas_noise_base_m + meas_noise_range_coeff * range_m^2` (depth error grows
with z^2). `spin_meas_inflation` is 1.0, because a ~30-sample mean is less
noisy than one sample. The orbit-averaging error it might address is a bias,
which inflating `R` doesn't fix.

The window mean describes the window's mean time, about `spin_window_s / 2`
before the newest sample, so the KF gets it at that time (`meas_t`). Stamping
it at the newest sample biased velocity low by 0.25s times chassis speed,
which the lead solve then extrapolated. Published `centre`, `velocity` and
`variance` come from `KalmanFilter6D.predicted(t_sec)`, a non-mutating
extrapolation to the detection stamp, so `header.stamp` matches the payload.

The filter resets only on a `robot_track_id` change or a `track_max_gap_s`
gap, never on a `class_id` handoff. `valid` goes true after 2 updates, because
an engagement can be shorter than one spin period. Consumers should weigh
`variance`, which stays large after a reset.

### mcb_relay.py

The bridge stays a pure UART/DJI translator; this node reshapes upstream output
for it. `relocalize` compares `/localization/odom` (published in every
`localization_mode` and `use_ekf` combination) with the MCB's raw `/odom`,
using no TF and no backend assumptions. When they differ by more than
`error_threshold_meters` (0.05) and raw speed is under `max_move_speed`
(0.05 m/s, so the correction is still current when the MCB applies it), it
publishes the localized `(x, y)` as a `Point` on `~/relocalize`. The bridge
packs that into a `RelocalizePayload` and the MCB resets its odometry origin.
`cv_target` and `fire_command` are straight republishes.

### lidar_self_filter.py

The lidar is bolted to the head, so the head's blind sector is fixed in the
lidar frame whatever the yaw, and a static angular filter needs no joint
states. It runs in sim and on hardware. Sim's `gpu_lidar` has no collision,
and gz-sim's `visibility_mask`/`visibility_flags` work per visual, so the URDF
approach either saw through the head or reported self-hits. Hardware has no
equivalent.

The sector covers the head's real footprint, wider than sim's self-hit
cluster. Sim's thin, non-watertight STL only returns hits at its tangent edge
and lets beams through its bulk. The raw cluster sits at about 2.967-3.022
rad. `blind_angle_end` (3.20) matches where sim wall hits return (from
~3.024); `blind_angle_start` (2.20) is widened to approximate the real head,
for 1.0 rad total. Both come from the sim mesh; retune against a real
`/scan_raw` capture.

### point_to_cv_target.py

`/cv/target` `x/y/z` is a root-frame point Type-C aims at, where it used to be
a camera-relative vector. See `CVTarget.msg` and
`ros2_dji_serial_bridge/README.md`'s wire-format history.

`/cv/target_state` has position, velocity and validity but no confidence, so
`panel_topic` still drives confidence, the `target_timeout_s` (0.5) watchdog
and the `/cv/panel_polygon` corners. Without `target_tracker` running,
`/cv/target` stays at zero confidence even with live panels.

A timer publishes at `cv_target_publish_rate_hz` (30) from cached state. The
tracker runs at detection rate (up to ~60Hz), faster than Type-C's PID needs.
`_compute_aim_point()` handles three cases per tick:

- No usable state, or TF fails: zero confidence, with a throttled `ERROR` on
  TF failure. Usable means present, younger than `target_timeout_s`, and on the
  newest panel's `robot_track_id`. On a target switch the panel names the new
  robot at once while the tracker needs two updates, so without that check
  the node aims at the old robot at full confidence with `track_valid=True`.
- `valid == False`: raw `panel` position, `lead_applied=False`,
  `track_valid=False`. No extrapolation off an unconverged track.
- `valid == True`: KF `centre` in root, after `solve_intercept()` if
  `lead_enabled`. The solve is a 2-3 iteration time-of-flight fixed point with
  no gravity, drag or elevation (Type-C handles those).

`lead_enabled` defaults to true. On 2026-09-17's headless shot-hit sweep it cut
mean miss distance at 1, 2 and 4 m/s (0.23 to 0.14m, 0.55 to 0.46m, 1.23 to
1.01m) and scored more hits at 0.5 m/s in both runs (4/18 vs 0/11, 2/11 vs
1/18). Moving hit rates stay low, because the aim point is the chassis centre and
nothing times shots to the spin. See `sim/CV_TEST_GAPS.md`.

The solve's tau is this tick's `now - state.header.stamp` plus
`firmware_latency_s` (0.0, unmeasured). It skips `LatencyStat.mean` because
cached state ages between arrival and tick, by up to a tracker period plus
tick phase (measured: 20ms mean at arrival, 50ms at tick), and the offset
jitters. `LatencyStat` is logged as a diagnostic.

Frames convert by TF: `lookup_transform(root_frame, odom_frame, Time())`. For
lead, the reverse lookup gives shooter position in odom, and
`RobotPose.vel_x/vel_y` rotated by it gives shooter velocity. Both use the
latest transform, since the solve needs where the shooter is now.

`fire_rate_hz` (2.0) drives a placeholder fire trigger gated on
`target_active`, cached confidence, and the last publish tick having emitted an
aim point (`aim_ok`), so a failed TF lookup or stale state holds fire. Real
firing logic (HP, heat, power, timing) is not built.
