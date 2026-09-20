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
| `mcb_relay` | `/localization/odom`, `/odom`, `/cv/target` | `dji_serial_bridge_node`'s `~/relocalize`, `~/cv_target` |
| `target_selector` | `/cv/panel_detections`, `/dji_serial_bridge/ref_sys` (team colour) | `/cv/panel_detection` (one pick), `/cv/robot_panels` (that robot's panels) |
| `target_tracker` | `/cv/robot_panels` | `/cv/target_state` (`TargetState`, odom frame, armor model) |
| `point_to_cv_target` | `/cv/target_state`, `/cv/panel_detection`, `/pose` | `/cv/target` (`CVTarget`, root frame, aim + fire decision), `/cv/panel_polygon` |

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
/cv/panel_detections --[target_selector]--> /cv/robot_panels --[target_tracker]--> /cv/target_state
                                        \-> /cv/panel_detection (one pick)
/cv/target_state (armor model) + /cv/panel_detection (confidence, liveness, corners) --[point_to_cv_target]--\
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

`dds_transport` picks the DDS transport per node. On `default`, most nodes use
the container profile (shared memory plus UDP) and six small high-level
publishers -- `dji_serial_bridge`, `pose_translator`, `odom_tf_broadcaster`,
`robot_state_publisher`, `target_tracker`, `point_to_cv_target` -- are pinned
to `config/fastdds_udp_only.xml`. The reason is visibility, not throughput: a
node on shared memory is nearly invisible to `ros2 topic`/`node list` run in a
shell on the same machine, though other machines see it fine. Pinning those six
keeps pose, odom, TF and the CV target greppable from a robot terminal.
`dds_transport:=udp_only` puts every node this file launches on UDP; it does
not reach `sentry_localization`'s nodes or the camera launch.

```bash
ros2 launch thornbots_pkg auto.launch.py dds_transport:=udp_only
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

Grouping is single-linkage at `panel_group_radius_m` (0.5). Adjacent panels
on one robot are `hypot(0.30, 0.24) = 0.384m` apart (opposite pairs
0.48-0.60m, never visible together). It was 0.4, and with 3cm detection noise
that split 43% of two-panel frames into two robots in sim (2026-09-17), which
starved the tracker of the second panel. Two robots whose nearest panels are
within 0.5m will merge; nobody has fixed that. Centroid linkage would instead split one spinning robot as its centroid
wanders. Clustering runs in camera frame, which is metric because every panel
in a `PanelDetectionArray` shares one camera pose.

Every panel of the winning robot also goes out on `/cv/robot_panels`,
winner first, for `target_tracker`. The per-frame pick flips between two
visible panels, and one panel per frame drops the armor model's spin lock.

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

The target is a 4-panel armor model, the standard RoboMaster anti-spin
tracker (rm_auto_aim's) reduced to position-only detections. On hardware
`class_id` is the robot's team and plate, the same on all four panels, so spin
can't come from handoffs; it has to come from geometry. The old `SpinDetector`
counted `class_id` changes and only worked because the emulator faked them.

`ArmorEKF` state is centre, centre velocity, the tracked panel's normal yaw,
spin rate `w` and that panel's radius, with the other pair's radius kept
aside. A panel measures `centre + r * (cos yaw, sin yaw, 0)`. Each detection
first associates to the nearest of the four predicted panels, skipping those
facing more than ~107 degrees from the camera; `k != 0` is a handoff, stepping
yaw by quarter turns and swapping radii on odd `k`. A cut at exactly 90 degrees
mis-assigned edge-on panels whenever the camera moved 10cm and held a wrong
spin for seconds. `PanelDetection.corners` stay unused:
`roi_depth_node.cpp`'s `deprojectDetection()` puts all four at one
`mean_depth_m`, so real corners carry no panel tilt.

Noise: `R` stddev is `meas_noise_base_m + meas_noise_range_coeff * range_m^2`
(depth error grows with z^2). `process_noise_accel` (2.0 m/s^2) drives the
centre, `process_noise_yaw_accel` (5.0 rad/s^2) the spin rate,
`process_noise_radius` (0.02) the radius, which clamps to 0.18-0.45m. These
came from an offline sweep against an emulator-shaped target; the higher yaw
noise let a jinking target's translation leak into `w`. Without spin, yaw and
radius are unobservable and drift, but the seen panel's position stays solid,
which is what `point_to_cv_target` aims at then.

Innovation gating: a normalised innovation over `gate_nis` (16.3, 99.9% for 3
dof) is skipped, and `max_outliers` (3) in a row re-seed centre, velocity and
yaw from the panel while keeping `w` and the radii. Sim's `target_driver`
reverses instantly at each end of its path, which otherwise wrecked `w`.

`ArmorTracker` runs five `ArmorEKF`s seeded at `w` = 0, +-7, +-13 rad/s
(1-2Hz both ways) on every panel and publishes the one with the lowest EWMA of
normalised innovation. A single filter fed 15cm noise for its first second,
as when sim's head slews in from rest, locked onto a wrong spin for good on 6
of 10 seeds; the bank recovered on 9. A hypothesis trailing the leader by 3
for 1s is re-seeded from the leader's centre and yaw with its own spin prior.

The filter resets on a `robot_track_id` change or a `track_max_gap_s` gap.
`valid` goes true after 2 updates, because an engagement can be shorter than
one spin period; consumers should weigh `variance` and `yaw_rate_variance`.
Published state is `ArmorTracker.predicted(t_sec)` at the detection stamp.

### mcb_relay.py

The bridge stays a pure UART/DJI translator; this node reshapes upstream output
for it. `relocalize` compares `/localization/odom` (published in every
`localization_mode` and `use_ekf` combination) with the MCB's raw `/odom`,
using no TF and no backend assumptions. When they differ by more than
`error_threshold_meters` (0.05) and raw speed is under `max_move_speed`
(0.05 m/s, so the correction is still current when the MCB applies it), it
publishes the localized `(x, y)` as a `Point` on `~/relocalize`. The bridge
packs that into a `RelocalizePayload` and the MCB resets its odometry origin.
`cv_target` is a straight republish, and carries the fire decision with the
aim point it was solved for.

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
- `valid == True`: `plan_shot()`'s aim point in root. See below.

`plan_shot()` picks a mode per tick, with hysteresis on `|yaw_rate|`: spin
mode above `spin_enter_rad_s` (3.0), panel mode below `spin_exit_rad_s` (2.0).

- Panel mode leads the tracked panel (centre velocity plus its tangential
  `r*w`) with `solve_intercept()` and fires now. A slow target's yaw and radius
  drift, so the seen panel beats the best-facing predicted one.
- Spin mode leads a point on the centre-to-shooter line, radius the mean of
  both pairs, half a tick ahead. That line is steady, so the gimbal can hold it
  while panels sweep past. It fires with the aim point's `delay_ms` set so a
  panel normal points along that line at impact, if that alignment falls within one
  publish tick (33ms); otherwise it waits for a later tick. Standard
  "centre aim plus timed fire"; a gimbal chasing each panel at 1-2Hz spin
  would lag it.

`solve_intercept()` is the time-of-flight fixed point, 2-3 iterations, with no
gravity, drag or elevation (Type-C handles those). `lead_enabled:=false`
aims at the current estimate and fires untimed, the control for the shot-hit
bench.

`fire` and `delay_ms` ride on `CVTarget`, so the fire decision reaches the
MCB in the same frame as the aim it was solved for, measured from that
frame's `header.stamp` (see `ros2_dji_serial_bridge/UART_PROTOCOL.md`). The
firmware struct still has to grow to match before hardware timing works.

The solve's tau is this tick's `now - state.header.stamp` plus
`firmware_latency_s` (0.05, the static fire-to-exit delay the shot-hit bench
models; unmeasured on hardware). It skips `LatencyStat.mean` because
cached state ages between arrival and tick, by up to a tracker period plus
tick phase (measured: 20ms mean at arrival, 50ms at tick), and the offset
jitters. `LatencyStat` is logged as a diagnostic.

Frames convert by TF: `lookup_transform(root_frame, odom_frame, Time())`. For
lead, the reverse lookup gives shooter position in odom, and
`RobotPose.vel_x/vel_y` rotated by it gives shooter velocity. Both use the
latest transform, since the solve needs where the shooter is now.

Each publish tick with an aim point may fire, at most `fire_rate_hz` (2.0) and
only above `fire_confidence_threshold`, so a failed TF lookup or stale state
holds fire. HP, heat and power gating are not built.
