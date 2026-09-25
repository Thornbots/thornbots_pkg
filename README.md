# thornbots_pkg

Hardware interface, robot description, and CV target selection for the Thornbots
ARC 2026 Sentry. It puts `/pose` (hardware or `sim`) and `/scan` on the graph,
runs `robot_state_publisher` off `urdf/sentry.urdf.xacro` (the `sentry_v2`
CAD's frames, with its meshes in `meshes/sentry_v2/`), republishes the
`odom->root` pose from `sentry_localization`, and turns detections into a
root-frame `CVTarget`. Localization backends are in
`sentry_localization/README.md`; game rules are in
`../ARCC_2026_SENTRY_CONTEXT.md`.

## Nodes

| Node | In | Out |
| --- | --- | --- |
| `pose_translator` | `/pose` | `/odom` (raw wheel odom), `/joint_states`. No TF. |
| `odom_tf_broadcaster` | `/localization/odom` | `odom->root` TF |
| `lidar_self_filter` | `/scan_raw` | `/scan`, head blind sector blanked |
| `mcb_relay` | `/localization/odom`, `/odom`, `/cv/target` | `dji_serial_bridge_node`'s `~/relocalize`, `~/cv_target` |
| `target_selector` | `/cv/panel_detections`, `/dji_serial_bridge/ref_sys` (team colour) | `/cv/panel_detection` (one pick), `/cv/panel_polygon` (its corners), `/cv/robot_panels` (that robot's panels) |
| `target_tracker` | `/cv/robot_panels` | `/cv/target_state` (`TargetState`, odom frame, armor model) |
| `point_to_cv_target` | `/cv/target_state`, `/pose` | `/cv/target` (`CVTarget`, root frame, aim + fire decision) |

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
                                        \-> /cv/panel_detection (one pick), /cv/panel_polygon (rviz/foxglove)
/cv/target_state (armor model, confidence, track id) --[point_to_cv_target]--> /cv/target (root frame)
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

`real_hardware` also sets `use_sim_time` (true when `real_hardware:=false`),
and setting it false keeps the launch off the real serial devices.

`localization_mode` (`amcl` default, `slam`, `mapping`, `none`) picks the
`map->odom` owner. `use_ekf` (default `true`) picks whether `odom->root` is
EKF-fused, with any mode; on `true` it also starts `rf2o_laser_odometry_node`,
which scan-matches `/scan` into the `/scan_odom` the EKF fuses with `/odom`.
Both, plus `map_file`, `load_map` and `odom_frame`, pass through to
`sentry_localization`.

```bash
ros2 launch thornbots_pkg auto.launch.py real_hardware:=false localization_mode:=mapping load_map:=false
ros2 launch thornbots_pkg auto.launch.py real_hardware:=false localization_mode:=none use_ekf:=false
```

`dds_transport` picks the DDS transport per node. On `default`, most nodes use
the container profile (shared memory plus UDP), and six small high-level
publishers (`dji_serial_bridge`, `pose_translator`, `odom_tf_broadcaster`,
`robot_state_publisher`, `target_tracker`, `point_to_cv_target`) are pinned
to `config/fastdds_udp_only.xml`. That pinning buys visibility rather than
throughput: a node on shared memory is nearly invisible to `ros2 topic`/`node
list` run in a shell on the same machine, though other machines see it fine.
Pinning those six keeps pose, odom, TF and the CV target greppable from a
robot terminal. `dds_transport:=udp_only` puts every node this file launches
on UDP; it does not reach `sentry_localization`'s nodes or the camera launch.

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
`test_point_to_cv_target.py` the intercept solve, shot planner, latency stat,
and the node's subscriptions: `TargetState` and `RobotPose`, nothing else.
`pytest test/` also picks up the ament copyright, flake8 and pep257 checks,
which `colcon test --packages-select thornbots_pkg` runs too.

The localization drift suite is `ros2 launch sim
localization_tests.launch.py`, which launches `auto.launch.py`; see
`sim/README.md`.

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
within 0.5m will merge; nobody has fixed that. Centroid linkage would instead
split one spinning robot as its centroid wanders. Clustering runs in camera
frame, which is metric because every panel in a `PanelDetectionArray` shares
one camera pose.

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

The filter runs in `odom`. `root` moves with the sentry, which breaks constant
velocity under acceleration, and the camera also rotates with the gimbal.
`lookupTransform(odom, camera, capture + pose_latency_s)` corrects both.
Capture time is the detection stamp less `camera_latency_s` (0, unmeasured):
the EKF updates at capture, and the TF lookup asks for the camera there. `pose_latency_s` (0.01, unmeasured, inside the documented 3-25ms range)
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

`ArmorEKF` state is the centre's position, velocity and acceleration
(3-vectors, `POS`/`VEL`/`ACC` in `target_tracker_core.py`), the tracked
panel's normal yaw,
spin rate `w`, that panel's radius and its pair's height `dz` above the
centre, with the other pair's radius kept aside and its height at `-dz`. A
panel measures `centre + r * (cos yaw, sin yaw, 0) + (0, 0, dz)`. Only the
two pairs' difference is observable, so the centre is their mean height;
`dz` starts at 0 (0.05 m std), drifts at `q_dz` (0.005 m/sqrt(s)) and clamps
to +-0.15 m. Each detection
first associates to the nearest of the four predicted panels, skipping those
facing more than ~107 degrees from the camera; `k != 0` is a handoff, stepping
yaw by quarter turns and, on odd `k`, swapping radii and flipping `dz`. A cut at exactly 90 degrees
mis-assigned edge-on panels whenever the camera moved 10cm and held a wrong
spin for seconds. `PanelDetection.corners` stay unused:
`roi_depth_node.cpp`'s `deprojectDetection()` puts all four at one
`mean_depth_m`, so real corners carry no panel tilt.

Noise: `R` stddev is `meas_noise_base_m + meas_noise_range_coeff * range_m^2`
(depth error grows with z^2). `process_noise_accel` (2.0 m/s^2) drives the
centre, `process_noise_yaw_accel` (5.0 rad/s^2) the spin rate,
`process_noise_radius` (0.02) the radius, which clamps to 0.18-0.45m. These
came from an offline sweep against an emulator-shaped target; the higher yaw
noise let a jinking target's translation leak into `w`.

Acceleration is a Singer model: it decays over `accel_time_constant_s` (1.0)
and `process_noise_jerk` (3.0 m/s^3) drives it. From panel positions alone a
filter can't separate the centre accelerating from the panel spinning faster
than one spin period, so a loose model (jerk 20 and up) explained the 43 m/s^2
centripetal swing of a 2Hz panel as a jinking centre and slammed the radius
between its clamps. At jerk 3 it tracks the target's 6 m/s^2 braking at the
path ends: facing-panel p95 at 4 m/s went from 0.65 m to 0.12 m on
`sim/tools/estimation_offline.py` (2026-09-25). `acceleration` is published.

A panel seen alone faces the camera: its neighbours sit 90 degrees round, and
one within the detector's ~75 degree cone would be seen too. So a frame with
one panel also measures the tracked panel's yaw as the bearing to the camera,
+-`single_panel_yaw_std` (0.3 rad). Without it a still target's yaw was
unobservable and random-walked a quarter turn in 30 s, and since
`point_to_cv_target` rebuilds the facing panel from centre, yaw and radius, it
aimed up to 0.4 m off a target that never moved.

Innovation gating: a normalised innovation over `gate_nis` (16.3, 99.9% for 3
dof) is skipped, and `max_outliers` (3) in a row re-seed centre, velocity and
yaw from the panel while keeping `w` and the radii. Sim's `target_driver`
reverses instantly at each end of its path, which otherwise wrecked `w`.

`ArmorTracker` runs five `ArmorEKF`s seeded at `w` = 0, +-7, +-13 rad/s
(1-2Hz both ways) on every panel and publishes the lead, scored by an EWMA of
normalised innovation. A single filter fed 15cm noise for its first second,
as when sim's head slews in from rest, locked onto a wrong spin for good on 6
of 10 seeds; the bank recovered on 9. The lead changes only once a challenger
beats it by `switch_margin` (1.0) for `switch_after_s` (0.5), since a noise
burst briefly favours a collapsed-radius wrong-sign hypothesis. A hypothesis
trailing the lead by `reseed_margin` (3.0) for `reseed_after_s` (1.0) is
re-seeded from the lead's centre and yaw, with its own spin prior and fresh
radii.

The filter resets on a `robot_track_id` change or a `track_max_gap_s` gap.
It publishes on every `/cv/robot_panels` message it can place in odom, from
the first. `valid` goes true after 2 updates, because an engagement can be
shorter than one spin period; consumers should weigh `variance` and
`yaw_rate_variance`. `confidence` is the winning panel's, `panel` its measured
position, `radius` both pairs' radii, `z_offset` `[dz, -dz]`, and
`acceleration` the centre's. Each state is
`ArmorTracker.predicted()` at the publish time and stamped with it, as
`TargetState.msg` asks, so `point_to_cv_target` only extrapolates from there.
Part 2 owns every delay up to that stamp (`../CV_SPLIT_PLAN.md`, Estimation).

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
states. It runs in sim and on hardware. Sim's lidar doesn't see the robot at
all (see `sim/README.md`), so this filter is the only thing modelling the
blind sector.

The sector, `blind_angle_start` 0.09 to `blind_angle_end` 1.41 rad (5-81 deg
counter-clockwise from the gun, +x), comes from the `sentry_v2` CAD. We sliced
the Onshape export at the RPLIDAR's scan plane (z ~0.355 m, +-6 mm). The lidar
sits on the head at (-0.120, -0.185) in root, 0.22 m from the yaw axis, so it
pans with the gimbal.

- Head-fixed parts (gimbal body, the GM6020 and M2006 motors, the lidar's own
  cover and standoffs) block 17-79.5 deg, from 0.045 to 0.34 m out.
- The pitch stage (shooter, flywheels) blocks more as the gun pitches nose
  down. Over +-0.6 rad of pitch the union is 6-79 deg.
- Chassis parts top out at 0.347-0.348 m, 7 mm under the scan plane, so the
  chassis blocks nothing at any head yaw.

The defaults add 1 deg of margin to the 6-79.5 deg union.

On hardware the sector is unverified. The URDF gives the `lidar` frame no
rotation (same axes as the head, +x along the gun), but nobody has measured
where the real RPLIDAR's 0 deg points relative to the gun. The CAD's lidar
part has its local x 30.7 deg counter-clockwise of the gun, which may or may
not be the sensor's 0 deg. Check the sector against a real `/scan_raw` before
trusting it.

### point_to_cv_target.py

`/cv/target` `x/y/z` is a root-frame point Type-C aims at, where it used to be
a camera-relative vector. See `CVTarget.msg` and
`ros2_dji_serial_bridge/README.md`'s wire-format history.

The node reads `/cv/target_state` and `/pose` and nothing else, so anything
that publishes a `TargetState` can drive it: `target_tracker` on hardware,
`sim`'s `target_state_truth` on the aim bench. Liveness is the state's age
against `target_timeout_s` (0.5); confidence and `robot_track_id` come off
the message.

A timer publishes at `cv_target_publish_rate_hz` (30) from cached state. The
tracker runs at detection rate (up to ~60Hz), faster than Type-C's PID needs.
`_compute_aim_point()` handles three cases per tick:

- No usable state, or TF fails: zero confidence, with a throttled `ERROR` on
  TF failure. Usable means present and younger than `target_timeout_s`.
- `valid == False`: raw `panel` position, `lead_applied=False`,
  `track_valid=False`, no fire. No extrapolation off an unconverged track.
- `valid == True`: `plan_shot()`'s aim point in root. See below.

`plan_shot()` picks a mode per tick, with hysteresis on `|yaw_rate|`: spin
mode above `spin_enter_rad_s` (3.0), panel mode below `spin_exit_rad_s` (2.0).
Every mode extrapolates the center with `TargetState.acceleration` and solves
the intercept on the target's true path (curved by acceleration and spin),
not a straight line.

- Panel mode leads the panel facing the shooter at impact, with that pair's
  radius and `z_offset`, and fires now.
- Spin mode, shotgating (`chase_settle_s < 0`, the default): leads a point on
  the center-to-shooter line, at the radius and height of the pair arriving
  next. The line is steady, so the gimbal can hold it while panels sweep
  past. It fires with `delay_ms` set so a panel normal points along the line
  at impact, if that alignment falls within one publish tick; about one tick
  in five at 2 Hz spin. The pair switches only once the last shot at the
  current one has left the muzzle: its height is a step, and a switch any
  earlier moved the gun under that shot (staggered 0.5 m/s: 58% to 97%).
- Spin mode, chase (`chase_settle_s >= 0`): leads the facing panel itself and
  fires on any tick whose panel will have faced the shooter for
  `chase_settle_s` at impact, with `chase_margin_s` still to go. Both cover
  the gimbal's jump between panels; 0 and 0 fire every tick. The fire is
  delayed to leave mid-hold of whichever aim is current then, where that aim
  is exact. On the point bench's perfect gimbal it hits 94-98% of shots at
  every tick, against shotgating's one tick in five. A real gimbal has to
  make a ~7 deg jump every quarter turn; measure it before turning this on.

Two latencies, kept apart. `gimbal_lag_s` (0.05) is how far the gimbal trails
the aim point, past the half tick each aim is held: the aim leads by
`gimbal_lag_s` plus half a tick. `firmware_latency_s` (0.05) is fire decision
to muzzle exit, and times the fire against the spin. Aiming both from the
firmware latency overshot moving targets in sim by ~27 ms of their motion,
since the sim gimbal reaches a moving setpoint in 35 ms.

`plan_shot()`'s intercept is a time-of-flight fixed point, 2-3 iterations, with
no gravity, drag or elevation (Type-C handles those). `lead_enabled:=false`
aims at the current estimate and fires untimed.

`fire` and `delay_ms` ride on `CVTarget`, so the fire decision reaches the
MCB in the same frame as the aim it was solved for, measured from that
frame's `header.stamp` (see `ros2_dji_serial_bridge/UART_PROTOCOL.md`). The
firmware struct still has to grow to match before hardware timing works.

Both horizons start from this tick's `now - state.header.stamp`, not
`LatencyStat.mean`, because cached state ages between arrival and tick, by
up to a tracker period plus tick phase (measured: 20ms mean at arrival, 50ms
at tick), and the offset jitters. `LatencyStat` is logged as a diagnostic.

Frames convert by TF: `lookup_transform(root_frame, odom_frame, Time())`. For
lead, the reverse lookup gives shooter position in odom, and
`RobotPose.vel_x/vel_y` rotated by it gives shooter velocity. Both use the
latest transform, and the position is carried at that velocity to the state's
stamp, which is `plan_shot()`'s time zero.

Our own motion: the shot leaves where we are at the aim horizon and carries
our velocity, so `plan_shot()` returns a gun point, the intercept less our
motion from the stamp to impact, and the node sends `gun - shooter` rotated
into root rather than the intercept transformed. At 1 m/s and 3 m the
difference is ~0.15 m, three panel half-widths. A still shooter gets the intercept itself.
`solve_intercept()` is no longer on the node's path; only its tests use it.

Each publish tick with an aim point may fire, at most `fire_rate_hz` (2.0) and
only above `fire_confidence_threshold`, so a failed TF lookup or stale state
holds fire. HP, heat and power gating are not built.
