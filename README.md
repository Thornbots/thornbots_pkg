# thornbots_pkg

Hardware interface, robot description, and CV target selection for the Thornbots
ARC 2026 Sentry. It puts `/dji_serial_bridge/pose` (hardware or `sim`) and `/scan` on the graph,
runs `robot_state_publisher` off `urdf/sentry.urdf.xacro` (the `sentry_v2`
CAD's frames, with its meshes in `meshes/sentry_v2/`), publishes raw MCB
odometry as `/odom` and `odom->root`, and turns detections into a raw
world-frame (`odom`) `CVTarget`. Fused navigation uses `nav_odom`;
localization backends are in
`sentry_localization/README.md`; game rules are in
`../docs/ARCC_2026_SENTRY_CONTEXT.md`.

## Nodes

| Node | In | Out |
| --- | --- | --- |
| `pose_translator` | `/dji_serial_bridge/pose` | `/odom` (raw wheel odom), `/joint_states`. No TF. |
| `odom_tf_broadcaster` | `/odom`, `/localization/odom` | raw `odom->root` and fused `nav_odom->odom` TF |
| `lidar_self_filter` | `/scan_raw` | `/scan`, head blind sector blanked |
| `mcb_relay` | `/localization/odom`, `/odom`, `/cv/target` | `dji_serial_bridge_node`'s `~/relocalize`, `~/cv_target` |
| `target_selector` | `/cv/panel_detections`, `/dji_serial_bridge/ref_sys` (team colour) | `/cv/panel_detection` (one pick), `/cv/panel_polygon` (its corners), `/cv/robot_panels` (that robot's panels) |
| `target_tracker` | `/cv/robot_panels` | `/cv/target_state` (`TargetState`, odom frame, armor model); `/cv/tracker/measurement` (`Header` of each detection folded in); `clock_ack_topic` if set (bench only) |
| `point_to_cv_target` | `/cv/target_state`, `/dji_serial_bridge/pose` | `/cv/target` (`CVTarget`, odom frame, aim + fire decision, or a patrol point); `tick_topic` if set (bench only) |

`mcb_relay` forwards outgoing aim and relocalization messages to the bridge;
pose translation, target selection and aiming consume the incoming pose/referee
topics shown above. `auto.launch.py` starts the relay with
`real_hardware:=true`; sim's `mcb_*` launches also start it explicitly.
`point_to_cv_target` runs in both modes because
`/cv/target` also feeds sim's `cv_head_aim`. The CV nodes each have an enable
arg (`enable_target_selector`, `enable_target_tracker`,
`enable_cv_target_bridge`) and a ROS-free core for unit tests (`include/thornbots_pkg/*_core.hpp`, or
`armor_tracker.hpp` for `target_tracker`). Every node is C++ (`src/`); only
`launch/` is Python.

```
/dji_serial_bridge/pose --[pose_translator]--> /odom --[odom_tf_broadcaster]--> odom->root TF
                                           \-> sentry_localization --> /localization/odom --[odom_tf_broadcaster]--> nav_odom->odom TF
                          \-> /joint_states --[robot_state_publisher]--> rest of TF tree
/scan_raw (sllidar_node or sim) --[lidar_self_filter]--> /scan --> sentry_localization (map->nav_odom TF owned by slam_toolbox/amcl there)
TF: map -> nav_odom -> odom -> root

/localization/odom vs /odom            --[mcb_relay, drift-gated]-------> dji_serial_bridge_node (~/relocalize) --> UART --> MCB
/cv/panel_detections --[target_selector]--> /cv/robot_panels --[target_tracker]--> /cv/target_state
                                        \-> /cv/panel_detection (one pick), /cv/panel_polygon (rviz/foxglove)
/cv/target_state (armor model, confidence, track id) --[point_to_cv_target]--> /cv/target (odom frame)
/cv/target --[mcb_relay]--> dji_serial_bridge_node (~/cv_target) --> UART --> MCB
           \-[sim's cv_head_aim]--> /head_pan_cmd, /head_pitch_cmd (sim only, see sim/README.md)
```

## CV interface

The [TargetState](../ros2_dji_serial_bridge/msg/TargetState.msg) boundary lets
the [aiming bench](../sim/README.md#run-the-tests) replace perception with
perfect state. [target_tracker](#target_tracker) owns correction up to the
state's publish-time stamp; [point_to_cv_target](#point_to_cv_target)
extrapolates to the fire horizon. Detections, target state and aim use raw,
never-relocalized `odom`, since
`map` corrections can jump mid-shot; `map` is for strategy. See
[firmware/frame coordination](../ros2_dji_serial_bridge/README.md#aim-and-navigation-frames)
and [remaining work](../docs/ROADMAP.md#b-hit-while-we-move).

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

# Sim: start `ros2 launch sim sim.launch.py` first; it provides /dji_serial_bridge/pose and /scan.
ros2 launch thornbots_pkg auto.launch.py real_hardware:=false
```

`real_hardware` also sets `use_sim_time` (true when `real_hardware:=false`),
and setting it false keeps the launch off the real serial devices.

`localization_mode` (`mapping` default since 2026-10-02, `slam`, `amcl`,
`none`) picks the `map->nav_odom` owner. Under `mapping`, `load_map` defaults
false (a blank map from the boot pose) and `autosave_map` defaults to
`real_hardware`. `/localization/odom` is published in every mode.
`use_rf2o` (default `true`) controls whether the navigation pose fuses
`/scan_odom` or passes through raw odometry; when enabled it starts
`rf2o_laser_odometry_node` to scan-match `/scan`. The MCB's raw POSE remains
`/odom` regardless.
Both, plus `map_file`, `load_map` and `odom_frame`, pass through to
`sentry_localization`.

Raw aim coordinates use the current field-centred wire frame; see
[UART_PROTOCOL message types](../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types).
Navigation's corrected `nav_odom` does not change raw `odom` or the CV aim
frame.

```bash
ros2 launch thornbots_pkg auto.launch.py real_hardware:=false localization_mode:=mapping load_map:=false
ros2 launch thornbots_pkg auto.launch.py real_hardware:=false localization_mode:=none use_rf2o:=false
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
the RPLIDAR A2M8 on hardware. The driver uses `angle_compensate: true` to
keep a fixed angular grid (720 bins on the measured A2M8), required by
rf2o and slam_toolbox. Uncompensated scans vary in beam count.
The docstring at the top of
`launch/auto.launch.py` documents only `real_hardware`; the other args are
described in their `DeclareLaunchArgument` `description=` strings.

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

The unit tests exercise the ROS-free cores on synthetic input and need no
running graph. `colcon test` runs them all, gtest and the pytest lint checks:

```bash
cd /workspaces/isaac_ros-dev
colcon test --packages-select thornbots_pkg && colcon test-result --verbose
./build/thornbots_pkg/test_armor_tracker   # the tracker's gtest alone
```

`test_armor_tracker.cpp` covers the armor EKF, the hypothesis bank, handoffs,
gating and the still hypothesis; `test_target_selector_core.cpp` scoring,
centrality, grouping and hysteresis; `test_mcb_relay_core.cpp` the relocalize
decision; `test_point_to_cv_target_core.cpp` the intercept solve, shot planner,
latency stat and patrol; `test_pose_translator_node.cpp` the stamp, frame,
joint and covariance mapping; `test_point_to_cv_target_node.cpp` the publish tick,
the consumer acknowledgment and the node's subscriptions: `TargetState`,
`RobotPose` and `RefSysStatus` (patrol only), nothing else. The pytest run is
the ament copyright check of `include/`, `launch/`, `src/` and `test/`, and
the flake8 and pep257 checks of `launch/` and `test/`.

The localization drift suite is `ros2 launch sim
localization_tests.launch.py`, which launches `auto.launch.py`; see
`sim/README.md`.

## Notes

Design rationale, kept here so in-code comments stay short.

### pose_translator

The TF tree is `map -> nav_odom -> odom
-> root`: raw MCB POSE supplies `/odom` and `odom->root`, while localization
provides the fused navigation pose and `map->nav_odom`. The broadcaster
interpolates raw position at the fused pose stamp to publish translation-only
`nav_odom->odom`; navigation cannot rotate the aim axes. It does not substitute
latest data or extrapolate. A backwards stamp or a discontinuous raw POSE step
clears raw history and pending navigation samples. Boot/team selection can
reanchor raw coordinates, as defined by the [wire owner](../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types).

`nav_odom` is an extra navigation reference whose axes stay aligned with raw
`odom`, matching the MCB's translation-only offset. Its pose can change with
navigation corrections; it is not the continuous odometry frame described by
[REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst).
Raw `odom` retains that role after startup initialization.

Nobody has measured raw odom covariance. The placeholder is 1cm stddev on
position and velocity, everything else zero. It has to be non-zero so the
navigation EKF can weight `/scan_odom` (rf2o) against the raw POSE input.
Unset fields (z, roll, pitch, yaw) stay 0; `odom0_config` in `ekf.yaml`
fuses the adapter's explicit fixed navigation heading and excludes raw position.
The adapter does not transform `/odom` yaw through EKF-derived TF. Its fused
navigation pose is `nav_odom`; it does not replace raw `/odom` or
`odom->root`.

`root` is heading-fixed, a deviation from REP 105's body-fixed `base_link`,
as rf2o's `fixed_heading` and the EKF assume.
`RobotPose.chassis_yaw` (0 until the firmware sends it) goes out only as the
URDF's `chassis_yaw` joint, which turns `body`, the wheels and the armor
under `root`. The head hangs off `root` with `headlink` at the MCB's world
head yaw (both joints turn about +z, counter-clockwise from above like
`head_yaw`; they turned about -z until 2026-10-03, which mirrored the camera
whenever the gimbal was off zero), so `root->lidar` and `root->camera` don't depend on chassis yaw,
and a spinning chassis can't reach localization. `robot_state_publisher`
publishes every POSE joint state (90 Hz); its 20 Hz default left the head
TF up to 50 ms behind each lidar scan, and rf2o read the head's swing in
that gap as chassis motion ([rf2o grading](../sentry_localization/README.md#configrf2oyaml)). `vel_x/vel_y` are the
MCB's world-frame velocity, so they need no rotation either. Not REP-105's
body-fixed `base_link`: nothing on the robot needs one yet, and a turning
`root` would mean freeing rf2o's heading and fusing a yaw into the EKF.

### target_selector

Team colour comes from `RefSysStatus.is_on_blue_team`: on blue it drops class
IDs 0-3, on red 4-7. With no colour given it passes every detection through
and shoots at all targets, allies included: before the first `RefSysStatus`
(always, in sim without a referee), and while `robot_id` is 0. That's
taproot's `RobotId::INVALID`, which the MCB sends until the referee assigns
an ID, with the blue bit reading 0, so the colour bit alone would read red.

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

### target_tracker

The filter runs in `odom`. `root` moves with the sentry, which breaks constant
velocity under acceleration, and the camera also rotates with the gimbal.
`lookupTransform(odom, camera, capture + pose_latency_s)` corrects both.
`auto.launch.py` exposes `pose_latency_s` for capture-time calibration. E2E's
PTY fixture selects zero; the hardware default remains the unmeasured 10 ms.
Capture time is the detection stamp less `camera_latency_s` (0, unmeasured):
the EKF updates at capture, and the TF lookup asks for the camera there. `pose_latency_s` (0.01, unmeasured, inside the documented 3-25ms range)
covers the delay `dji_serial_bridge_node`'s stamp leaves in:
`RobotPose` is stamped with the last-byte read time less the frame's wire
time, so USB-serial/read buffering and the MCB's sample-to-send delay remain
([UART_PROTOCOL](../ros2_dji_serial_bridge/UART_PROTOCOL.md)). Sweep it on
hardware; the bridge already removes wire time, so don't add it here.

A missing transform logs an error and drops the detection, with no stale or
zero fallback. In sim `robot_state_publisher` is always up, so a silent
fallback would hide a broken TF tree until competition.

A transform that is only behind is waited for. Each detection queues until
TF covers its capture time, then updates the filter with the camera pose from
that moment. After `tf_max_wait_s` (0.25) it is dropped and the gap logged.
It is never matched to the newest camera pose instead: the bearing error
would be the gap times the head's slew rate. That fallback was the rule
until 2026-09-25, and the gz estimation bench logs showed it using poses 0.12-0.25 s stale
while the head tracked a moving target. Every 5 s the node logs how long
detections waited, how many it dropped, and the sim time from capture to
the filter update. Waiting detections are retried every 5 ms of wall time,
not sim time: the estimation bench's `bench_world` holds sim time until this node has folded
in each frame's detection, so a sim-time retry would never fire.

`TargetState` is stamped at publish time, so it can't show a backlog: a
tracker ten frames behind still publishes "now". `/cv/tracker/measurement`
echoes each folded-in detection's header, and `bench_world` paces on that.
Pacing on `/cv/target_state` let the tracker sit 0.12-0.21 s behind at
`real_time_factor:=0` (2026-09-26), its KeepLast(10) queue full.

The publish-time stamp is `now()`, which reads `/clock` on its own thread,
so a detection arriving just after a `/clock` update could be stamped with
that tick or the one before, and repeat runs of the estimation bench
diverged. With `clock_ack_topic` set (the bench sets
`/cv/tracker/clock_ack`; off by default), the node echoes every clock
update once `now()` reads it, and `bench_world` sends a step's detections
only after the echo of the last tick arrives.

For the estimation bench, `point_to_cv_target` can publish `state_ack_topic`
(off by default). It echoes each consumed model's Header, retaining the
model stamp and frame. A `tick_topic` Header records the decision time;
its frame names the output aim frame when an aim was published, otherwise
it is empty. The bench waits for model consumption and for the indicated
output point to arrive before advancing physics. Publication on separate
DDS topics alone does not establish that ordering.

Every TF lookup in `target_tracker` and `point_to_cv_target` is
non-blocking. `/tf` is serviced by the same executor as the detection
callback, and `Buffer.lookup_transform(timeout=...)` sleeps in a wall-clock
loop, so a 50ms wait per ~60Hz detection starved `/tf`. The buffer then fell
0.6-1.7s behind detections that TF itself was ~60ms ahead of, and the tracker
dropped nearly everything (2026-09-17, measured against a separate listener
on the same run). So `target_tracker` spins its listener on its own thread:
rclcpp's `TransformListener(spin_thread=true)` gives `/tf` a callback group
and executor of its own (rclpy's added the whole node to a second executor,
so the Python node used a separate `target_tracker_tf` node). A lookup that
fails while TF is behind waits; one that fails after TF caught up is retried
once before the detection is dropped, since the listener thread can catch up
between the two (2026-09-28: ~20 needless drops a run at ~27x).

The target is a 4-panel armor model, the standard RoboMaster anti-spin
tracker (rm_auto_aim's) reduced to position-only detections. On hardware
`class_id` is the robot's team and plate, the same on all four panels, so spin
can't come from handoffs; it has to come from geometry. The old `SpinDetector`
counted `class_id` changes and only worked because the emulator faked them.

`ArmorEKF` state is the centre's position, velocity and acceleration
(3-vectors at `POS`/`VEL`/`ACC` in `armor_tracker.hpp`), the tracked
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

Noise: `R`'s stddev along the camera ray is
`meas_noise_base_m + meas_noise_range_coeff * range_m^2` (depth error grows
with z^2); across it, `meas_noise_lateral_m` (0.04). `process_noise_accel` (2.0 m/s^2) drives the
centre, `process_noise_yaw_accel` (5.0 rad/s^2) the spin rate,
`process_noise_radius` (0.02) the radius, which clamps to 0.18-0.45m. These
came from a sweep against an emulator-shaped target; the higher yaw
noise let a jinking target's translation leak into `w`.

`radius_prior_std` controls the initial radius uncertainty around
`panel_radius_m`, including bank reseeds. Position reacquisition that keeps
spin also keeps the learned radius uncertainty. `log_innovations` enables
one diagnostic line per updated detection: capture time, track, panel count,
lead hypothesis, associated panel, result (update/outlier/reacquire = 0/1/2),
pre-update residual/NIS/log determinant and post-update spin/radii/score.
It is off by default; parameter defaults live in `src/target_tracker.cpp`.

Acceleration is a Singer model: it decays over `accel_time_constant_s` (1.0)
and `process_noise_jerk` (3.0 m/s^3) drives it. From panel positions alone a
filter can't separate the centre accelerating from the panel spinning faster
than one spin period, so a loose model (jerk 20 and up) explained the 43 m/s^2
centripetal swing of a 2Hz panel as a jinking centre and slammed the radius
between its clamps. At jerk 3 it tracks the target's 6 m/s^2 braking at the
path ends. `acceleration` is published.

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
negative log-likelihood (NIS + log det S). NIS alone rewards the loosest
model, whose wide S shrinks every innovation. A single filter fed 15cm noise for its first second,
as when sim's head slews in from rest, locked onto a wrong spin for good on 6
of 10 seeds; the bank recovered on 9. The lead changes only once a challenger
beats it by `switch_margin` (1.0) for `switch_after_s` (0.5), since a noise
burst briefly favours a collapsed-radius wrong-sign hypothesis. A hypothesis
trailing the lead by `reseed_margin` (3.0) for `reseed_after_s` (1.0) is
re-seeded from the lead's centre and yaw, with its own spin prior and fresh
radii.

A sixth, still hypothesis (`still_hypothesis`) covers a parked,
non-spinning target: velocity, acceleration and `w` pinned at exactly 0,
the centre and yaw random-walking at `still_process_noise_pos` (0.02
m/sqrt(s)) and `still_process_noise_yaw` (0.05 rad/sqrt(s)). The moving
filters read a still target's noise as motion (velocity p95 0.19 m/s on
the gz estimation bench), which Part 1 turned into lead. The still filter
takes the lead at a 0.25 margin rather than 1.0, since it wins by only
~0.6 per sample, and hands it back once a CUSUM of the per-sample
log-likelihood ratio against the best moving filter passes
`still_exit_llr` (15). A single gated detection adds ~13, so one misfire
can't knock it out. On the gz estimation bench the parked cells read velocity and spin
exactly 0 and facing-panel p95 1.3 cm (flat), from 4.1 cm. A target that
pulls away at 6 m/s^2 is handed over after about 0.23 s, with up to 9 cm
error in between (4 cm without the still model); that came from a
unit-test probe, and the estimation bench has no drive-off cell to confirm it yet.

The filter resets on a `robot_track_id` change or a `track_max_gap_s` gap.
It publishes on every `/cv/robot_panels` message it can place in odom, from
the first. `valid` goes true after 2 updates, because an engagement can be
shorter than one spin period; consumers should weigh `variance` and
`yaw_rate_variance`. `confidence` is the winning panel's, `panel` its measured
position, `radius` both pairs' radii, `z_offset` `[dz, -dz]`, and
`acceleration` the centre's. Each state is
`ArmorTracker::predicted()` at the publish time and stamped with it, as
`TargetState.msg` asks, so `point_to_cv_target` only extrapolates from there.
See the [CV interface](#cv-interface) for correction ownership.

### mcb_relay

The bridge stays a pure UART/DJI translator; this node reshapes upstream output
for it. `relocalize` compares fused `/localization/odom` with the estimated MCB
navigation position: raw `/odom` plus the last sent offset, using no TF (`Relocalizer` in
`mcb_relay_core.hpp`). Raw `/odom` never reflects a `RELOCALIZE`; the relay
sends gated absolute navigation positions to correct the navigation offset
while CV continues in the unchanged raw frame.

- The desired offset is `loc(t) - odom(t)` at the localization stamp,
  with `/odom` interpolated there from a 1 s buffer.
- The navigation position sent is `loc(t)`, adjusted for estimated message
  delay. Its age already includes the inbound UART leg because the
  [bridge stamp](../ros2_dji_serial_bridge/UART_PROTOCOL.md#mcb--jetson)
  estimates MCB send start. Add only the outbound `uart_latency_s` and
  `mcb_read_delay_s`; both are placeholders until measured on the robot.
  The point is stamped with the estimated apply time. The wire drops the
  stamp; see the [wire contract](../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types).
- It is sent only when confident: the localization's xy std, combined with
  speed x `latency_std_s` (3 ms), stays under `max_std_m` (0.02), and the
  change from the last sent offset clears both `error_threshold_meters` (0.05) and `n_sigma` (3) of
  that std. The EKF's std reads ~1 mm at rest and ~7 mm at 1 m/s in sim.
- After a send it waits `hold_off_s` (0.3 s) before another correction,
  limiting correction rate; an unchanged offset is not resent. Raw `/odom`
  does not echo corrections. Clock resets/raw reanchors forget the sent offset.
  UART has no correction acknowledgement: the stored offset estimates the MCB
  state and cannot detect mailbox loss; hardware validation remains open.

The bridge packs the navigation position using the existing `RELOCALIZE`
payload. Raw POSE and the CV aim frame remain unchanged. See the
[wire contract](../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types).

`cv_target` is a straight republish, and carries the fire decision with the
aim point it was solved for.

### lidar_self_filter

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

### point_to_cv_target

`/cv/target` `x/y/z` is an `odom` point (`header.frame_id`) the MCB holds
and aims at from wherever the chassis is, since 2026-09-27. It was a
root-frame point before that and a camera-relative vector before that. See
`CVTarget.msg` and
[UART_PROTOCOL](../ros2_dji_serial_bridge/UART_PROTOCOL.md).

The node aims from `/cv/target_state` and `/dji_serial_bridge/pose` alone, so anything that publishes a `TargetState` can drive it: `target_tracker` on hardware,
`sim`'s `target_state_truth` on the aiming bench. Liveness is the state's age
against `target_timeout_s` (0.5); confidence and `robot_track_id` come off
the message.

A timer publishes at `cv_target_publish_rate_hz` (40) from cached state. The
tracker runs at detection rate (up to ~60Hz), faster than Type-C's PID needs.
`compute_aim_point()` handles three cases per tick:

- No usable state, or TF fails: no target aim; Jetson patrol may supply a
  non-firing point instead. TF failure logs a throttled `ERROR`.
  Usable means present and younger than
  `target_timeout_s`. With `tick_topic` set (the lockstep benches), a
  `Header` still goes out every tick.
- `valid == False`: raw `panel` position, no lead, no fire. No
  extrapolation off an unconverged track.
- `valid == True`: `plan_shot()`'s aim point in odom. See below.

`plan_shot()` picks a mode per tick, with hysteresis on `|yaw_rate|`: spin
mode above `spin_enter_rad_s` (3.0), panel mode below `spin_exit_rad_s` (2.0).
Every mode extrapolates the center with `TargetState.acceleration` and solves
the intercept on the target's true path (curved by acceleration and spin),
not a straight line.

- Panel mode leads the panel facing the shooter at impact, with that pair's
  radius and `z_offset`, and fires now. `choose_panel()` selects the most
  face-on normal near impact, then holds that physical panel by its odom yaw
  until another faces us `panel_switch_margin_rad` more squarely. A new track,
  invalid state, spin mode or timeout clears the hold, including a late update
  arriving before the timeout timer runs. Holding by yaw survives quarter-turn
  relabeling of the target model. Zero margin gives the immediate best-panel
  choice; finite nonnegative margins below pi/2 are accepted.
- Spin mode, shotgating (`chase_settle_s < 0`, default -1): leads a point on
  the center-to-shooter line, at the radius and height of the pair arriving
  next. The line is steady, so the gimbal can hold it while panels sweep
  past. It fires with `delay_ms` set so a panel normal points along the line
  at impact, if that alignment falls within one publish tick; about one tick
  in five at 2 Hz spin. The pair switches only once the last shot at the
  current one has left the muzzle: its height is a step, and a switch any
  earlier moved the gun under that shot (staggered 0.5 m/s: 58% to 97%).
  This is the default after 1 Hz hosted tests showed persistent yaw error
  while chasing despite fresh detections and valid tracks. It changes only
  spin mode; nonspinning targets retain panel lead and hysteresis. The
  [accuracy investigation](../sim/docs/aim-accuracy-2026-10-12.md) records the
  filter, lead and fire-policy ablations and repeat measurements.
- Spin mode, chase (`chase_settle_s >= 0`, opt-in): leads the facing panel itself and
  fires on any tick whose panel will have faced the shooter for
  `chase_settle_s` at impact, with `chase_margin_s` still to go. Both cover
  the gimbal's jump between panels; 0 and 0 fire every tick. The fire is
  delayed to leave mid-hold of whichever aim is current then, where that aim
  is exact. On the point bench's perfect gimbal it hits 94-98% of shots at
  every tick, against shotgating's one tick in five. A real gimbal has to
  make a ~7 deg jump every quarter turn and settle; measure that on hardware
  and set `chase_settle_s` to it, or fall back to shotgating.
  `auto.launch.py` exposes both chase window controls and the independent
  gimbal/firmware latency estimates for calibration.

The panel margin was selected on the 45-degree aiming bench to suppress
noise-driven switches while preserving slow-turn panel transitions. The
[three-run measurement](../sim/docs/panel-choice-2026-10-11.md#aiming-bench)
compares margins, exact-yaw controls and synthetic orientation-error stress.
It uses a perfect gimbal and does not establish hardware calibration or
measured tracker-noise tolerance.

Two latencies, kept apart. `gimbal_lag_s` (0.05) is how far the gimbal trails
the aim point, past the half tick each aim is held: the aim leads by
`gimbal_lag_s` plus half a tick. `auto.launch.py` exposes this calibration;
the hardware default remains 50 ms. After filtering absolute setpoint rate, the hosted E2E fixture uses zero
yaw lag from
command/achieved-angle replay in the
[accuracy investigation](../sim/docs/aim-accuracy-2026-10-12.md). `firmware_latency_s` (0.094) is indexing request
to muzzle exit, and times the fire against the spin. Aiming both from the
firmware latency overshot moving targets in sim by ~27 ms of their motion,
in the earlier fixture. The outgoing wire delay includes this indexer estimate
plus the planner's alignment wait, matching the
[wire contract](../ros2_dji_serial_bridge/UART_PROTOCOL.md#cv_target-id1--aim-point-and-fire-decision).
The sentry default matches the 94 ms hosted chamber-exit calibration; hardware timing remains unmeasured.

`plan_shot()`'s intercept is a time-of-flight fixed point, 2-3 iterations, with
no gravity, drag or elevation (Type-C handles those). `lead_enabled:=false`
aims at the current estimate and fires untimed.

`fire` and `delay_ms` ride on `CVTarget`, so the fire decision reaches the
MCB in the same frame as the aim it was solved for, and the MCB runs
`delay_ms` from receiving that frame (no stamp crosses the wire; see
`ros2_dji_serial_bridge/UART_PROTOCOL.md`). The firmware already reads the
15-byte payload; remaining timing and frame work is tracked in the bridge's
[firmware status](../ros2_dji_serial_bridge/README.md#where-the-firmware-stands).

Both horizons start from this tick's `now - state.header.stamp`, not
`LatencyStat.mean`, because cached state ages between arrival and tick, by
up to a tracker period plus tick phase (measured: 20ms mean at arrival, 50ms
at tick), and the offset jitters. `LatencyStat` is logged as a diagnostic.

Aim state and shooter pose share raw `odom`; lead uses `odom->root` at the
state stamp and world-frame `RobotPose.vel_x/vel_y`. If the state stamp is
newer than the newest raw POSE, the latest transform is used and its position
is carried at that velocity to the state's stamp. The lookup does not wait,
since waiting in a callback starves `/tf`. Localization corrections do not
move the CV target or change the outgoing aim frame. See
[UART_PROTOCOL message types](../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types)
for the wire frame and payload contract.

Our own motion: the shot leaves where we are at the aim horizon and carries
our velocity, so `plan_shot()` returns the intercept less our motion over
the flight: the odom point a gun at our exit position points through. At
1 m/s and 3 m the difference is ~0.15 m, three panel half-widths. A still
shooter gets the intercept itself.
`solve_intercept()` is no longer on the node's path; only its tests use it.

Each publish tick with an aim point may fire, at most `fire_rate_hz` (2.0) and
only above `fire_confidence_threshold`, so a failed TF lookup or stale state
holds fire. HP, heat and power gating are not built.

Every frame carries `type_c_based_patrol` (default false) and a per-frame
`turn_to_hit` decision. A held target estimate clears `turn_to_hit`, including
an unconverged estimate or a coasting/predicted track during brief occlusion.
When the estimate expires, patrol frames set it. Reacquisition clears it on
the next frame, with no cooldown. The decision does not depend on whether the
aim solve succeeds or the frame permits firing.

Every Jetson patrol point sets `CVTarget.patrol`;
target aim points leave it clear, including non-firing holds. This identifies
the source of the point and is independent of `type_c_based_patrol`, which
permits firmware patrol. Wire semantics live in the
[CV_TARGET flag table](../ros2_dji_serial_bridge/UART_PROTOCOL.md#cv_target-id1--aim-point-and-fire-decision).

The Jetson patrols itself (`patrol_enabled`, default true; `auto.launch.py`
arg of the same name). After `patrol_after_s` (0.2) with no aim point it
sends patrol points at the publish rate, never with `fire` set: a point
`patrol_range_m` (3.0) out from the `muzzle` frame, `patrol_pitch_down_rad`
(0.05) below level, its yaw starting at the gun's and turning at
`patrol_rate_rad_s` (-2.0, clockwise). Firmware patrol is slower: the compiled MCB uses -0.0002 rad per 1 ms
cycle (0.2 rad/s clockwise), with pitch 0.05. The Jetson sweep remains its
separate configured rate. The yaw
is integrated on itself, not on the gun's, so a gun that lags can't stall
the sweep. Firmware owns turns toward hits: REF_SYS reports a chassis-relative
angle, while the current POSE payload does not carry chassis yaw. The removed
Jetson `hit_angle_sign`/`hit_turn_s` path incorrectly added that angle to gun
yaw. The `turn_to_hit` parameter is also removed; the decision now follows
estimate availability on each frame.

Why the Jetson and not the MCB: a patrol frame keeps a `CV_TARGET` going
between targets, so the MCB hears the flags all the time and never patrols
on its own. MCBV3#78 found the other way broken: the MCB only patrols with
no frame coming, the one time it hears no flags. A patrol frame is an
aim point with `fire` clear and `patrol` set; the payload size is unchanged. That holds because
MCBV3 `nightly` requests a shot only when `fire` is set
(`AutoAimAndFireCommand.cpp`). The aiming and estimation benches disable
patrol; every `mcb_*` match stage enables it. The old E1 scorer is gone.
Firmware may interrupt Jetson patrol toward a hit only while no target
estimate is held. Its next target frame cancels that turn. See
[firmware coordination](../ros2_dji_serial_bridge/README.md#where-the-firmware-stands).
The [three-run hosted comparison](docs/hit-permission-2026-10-15.md#hosted-drive-and-match)
records the unit checks and drive/match regression guard against nightly with
the angle-only firmware correction applied.



### Initial field position

`auto.launch.py initial_x:=4.625 initial_y:=0.0` seeds the EKF and AMCL
at a known team spawn; the default stays `(0, 0)`. `root` remains heading-fixed.
Scan odometry initializes from the first `/odom` pose so it shares the
firmware's origin. Starting the EKF at zero with a nonzero firmware pose
would otherwise make `mcb_relay` immediately reset the firmware to zero.
