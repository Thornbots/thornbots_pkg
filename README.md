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
| `match_wifi_guard` | `/dji_serial_bridge/ref_sys` | opt-in host Wi-Fi disconnect request |
| `mmm` | PING, `/scan`, `/odom`, `/joint_states`, referee team, `map->root` TF | PING, SLAM reset and localization save service; opt-in host shutdown request |
| `mcb_relay` | `/localization/odom`, `/odom`, `/cv/target` | `dji_serial_bridge_node`'s `~/relocalize`, `~/cv_target` |
| `target_selector` | `/cv/panel_detections`, `/dji_serial_bridge/ref_sys` (team colour) | `/cv/panel_detection` (one pick), `/cv/panel_polygon` (its corners), `/cv/robot_panels` (that robot's panels) |
| `target_tracker` | `/cv/robot_panels` | `/cv/target_state` (`TargetState`, odom frame, armor model); `/cv/tracker/measurement` (`Header` of each detection folded in); `clock_ack_topic` if set (bench only) |
| `point_to_cv_target` | `/cv/target_state`, `/dji_serial_bridge/pose` | `/cv/target` (`CVTarget`, odom frame, aim + fire decision, or a non-firing idle look); `tick_topic` if set (bench only) |

`mcb_relay` forwards outgoing aim and relocalization messages to the bridge;
pose translation, target selection and aiming consume the incoming pose/referee
topics shown above. `auto.launch.py` starts the relay with
`real_hardware:=true`; sim's `mcb_*` launches also start it explicitly.
`point_to_cv_target` runs in both modes because
`/cv/target` also feeds sim's `cv_head_aim`. The CV nodes each have an enable
arg (`enable_target_selector`, `enable_target_tracker`,
`enable_cv_target_bridge`) and a ROS-free core for unit tests (`include/thornbots_pkg/*_core.hpp`, or
`armor_tracker.hpp` for `target_tracker`). Runtime nodes are C++ (`src/`);
launch and the operator workflow use Python.

```
/dji_serial_bridge/pose --[pose_translator]--> /odom --[odom_tf_broadcaster]--> odom->root TF
                                           \-> sentry_localization --> /localization/odom --[odom_tf_broadcaster]--> nav_odom->odom TF
                          \-> /joint_states --[robot_state_publisher]--> rest of TF tree
/scan_raw (sllidar_node or sim) --[lidar_self_filter]--> /scan --> sentry_localization (map->nav_odom TF owned by slam_toolbox/amcl there)
TF: map -> nav_odom -> odom -> root
/dji_serial_bridge/ref_sys --[match_wifi_guard, opt-in]--> host Wi-Fi disconnect request
PING / scan / odom / joints / referee / TF --[mmm]--> PING, slam_toolbox/reset, map_autosaver/save_now

/localization/odom vs /odom            --[mcb_relay, drift-gated]-------> dji_serial_bridge_node (~/relocalize) --> UART --> MCB
/cv/panel_detections --[target_selector]--> /cv/robot_panels --[target_tracker]--> /cv/target_state
                                        \-> /cv/panel_detection (one pick), /cv/panel_polygon (rviz/foxglove)
/cv/target_state (armor model, confidence, track id) --[point_to_cv_target]--> /cv/target (odom frame)
/cv/target --[mcb_relay]--> dji_serial_bridge_node (~/cv_target) --> UART --> MCB
           \-[sim's cv_head_aim]--> /head_pan_cmd, /head_pitch_cmd (sim only, see sim/README.md)
```

### match_wifi_guard

`auto.launch.py match_wifi_off:=true` starts this C++ node; it is off by default,
including in simulation where the MCB emulator starts in battle. It subscribes
to `/dji_serial_bridge/ref_sys` (`RefSysStatus`, SensorDataQoS) and requires
three consecutive messages with `game_stage == 4`. Any other stage resets the
count. Once confirmed it attempts exactly one atomic request per node/container
run, including when writing fails, and never reconnects on match end.
`match_wifi_request_file` forwards to the node's `request_file` parameter.
The ROS-free core tests cover debounce, one-shot behavior, exact contents,
permissions, temporary-file cleanup and a missing parent directory.
See the [host opt-in and request contract](../isaac-ros-startup/README.md#match-wi-fi-disconnect-opt-in)
for installation, Wi-Fi restoration and operational effects.

### MCB Mapping Machine

`auto.launch.py` starts the C++ `mmm` node with the normal stack. Disable it
with `enable_mmm:=false`, including as a launch argument to the manual
[operator workflow](#field-day-and-room-workflow). Only one PING coordinator
may run. The [wire contract](../ros2_dji_serial_bridge/UART_PROTOCOL.md#ping-id5--mcb-mapping-machine-and-link-check)
owns all MCB states, acknowledgments and retry timing.

On start, MMM requires the running `localization_mode` to be `mapping` and
calls Jazzy's `/slam_toolbox/reset` (`slam_toolbox/srv/Reset`,
`pause_new_measurements=false`) to clear the graph inside the live stack.
It creates `map_save_dir/mmm-<system-clock count>/`, then captures `boot.json`
and `session.json` after fixed-geometry fresh lidar, raw `/odom`, fresh
chassis joints and map TF, and at least two stationary seconds. Field capture
also requires the referee team and raw `/odom` within 0.30 m of the team start;
practice without a referee uses `seed_explicit_team:=red|blue`.
Sim defaults `mmm_room:=true`;
hardware defaults false. `mmm_room:=true` captures room coordinates. Setup
failure or the `mmm_timeout_s` wall deadline sends 6; it never stops the service.
A fresh start after the wire work window creates a new session and resets the
mapping graph and readiness evidence. A start while saving sends 6 without
interrupting the save. Transient invalid scans restart fresh-scan evidence and
the boot stationary window; a real lidar geometry change fails setup and blocks
saves for that session.

The ROS-free `mmm_core` owns protocol bookkeeping, pose evidence and save
selection. Readiness requires TF no older than 0.5 s, `map->root` within
0.50 m and 15° (0.2618 rad) of captured boot to enter, and two seconds parked
within 0.03 m/0.03 rad. Hysteresis retains readiness up to 0.60 m and
18° (0.3142 rad). Leaving the outer limits or losing evidence announces mapping;
a parked return announces ready. The node never mirrors the MCB state machine.

Save requests are acknowledged before asynchronous work. Only the last completed
2 exchange plus a current parked start selects a validated `snapshot/`;
all other saves use distinct `incomplete-<counter>/` directories and are marked
unvalidated. Setup/busy requests are acknowledged without concurrent saves.
[Localization's save service](../sentry_localization/README.md#explicit-mapping-snapshots)
owns graph/image writing and bundle validation. MMM carries boot metadata into
the bundle and guards the parked condition through completion. In field mode,
this save guard allows raw `/odom` within 0.50 m of the team start, while boot
capture keeps the tighter mark requirement above. Chassis yaw must remain within
0.03 rad of boot during capture and saving. Candidates stay
unvalidated until this guard passes; required files and directories are fsynced
before publishing `snapshot/` or requesting shutdown. Incomplete saves
are recovery material; registration and deployment remain manual steps. A failed
Ready save sends 4 and waits for its exchange before announcing 2 again at start.

Host shutdown is disabled by default. `mmm_shutdown_after_save:=true` writes
`MMM_READY_SAVE_COMPLETE` followed by a newline to
`mmm_shutdown_request_file` only after a successful guarded Ready save.
Install the [opt-in host mechanism](../isaac-ros-startup/README.md#mmm-shutdown-opt-in)
to expose that path inside the container. Missing/unwritable paths are logged;
no shutdown is attempted from the container itself. Incomplete saves never request
shutdown. ACK means receipt, not durable storage; keep power on until saving
finishes. Physical storage/power timing still needs robot acceptance.

The full-stack fake-MCB launch test is opt-in because `sim` depends on this
package. Build `sim`, then rebuild `thornbots_pkg` with
`--cmake-args -DTHORNBOTS_MMM_SIM_TEST=ON` and run `colcon test
--packages-select thornbots_pkg`. It exercises start, ready, driving away,
parked return and validated save using the normal sim/robot entrypoints.
`test_mmm_core` covers protocol loss/retry termination, StartGate hysteresis,
save guards and failure recovery without ROS.

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

`localization_mode` (`amcl` default since 2026-10-30, `mapping`, `slam`,
`none`) picks the `map->nav_odom` owner. Under `mapping`, `load_map` defaults
false (a blank pose graph) and `autosave_map` defaults to
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
the RPLIDAR A2M8 on hardware. `lidar_inverted` (default `true`, the lidar is
mounted upside down) sets the driver's `inverted`, which negates scan angles so
the `lidar` frame stays z-up for rf2o, AMCL and slam_toolbox. The driver uses `angle_compensate: true` to
keep a fixed angular grid (720 bins on the measured A2M8), required by
rf2o and slam_toolbox. Uncompensated scans vary in beam count.
`lidar_x`, `lidar_y`, `lidar_z` (meters in `head`) and `lidar_yaw` (radians CCW
from head +X) override the lidar TF at launch; defaults use the Oct 31 CAD
translation and robot-measured scan-zero yaw (see [self-filter](#lidar_self_filter)).
`lidar_blind_angle_start` / `lidar_blind_angle_end` (-1.927 / -0.171 rad in the scan
frame) set the [self-filter sector](#lidar_self_filter), including across scan zero.
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
Agents stopping backgrounded launches use `kill_launch.sh`: [Docker skill](../.claude/skills/isaac-ros-docker/SKILL.md#helper-scripts-never-hand-rolled-docker-exec).

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
latency stat and idle looks; `test_pose_translator_node.cpp` the stamp, frame,
joint and covariance mapping; `test_point_to_cv_target_node.cpp` the publish tick,
the consumer acknowledgment and the node's subscriptions: `TargetState`,
`RobotPose` and `RefSysStatus` (idle team/hit events), nothing else. The pytest run is
the ament copyright check of `include/`, `launch/`, `src/` and `test/`, and
the flake8 and pep257 checks of `launch/` and `test/`.
It also runs the ROS-free operator workflow geometry, artifact, convergence
and process-lifecycle regressions in `test/test_operator_workflow.py`.

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

#### Odometry pods

The [CAD pod table and firmware channel/sign derivation](../docs/investigations/sentry-cad-2026-10-26.md#odometry-pods)
record the start-frame encoder axes, red/blue signs and exported wheel geometry.
The motor-to-CAD yaw datum is unknown, so source-part/channel labels are conditional.
The pods were omitted from the imported model; its four `wheel_*` links are
drive wheels, not encoder pods.

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
Empty detection frames leave the incumbent and its `robot_track_id` unchanged;
reappearance within the default 1 m gate retains the ID, including after a
short detection dropout. Camera/target motion beyond that gate can create a new ID.

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

### Robot geometry

`urdf/sentry.urdf.xacro` uses the team's 2026-10-26 low-resolution CAD.
Link and joint names retain their existing meanings: `head` +x is the gun's
horizontal forward direction, +y is left, and positive head yaw is CCW.
The CAD gun points along `lpr_yaw` -y and is elevated 20° in its exported
pose; imported pitch assets are rebased so zero pitch is horizontal.
`camera` is at the RGB FOV apex (44 mm left of the IR apex) with REP 103 body
axes, not optical axes: `/cv/panel_detections` are deprojected with the colour
intrinsics, so the frame is the colour lens.
`muzzle` is on the pitch axis between the flywheels, pointing along
pitch-frame +x. Its 1.903 mm lateral offset comes from the CAD bore centre;
the bore front is 172.976 mm ahead of this aiming origin.

The clearance datum places the lowest exported wheel on z=0. The exported
wheel bottoms differ by 32.5 mm, so this does not establish the real settled
ground plane. The heading-fixed [root convention](#pose_translator) is
unchanged. The [dated CAD investigation](../docs/investigations/sentry-cad-2026-10-26.md)
records the archive checksum, derivation, transform comparison, tests and
hardware checks still required. The earlier Oct 25 position-only fit differed
from CAD by **10.6 cm** in XY; the Oct 26 joint fit resolves the scan-zero yaw
and supports the CAD radius (see [self-filter](#lidar_self_filter)).

### lidar_self_filter

The lidar is bolted to the head, so the head's blind sector is fixed in the
lidar frame whatever the yaw, and a static angular filter needs no joint
states. It runs in sim and on hardware. Sim's lidar doesn't see the robot at
all (see `sim/README.md`), so this filter is the only thing modelling the
blind sector.

The lidar was flipped upside down on 2026-10-31 to lower the scan plane.
The mount translation in `head` is the team's Oct 31 CAD (-0.0056,
-0.193326, 0.063707) m. With `lidar_inverted`, scan zero is 2.579 rad
(147.75 degrees) CCW of the gun, and the sector is -1.927..-0.171 rad
(-110.4..-9.8 degrees) in the scan frame: the level-pitch head block plus
5 degrees each side. Both come from one parked robot pose; nose-up/down
sectors and a head-turn fit are still to be measured. The
[dated measurement note](../docs/investigations/lidar-flip-2026-10-31.md)
records the evidence; the [Oct 26 note](../docs/investigations/lidar-mount-2026-10-26.md)
covers the previous upright mount.

The filter compares scan-frame angles without TF. Negative bounds and sectors
crossing scan zero are supported. Keep the simulated sensor mount and robot
TF consistent when overriding the mount; sim uses this same filter.

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

- No usable state, or TF fails: no target aim; idle logic supplies a
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
[accuracy investigation](../sim/docs/aim-accuracy-2026-10-12.md). `firmware_latency_s` (0.020) is indexing request
to muzzle exit, the sentry's `FIRING_LATENCY_TIME` timed on the robot
(2026-10-28). The wire delay never goes below `min_fire_delay_s` (0.040), so
the MCB always waits before requesting; the larger of the two times the fire
against the spin. Aiming both from the
firmware latency overshot moving targets in sim by ~27 ms of their motion,
in the earlier fixture. The outgoing wire delay includes this indexer estimate
plus the planner's alignment wait, matching the
[wire contract](../ros2_dji_serial_bridge/UART_PROTOCOL.md#cv_target-id1--aim-point-and-fire-decision).

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

Every publish tick with a shot solution, a satisfied fire lock and confidence at least
`fire_confidence_threshold` (0.5) requests fire: up to 40 Hz at the default
publish rate, with no Jetson fire-rate throttle. Each request carries the
planner's alignment wait plus the exit delay, rounded to milliseconds and
clamped to uint16 (40 ms minimum with the defaults). The MCB holds a pending
request and ignores newer frames until it completes; see the
[wire contract](../ros2_dji_serial_bridge/UART_PROTOCOL.md#cv_target-id1--aim-point-and-fire-decision).
Failed TF, unconverged tracks and stale detections still hold fire, as do the
spin alignment and chase windows above. HP, heat and power gating are not built.

`TargetState.detection_stamp` carries the newest contributing detection's corrected
capture time; `header.stamp` still identifies the prediction time. Firing requires
that capture stamp to be at most
`fire_max_state_age_s` (0.15 s) old. A held prediction still aims, without
firing, until `target_timeout_s` (0.5 s). The fire-age limit is a node parameter,
not a launch argument.

`fire_lock_s` (1.0 s) requires tracking of the same
`TargetState.robot_track_id`, starting at the first observation with a converged,
fresh track above the confidence threshold. Panel handoffs keep that identity.
`fire_lock_grace_s` (0.3 s, nine frames at 30 fps) preserves the dwell start
through brief invalid, low-confidence or stale/future states. Gap time counts
toward the dwell, but firing stays blocked until all gates pass again. Longer
ineligible gaps, identity changes (even between publish ticks), and target timeout
reset the lock. State callbacks and publish ticks both observe eligibility.
Elapsed time uses the node clock, as does fire age; a backwards clock jump restarts
the dwell. Planner shot windows do not reset it. Setting `fire_lock_s` to 0
disables only this wait; the other firing gates still apply.

Stationary targets bypass the dwell: the existing spin hysteresis must be outside
spin mode, and the norm of `TargetState.velocity` (centre velocity in odom,
including z) must be below `fire_stationary_speed_mps` (0.1 m/s). This tolerance
allows small parked-target velocity noise; it is not a measured hardware threshold.
All three lock parameters accept finite nonnegative values and are node parameters,
not launch arguments. The 40 ms minimum exit delay gives the gimbal more time to
acquire each aim point before firing.

Every non-target frame clears `fire`, sets `turn_to_hit`, and sets the existing
`CVTarget.patrol` bit to identify an idle look. Target points leave both bits
clear, including non-firing coasting estimates. The wire layout is unchanged;
see the [CV_TARGET flag table](../ros2_dji_serial_bridge/UART_PROTOCOL.md#cv_target-id1--aim-point-and-fire-decision).
`type_c_based_patrol` remains false by default. There is no Jetson patrol or
patrol launch argument.

Idle looks choose the last enemy until 3 s after the 0.5 s target hold expires,
or follow the gun for 2 s after REF_SYS reports a hit; the newer capture/event
wins. The last predicted position is held once aiming expires. Firmware owns
the hit turn; the Jetson never converts its chassis-relative hit angle. With
neither recent event, `idle_direction:=zone` (default) looks at the enemy's
starting zone. `idle_direction:=hold` follows the current gun direction while
continuing non-firing frames, so hit-turn permission remains live.

The enemy zone mirrors only our firmware start x: red boots at (-4.625, 0),
blue at (+4.625, 0). `zone_start_x_m` and `zone_start_y_m` are also `auto.launch.py` arguments,
so simulation fixtures can express the zone in their scene frame. They default
to `OdometrySubsystemConstants.hpp`; the REF_SYS blue flag alone chooses the
side, including late blue arrival and missing/zero robot ID. Idle points use
`idle_height_m` (0.15), near armor height. Idle yaw commands change at most
`idle_rate_rad_s` (0.5 rad/s); a new bearing must differ by `idle_switch_rad`
(pi/6, 30 degrees) before adoption. Hit/hold looks follow the actual gun so
firmware hit turns are not pulled back. Target aiming retains its existing
response. Before muzzle TF is available, idle frames retain the last commanded
point, or the enemy zone at boot.

For the robot service, set `AUTO_LAUNCH_ARGS="idle_direction:=hold"` in
`/etc/thornbots/launch.env` and restart `thornbots` to hold instead of looking
at the zone. The startup script already forwards these arguments to
`auto.launch.py`; other launch settings can remain in the same variable.

### Initial field position

See [localization's initial-position contract](../sentry_localization/README.md#initial-field-position)
and [field-day and room workflow](#field-day-and-room-workflow)
for navigation seeds, independent AMCL field poses and mapping a new field.

## Field-day and room workflow

`operator_workflow` runs the mapping, boot-capture, save, registration and AMCL acceptance steps.
`$SESSION` names one attempt's folder; choose a new name for every attempt.
`--room` rehearses without a referee, keeping boot coordinates instead of field registration.

### Field-day card — keep the robot parked at its start for steps 2, 4 and 5

Before power-on, place the robot on its start mark and keep it still while the
IMU settles; moving it during settling caused heading drift. After any SWD
flash, power-cycle the MCB before trusting odometry: a warm reset leaves the
Pico I2C dead. MCB power-cycling also reboots the Jetson (shared power); re-sync
the clock, restart the container and begin a new session. Turning the controller
off during a session invalidates that attempt; keep it on throughout.

1. Stop the existing robot service with `sudo systemctl stop thornbots`
   (or stop the manual launch). In a sourced robot-container terminal,
   choose a **new** session and start mapping (keep this terminal open):
   `export SESSION=/workspaces/isaac_ros-dev/maps/oct31-red-01`
   then `ros2 run thornbots_pkg operator_workflow mapping "$SESSION"`.
2. In a second terminal, set the same `SESSION`; check head yaw ≈ 0 with
   `ros2 run tf2_ros tf2_echo root head` (Ctrl-C after checking), then capture **before moving**:
   `ros2 run thornbots_pkg operator_workflow boot-pose "$SESSION"`.
3. Drive ≤0.4 m/s with the head still, covering the whole field
   ([broad-field coverage](../sentry_localization/README.md#broad-field-coverage)). Before crossing rough or
   raised ground: `ros2 run thornbots_pkg operator_workflow pause "$SESSION"`.
   Stop on level ground, let the chassis settle, then `... resume "$SESSION"`
   and keep the head centred and still. Return to the captured start, park the
   chassis and head, and inspect coverage/loop closure and scan/wall overlap.
   Save **before stopping mapping**:
   `ros2 run thornbots_pkg operator_workflow save "$SESSION"`.
4. Register the boot chassis heading in radians. Team comes from
   the captured referee message. **Red**, facing +x:
   `ros2 run thornbots_pkg operator_workflow align "$SESSION" --yaw 0`.
   **Blue**, facing -x (use `oct31-blue-01` as the session in step 1):
   `ros2 run thornbots_pkg operator_workflow align "$SESSION" --yaw 3.141592653589793`.
   x/y default to the team's contract start. Optional surveyed `--x`/`--y`
   refinements must stay within 0.3 m of captured raw odometry. Never enter degrees.
5. Stop mapping with Ctrl-C. Park at the live team's contract start. Run:
   `ros2 run thornbots_pkg operator_workflow localize "$SESSION"`.
   Wait for **LOCALIZE PASS**, check RViz scan/wall overlap, and leave it running.
   **FAIL:** stay parked, read the observed/expected pose and fix the cause;
   do not enable autonomous operation on a failed or visually wrong map.

### Preparation and rationale

`mapping` and `localize` also start `foxglove_bridge` on port 8765. Connect
Foxglove to `ws://<robot IP>:8765` (use an SSH tunnel if the port is unreachable).
Only `connectionGraph` and `assets` capabilities are enabled: clients cannot
publish, call services or access parameters. The workflow stops its bridge
with the robot launch, including on failure; keep the workflow terminal open.
The robot image already supplies `ros-jazzy-foxglove-bridge`.

Rebuild/source the overlay before use (`colcon build --symlink-install
--packages-select sentry_localization thornbots_pkg`, then `source install/setup.bash`).
These commands run in the robot container. For the existing robot service see
[full robot pipeline](../realsense-yolov8-nitros-bridge/README.md#full-robot-pipeline).
Use the same ROS domain/middleware as the robot. The bounded existing-stack
check cannot detect nodes hidden in another domain.

One new session holds `session.json`, `boot.json`, `snapshot/`, `deployment/`
and independent `backup/` autosaves. Keep the whole directory. Autosaves run
while mapping; an unsaved Ctrl-C/SIGTERM prints a prominent warning and the
backup path. Backups are recovery material, not a validated deployment bundle.
The same warning prints if the launch exits on its own.
An explicit save validates fresh occupancy data and serialize service success,
then atomically installs PGM/YAML plus nonempty `.posegraph/.data` and boot data.
The manual operator workflow requires the live mapping session ID and a fresh `map->root` within
0.15 m/0.15 rad of captured boot. This is SLAM's pose, not raw odometry.
The controller-driven [MMM workflow](#mcb-mapping-machine) uses its own readiness gate.
Return to the boot mark and heading because boot anchors map registration and
the AMCL start; saving away from it breaks that provenance. Keep parked
throughout saving. Occupancy and graph are sequential snapshots, not one
transactional SLAM snapshot.

No operator odometry seed is accepted. Mapping and localize default to raw wheel
passthrough in `nav_odom`; it needs no seed. On the level-station sim map, raw
input passed the 21 m AMCL drive while rf2o/EKF failed it, and raw input keeps
rf2o's lidar corrections out of SLAM's own scan matching. `--rf2o` opts in to
rf2o/EKF. Sim wheels are exact, so wheel slip on the robot is untested.
Field startup first runs the raw-input
stack with only UART input and pose translation (no relay or command producers), verifies referee team and raw
start, and stops that stack before launching the selected backend. Field
rf2o/EKF startup independently waits up to 15 wall seconds for a nonzero referee
assignment and a subsequent fresh `/odom` at that team's start. Explicit
`--no-referee` bypasses only the referee gate: EKF still checks fresh raw x/y
against the explicit team's start after restart. `--raw-odom` is retained as an
explicit alias for the wheel default. Room rehearsal uses `mapping "$SESSION" --room`,
then the same boot/save steps, `prepare-room "$SESSION"`, and `localize
"$SESSION"`. Room mode keeps boot coordinates and raw passthrough. It makes no
field registration claim. Map the whole field with the
[broad-field coverage](../sentry_localization/README.md#broad-field-coverage) procedure; a central loop alone
leaves much of the field unseen.

Room sessions record `frame_mode: room` and `team: null`; boot/save and
`prepare-room` need no referee (including `robot_id: 0`). On a practice field
without a referee, use `mapping "$SESSION" --no-referee --team red` (or blue)
and the same flags on `localize`. This explicit choice still requires raw
`/odom` within 0.3 m of that team's contract start. It cannot repair a wrong-side
firmware seed. Boot/save inherit the choice from `session.json`.

Boot capture requires 20 fixed-geometry fresh lidar scans and at least two
stationary seconds of stamped root/chassis data (3 cm/0.03 rad motion limit).
It records raw `/odom`, referee team and the live mapping session ID. Field
alignment derives team from that record; optional `--team` must agree. Raw
odometry and measured field x must have the team's sign: red negative, blue
positive. Firmware already adds the team heading to IMU yaw, so measured chassis
heading minus captured chassis yaw must be within 0.25 rad of zero. A 180-degree
team/heading mistake therefore fails even on the symmetric field. `--yaw`
accepts radians only in [-pi, pi]; negative scientific notation is accepted.
These checks cannot replace checking the referee configuration and surveyed
heading against the physical field.

A field-aligned map can be used by either team. AMCL starts from the live team's
contract root (-4.625, 0, 0) or (4.625, 0, 0), checked against live raw odometry;
the map's captured team describes registration provenance. AMCL supplies
`map->nav_odom`, not `/localization/odom`: with rf2o that topic comes from EKF
fusing raw velocity/fixed heading and rf2o position; room/`--raw-odom` uses
passthrough raw odometry. The relay consumes this navigation odometry, so an
incorrect EKF seed cannot be repaired by a correct AMCL map pose alone.
Automatic `localize` and standalone `convergence` both require samples no older
than 3 seconds, alongside the existing covariance, scan, TF and parked-pose gates.

The rigid registration maps captured boot root to the surveyed physical start.
It resamples cell centres into a yaw-zero occupancy raster; editing YAML yaw
alone is incorrect for AMCL. Thin surfaces can change by a cell; inspect them.
The graph stays untouched in its original boot frame. Do not load it as a
transformed field graph. Registration adjusts map localization only; firmware
routes, raw CV aim, odometry constants and movement are unchanged.

`localize` runs artifact/registration `check`, starts AMCL, then automatically
runs parked `convergence`; one final LOCALIZE PASS/FAIL reports the result.
Field convergence additionally compares current referee team/raw odometry with
the boot record. It requests stationary AMCL updates and requires five fresh
pose stamps over five seconds, xy std <=0.25 m, yaw std <=0.15 rad, distance from
registered start <=0.4 m and yaw difference <=0.25 rad, plus fresh scans and
stamped map/root/lidar TF. A timeout prints observed and expected pose and says
when the robot is away from its start. This is filter convergence, not an
independent proof of field accuracy. RViz wall/scan inspection remains required.

For a longer parked check in another terminal (keep the real head centred):
`ros2 run thornbots_pkg operator_workflow convergence "$SESSION" --hold 30 --samples 20 --max-distance 0.15 --max-std 0.15`.
`check "$SESSION"` and `preflight` are also available separately. Each successful
step prints its next command. For simulation add `--sim-time` to mapping,
boot-pose, save, localize and convergence, and `real_hardware:=false` to
mapping/localize. Timeouts use wall time even when simulation pauses.

The Oct 18 hardware lidar had variable beam count/angular increments; preflight
rejects that stream. Hardware scan consistency remains unvalidated. On a failed
save or doubtful boot provenance, retain the session and recapture/remap in a
new session. On failed localization keep parked and stop the stack before using
the existing odometry-only fallback (`auto.launch.py localization_mode:=none
use_rf2o:=false`), which makes no map-localization claim.

Offline workflow regression checks: `python3 -m pytest test/test_operator_workflow.py`
from this package. Package suites run through `colcon test
--packages-select thornbots_pkg sentry_localization`. ROS rehearsal additionally exercises
clock ordering, TF, serialization, seed gating and parked scan updates.

## Parked room health check

In a sourced robot-container terminal, start the normal stack, park the chassis
**and head**, then run this passive observer (30 seconds plus 3 seconds discovery):

```bash
ros2 run thornbots_pkg room_check
```

It subscribes only: no motion commands, services, parameter writes, PINGs or MCB
traffic. Leave the normal launch running. Each run creates a unique UTC-dated
`room-evidence/` subdirectory containing `summary.txt`, per-message receipts,
stamps and positions in `observations.csv`, per-stream `metrics.csv`, observed
`rosout.csv`, graph snapshots
and parameter/environment settings. Keep that directory alongside the normal
stack/bag logs. It records metadata, not images or a replayable rosbag. Exit codes
are 0 for passing observed checks, 1 for failed checks, 2 for a collection error.
**SKIP is unverified**, including optional components absent from the graph.
POSE, head TF, raw odometry and scans are always required; absence fails clearly.
A publisher that disappears after discovery fails its delivery/coverage check.

For a longer stationary drift sample ([T39](../docs/ROADMAP.md#short-todos)):

```bash
ros2 run thornbots_pkg room_check --ros-args -p duration_s:=60.0 -p output_dir:=/workspaces/isaac_ros-dev/evidence
```

Default screening limits are POSE/head TF 90 Hz ±20%, maximum 100 ms gap;
REF_SYS 10 Hz ±30%, maximum 350 ms gap; scan delivery 10 Hz ±50%, maximum
500 ms gap; 720 fixed bins and exactly constant angular bounds/increment (matching rf2o). Rates use
counts over the entire observation window, including leading/trailing silence.
Gaps include both boundaries; these are observer receipt gaps, not proof of wire
loss. CSV stamps allow source-gap inspection. REF_SYS absence is SKIP without
`require_referee:=true`; unassigned `robot_id=0` is explicitly reported.
A present but stalled referee stream fails even without that flag.

`rf2o accepts scans` requires output for at least 80% of observed scans and no
500 ms output gap. It also reports `/scan_odom/quality` failed/no-increment
counts when available. Geometry bad/changed counts are exact for **received**
scans; rf2o skip/drop and SLAM/AMCL drop **log reports** are lower bounds because
nodes can throttle warnings and DDS can lose observations. Zero reports does
not prove zero internal drops. An absent rf2o/navigation/map backend is SKIP;
`require_rf2o:=true` and `require_localization:=true` make these required.
This accommodates the [room/field workflow](#field-day-and-room-workflow),
whose default raw-wheel mode intentionally does not run rf2o. The observer does
not save/register maps or request stationary AMCL updates.

Raw odometry must stay within 3 cm/0.03 rad of the first sample; navigation and
TF `map->root` within 15 cm/0.03 rad, over at least 90% of the chosen interval.
Maximum excursion (so a return to the start cannot hide drift), accumulated XY
path and coverage span are printed. This tests parked repeatability, not surveyed
accuracy. Stamps must be nonzero, monotonic, at most 500 ms old or 50 ms future;
`map->nav_odom` alone allows 1.1 s future for the backend transform tolerance.
Wall/ROS clock jumps, stopped sim time, stale/future stamps and interrupted
collection fail. These are screening defaults, not hardware acceptance claims.
Parameters `raw_drift_m`, `localization_drift_m`, `drift_rad`, `max_stamp_age_s`,
`max_future_s`, `pose_hz` and `scan_bins` can tighten or adapt the test; their
values are retained in the evidence. Use `use_sim_time:=true` in sim: rates/gaps
then use ROS time, with wall gaps also shown; collection duration stays wall time.
Sim startup allows up to 10 extra wall seconds for the first nonzero `/clock`;
no clock still fails clearly. The default gz pose emulator is 100 Hz, inside
the hardware screening band.

Optional camera/inference block (show a visible, tracked panel):

```bash
ros2 run thornbots_pkg room_check --ros-args -p camera:=true
```

It measures **delivered** raw camera and YOLO fps (60 Hz ±20% by default;
`camera_hz`/`yolo_hz` change expectations), not publisher-side sensor loss.
`camera_topic` defaults to `/color/image_raw` (`sensor_msgs/Image`), `yolo_topic`
to `/detections_output` (`vision_msgs/Detection2DArray`). Raw image subscription
adds transport/observer load, especially on UDP. Camera-to-TargetState latency
is a named **proxy**: state receipt minus the capture stamp of the nearest tracker
measurement receipt within 3 ms, allowing either arrival order. There is no shared
capture ID. At least 10 pairs and 80% state coverage are required, with p95 <=200 ms
(`latency_s`). State header age is never called capture latency. Empty/no target
or inadequate pairing fails this optional block instead of inventing a latency.

Optional stale-CV diagnostic, in a separate run (not an acceptance requirement):

```bash
ros2 run thornbots_pkg room_check --ros-args -p stale_cv:=true -p duration_s:=30.0
```

First show a tracked panel and let active CV decisions arrive. During collection,
**you** stop the YOLO launch in its terminal; keep tracker, aiming and relay running.
The command never kills any process. It counts actual `/cv/target` fire decisions
and checks that they cease within 200 ms of the last YOLO receipt, with at least
one second of continued consumer observation. Aim-only and idle traffic are
allowed. No firing before the stop, no producer stop or no subsequent consumer
decisions reports NOT EXERCISED. The line is INFO with PASS/FAIL in its detail
and never changes the run's verdict or exit code. This diagnostic uses observer receipt time,
not capture time or the exact process-stop time; the aiming node's 150 ms
capture-age firing gate is tested separately.
