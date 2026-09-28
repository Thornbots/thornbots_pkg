# thornbots_pkg: agent notes

Hardware interface, robot description, and CV target selection for the Thornbots
Sentry. **Reference docs live in `README.md`** (topics, nodes, launch args,
`## Notes` design rationale). Read it before changing behavior.

`README.md` commands are written for a human in a container terminal. You run
them from the host through `../isaac_ros_common/scripts/dexec.sh` (load the
`isaac-ros-docker` skill first), which sources both workspaces for you:

```bash
../isaac_ros_common/scripts/dexec.sh -- colcon build --symlink-install \
  --packages-select thornbots_pkg sentry_localization
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch thornbots_pkg auto.launch.py real_hardware:=false
../isaac_ros_common/scripts/dexec.sh -d -- rviz2 -d install/sim/share/sim/rviz/config.rviz
```

`auto.launch.py` is the single entry point; it already includes
`sentry_localization`'s launch. Don't launch that package separately.

**This package is shadowed by `/workspaces/ros2_ws`** (`Dockerfile.thornbots`
LAYER 5 copies this directory in at build time). Once it's built locally, an
edit under `src/thornbots_pkg` is live under `dexec.sh` but _not_ in the user's
terminal, which resolves to the image-baked snapshot. Before trusting any result:
`../isaac_ros_common/scripts/dexec.sh -- ros2 pkg prefix thornbots_pkg`
(`/workspaces/isaac_ros-dev/…` = your edit is live).

Stop launch trees with `../isaac_ros_common/scripts/kill_launch.sh <pid>`, never
`pkill`: a half-killed tree leaves duplicate TF publishers that corrupt the next
run.

## Scope

- Owns `/pose` consumption, `odom->root` republish, the URDF, and the
  `mcb_relay` boundary to `dji_serial_bridge`. No other node may touch that
  bridge's topics.
- Localization backends (SLAM/AMCL/EKF) belong to `sentry_localization`; gz-sim
  worlds belong to `sim`. Change those there, not here.

## Current priority

CV first (`target_selector`, `target_tracker`, `point_to_cv_target`). Don't
front-run firing logic unless asked.

## Open

- **The URDF is now `sentry_v2`'s frames and meshes** (`meshes/sentry_v2/`),
  with a `muzzle` frame on `head_pitch`. Wheel and suspension joints are
  fixed, since `/joint_states` only carries `headlink` and `headpitch`.
- **Measure the real lidar's blind sector.** `lidar_self_filter`'s 0.09-1.41
  rad comes from the CAD (README.md), and nobody has measured where the real
  RPLIDAR's 0 deg points relative to the gun. Capture `/scan_raw` on the
  robot, find its 0 deg direction and the head's real shadow, and fix the
  `lidar` frame's yaw or the sector to match.
- **`auto.launch.py` starts `target_selector`, `target_tracker` and
  `point_to_cv_target`, but not what feeds them.** Decided 2026-07-27: this
  package owns launching the whole stack. It still needs an arg that starts
  either `sim`'s `target_driver`/`cv_target_emulator` (sim path) or the
  `realsense-yolov8-nitros-bridge` chain plus `roi_depth_node` (hardware
  path), the way `real_hardware` switches `pose_emulator` against the real
  Type-C driver.
- **Firing logic is partial.** `point_to_cv_target` aims and fires per
  publish tick, at most `fire_rate_hz`, and times shots against a spinning
  target with `CVTarget.delay_ms`. No HP/heat/power gating, and the MCB
  firmware's `CVData` struct hasn't grown the merged fire fields yet, so the
  timing only works in sim.
- **`CVTarget` is an `odom` point since 2026-09-27**, and the firmware
  still reads it as `root`. Which `odom` the MCB can share is open:
  `../CV_SPLIT_PLAN.md` W.3's open issues.
- **`mcb_relay`'s relocalize latencies are placeholders.** Measure the
  UART legs (USB-serial latency timer included) and the MCB's RX poll on
  the robot, then set `uart_latency_s`, `mcb_read_delay_s` and
  `latency_std_s` (README.md `### mcb_relay.py`). Only unit-tested: sim
  runs no bridge, so nothing consumes `~/relocalize` there.
- **The Referee System UART/data-interface spec has not been sourced.** Needed
  before real firing-timing work can start; see
  `../ARCC_2026_SENTRY_CONTEXT.md`.
- **Part 1 (`point_to_cv_target`) is done on `sim`'s aiming bench**:
  per-cell floors for still and moving shooters, 95-99% hit in chase mode.
  It aims for our own motion. The 2-4 cm sideways offset seen with the
  tracker in the loop is gone on the perfect model, so it is Part 2's.
- **`ArmorEKF`** state is `[pos, vel, acc, yaw, w, r, dz]` by named slices,
  with a single-panel yaw measurement and a still hypothesis for a parked,
  non-spinning target. Unit-tested. On the estimation bench moving cells read 0.08-0.17 m
  facing p95 medians (2026-09-26); what's next is in `../CV_SPLIT_PLAN.md`
  "Where this stands". The estimation bench has no drive-off cell yet (a unit-test probe:
  ~0.23 s at up to 9 cm).
- **Chase mode is the default** (`chase_settle_s` 0, since 2026-09-25). It needs the
  gimbal to jump ~7 deg every quarter turn and settle; measure that on
  hardware and set `chase_settle_s` to the settle time.
- **`target_tracker` is the estimation bench's slowest node** (2026-09-25): at ~8x on
  `sim`'s `bench_world` it saturates a core, 46% of its main thread in
  `ArmorTracker.step`'s small-matrix numpy and 27% in its TF listener
  thread. A C++ core would lift it and speed up the Jetson too; the
  user's call, not started.
- **Jazzy:** the CV tests pass in the Isaac ROS 4.6
  container, and the aiming and estimation benches give Humble's results on the laptop. Nothing has
  run on hardware yet (`../JAZZY_PLAN.md` step 5).

## Committing

This package is a submodule of `thornbots_workspace`, on branch `main`. Commit
and push here first, then bump this gitlink in `../` — one logical change, one
bump, never a gitlink pointing at an unpushed commit. Full rule in
`../CLAUDE.md` § Packages.
