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
- **`auto.launch.py` should bring up the CV stack too, and doesn't yet.**
  Decided 2026-07-27: this package owns launching the whole stack, since it
  already owns pose/TF ownership and the `real_hardware`/`localization_mode`
  sim-vs-real toggle. It needs a new arg that starts either `sim`'s
  `spawn_target`/`target_driver`/`cv_target_emulator` nodes (sim path) or the
  `realsense-yolov8-nitros-bridge` chain (hardware path), mirroring how
  `real_hardware` already switches `pose_emulator` against the real Type-C
  driver. Until it lands, `sim.launch.py spawn_target:=true` plus a hand-run
  `point_to_cv_target` works standalone.
- **Firing logic is partial.** `point_to_cv_target` aims and fires per
  publish tick, at most `fire_rate_hz`, and times shots against a spinning
  target with `CVTarget.delay_ms`. No HP/heat/power gating, and the MCB
  firmware's `CVData` struct hasn't grown the merged fire fields yet, so the
  timing only works in sim.
- **The Referee System UART/data-interface spec has not been sourced.** Needed
  before real firing-timing work can start; see
  `../ARCC_2026_SENTRY_CONTEXT.md`.
- **Part 1 (`point_to_cv_target`) passes `sim`'s gz-free aim bench 10/10**
  (2026-09-24, chase mode: 94-98% of shots hit at every tick, 99% stationary).
  The old misses split between the halves:
  - Part 1's, now fixed: aiming at panel 0 not the facing one, one height,
    no acceleration, and the lead taken from the firmware latency, not the
    gimbal's lag.
  - Part 2's, still open: the 2-4 cm sideways offset is gone on the
    perfect model, so it comes from the tracker or gz geometry.
- **`ArmorEKF` gained acceleration and a single-panel yaw measurement**
  (2026-09-25), state `[pos, vel, acc, yaw, w, r, dz]` by named slices.
  Unit-tested and checked on `sim/tools/estimation_offline.py`, not yet on
  gz C2. Where it stopped and what's next: `../CV_SPLIT_PLAN.md` "Where
  this stopped".
  - A still hypothesis (2026-09-25) gives a parked, non-spinning target
    exactly zero velocity and spin; README.md has the numbers. Offline
    only; gz C2 hasn't run it. Drive-off costs ~0.23 s at up to 9 cm.
  - gz's shots land 1.6 cm low on every case, likely the chassis sagging
    on its placeholder springs while TF keeps `root` at z = 0. Not traced.
- **Part 1 now aims for our own motion** (2026-09-25): the shot leaves
  where we are at the aim horizon and carries our velocity. On the aim
  bench at `shooter_speed:=1.0`, 95-99%, at most 1.5 points under a still
  shooter.
- **Chase mode is the default** (`chase_settle_s` 0, since 2026-09-25). It needs the
  gimbal to jump ~7 deg every quarter turn and settle; measure that on
  hardware and set `chase_settle_s` to the settle time.
- **Jazzy: drop `setup.py`'s `tests_require`.** The CV tests are part of
  the Jazzy move's done-when bar. `../JAZZY_PLAN.md`.

## Committing

This package is a submodule of `thornbots_workspace`, on branch `main`. Commit
and push here first, then bump this gitlink in `../` — one logical change, one
bump, never a gitlink pointing at an unpushed commit. Full rule in
`../CLAUDE.md` § Packages.
