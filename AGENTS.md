# thornbots_pkg: agent notes

Hardware interface, robot description, and CV target selection for the Thornbots
Sentry. **Reference docs live in `README.md`** (topics, nodes, launch args,
`## Notes` design rationale). Read it before changing behavior.

`auto.launch.py` is the single entry point; it already includes
`sentry_localization`'s launch. Don't launch that package separately.

**This package is shadowed by `/workspaces/ros2_ws`** (`Dockerfile.thornbots`
LAYER `RECLONE_SENTRY` clones it from GitHub). Once it's built locally, an edit
under `src/thornbots_pkg` is live under `dexec.sh` but _not_ in the user's
terminal, which resolves to the image-baked clone. Before trusting any result:
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

- **`auto.launch.py` should bring up the CV stack too, and doesn't yet.**
  Decided 2026-07-27: this package owns launching the whole stack, since it
  already owns pose/TF ownership and the `real_hardware`/`localization_mode`
  sim-vs-real toggle. It needs a new arg that starts either `sim`'s
  `spawn_target`/`target_driver`/`cv_target_emulator` nodes (sim path) or the
  `realsense-yolov8-nitros-bridge` chain (hardware path), mirroring how
  `real_hardware` already switches `pose_emulator` against the real Type-C
  driver. Until it lands, `sim.launch.py spawn_target:=true` plus a hand-run
  `point_to_cv_target` works standalone.
- **Real firing logic is not built** — no HP/heat/power gating, no timing.
  `point_to_cv_target` has a placeholder fire trigger (`fire_rate_hz`,
  defaulting to on) gated only on `target_active` and cached confidence; it
  never consults the aim solve, so it can fire when the TF lookup has failed. An
  aim/lead controller does exist (`target_tracker.py` plus
  `point_to_cv_target`'s `lead_enabled` intercept solve). Out of scope until CV
  is done, per the priority above.
- **The Referee System UART/data-interface spec has not been sourced.** Needed
  before real firing-timing work can start; see
  `../ARCC_2026_SENTRY_CONTEXT.md`.
