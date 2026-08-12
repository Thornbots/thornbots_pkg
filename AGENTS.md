# sentry_pkg — agent notes

Hardware interface, robot description, and CV target selection for the
Thornbots Sentry. **Reference docs live in `README.md`** (topics, nodes,
launch args, `## Notes` design rationale) — read it before changing
behavior. This file is only the operating contract for working here.

Parent conventions in `../CLAUDE.md` apply, notably: **in-code
comments/docstrings under 10 lines**; longer prose goes to a `## Notes`
subheading in `README.md`.

## Running anything

Never hand-roll `docker exec`. Use `../isaac_ros_common/scripts/dexec.sh`,
which is the only path with correct env parity (ROS_DOMAIN_ID, FastDDS
profile, both workspace installs, `-u admin` for GUI). Load the
`isaac-ros-docker` skill before your first container command.

```bash
# all paths below are relative to this package dir
../isaac_ros_common/scripts/dexec.sh -- colcon build --symlink-install --packages-select sentry_pkg
../isaac_ros_common/scripts/dexec.sh -- colcon test  --packages-select sentry_pkg
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch sentry_pkg auto.launch.py real_hardware:=false
```

`auto.launch.py` is the single entry point — it already includes
`sentry_localization`'s launch. Don't launch that package separately.

**This package is shadowed by `/workspaces/ros2_ws`** (`Dockerfile.thornbots`
LAYER `RECLONE_SENTRY` clones it from GitHub). Once it's built locally, an
edit under `src/sentry_pkg` is live under `dexec.sh` but *not* in the user's
terminal, which resolves to the image-baked clone. Before trusting any
result: `../isaac_ros_common/scripts/dexec.sh -- ros2 pkg prefix sentry_pkg`
(`/workspaces/isaac_ros-dev/…` = your edit is live).

Stop launch trees with `../isaac_ros_common/scripts/kill_launch.sh <pid>`, never `pkill` — a
half-killed tree leaves duplicate TF publishers that corrupt the next run.

## Scope

- Owns `/pose` consumption, `odom->root` republish, the URDF, and the
  `mcb_relay` boundary to `dji_serial_bridge` — no other node may touch
  that bridge's topics.
- Localization backends (SLAM/AMCL/EKF) belong to `sentry_localization`;
  gz-sim worlds belong to `sim`. Change those there, not here.
- Its own git repo (`Thornbots/sentry_pkg`) — commits here are separate
  from the workspace.

## Current priority

CV first (`target_selector`, `target_tracker`, `point_to_cv_target`).
Don't front-run firing logic unless asked. See `../SESSION_NOTES.md`.
