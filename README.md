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
- Its own `robot_state_publisher`, off `urdf/sentry.urdf.xacro`.
- Includes `sentry_localization`'s launch file to bring up whichever
  localization backend `localization_mode` selects.

## Node/topic pipeline

```
/pose --[pose_translator]--> /odom --> sentry_localization --> /localization/odom --[odom_tf_broadcaster]--> odom->root TF
                          \-> /joint_states --[robot_state_publisher]--> rest of TF tree
/scan ------------------------------> sentry_localization (map->odom TF owned directly by slam_toolbox/amcl there)
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

## Testing

The localization drift/jerk-correction integration suite now lives in
`sentry_localization/test/slam_integration/run_localization_drift_tests.py`
(it launches this package's `auto.launch.py`, which includes
`sentry_localization`) — see `sentry_localization/README.md`'s Testing
section.

Standard `colcon test`-style checks (`ament_copyright`/`ament_flake8`/
`ament_pep257`) apply via the normal `colcon test --packages-select
sentry_pkg`.

## Cleaning up

Always fully stop a launch tree (`isaac_ros_common/scripts/kill_launch.sh
<pid>`, not a bare `pkill`) before relaunching — a half-killed tree leaves
duplicate nodes publishing TF, which causes jitter in the next run.
