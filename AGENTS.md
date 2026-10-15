# thornbots_pkg

Follow [workspace rules](../AGENTS.md) and [CI](../docs/CI.md).
Read [node graph](README.md#nodes), [CV boundary](README.md#cv-interface) and
[design rationale](README.md#notes) before changing behavior.
`auto.launch.py` includes localization; do not launch it separately.
CV selection/tracking/aiming comes first; do not front-run firing work unless asked.

## Scope

Own robot description, pose translation, target selection/tracking and aiming.
Only `mcb_relay` publishes outgoing aim/relocalization to the UART bridge.
Localization belongs to `sentry_localization`, worlds to `sim`.
Keep runtime nodes C++; Python is for launch, with ROS-free cores unit-tested.

## Open

- `auto.launch.py` starts CV consumers but still needs an arg selecting their
  sim or hardware detection producers; this package owns full-stack launch.
- Validate the real lidar zero direction/blind sector; the current filter is
  CAD-derived: [lidar filter](README.md#lidar_self_filter).
- Measure relay UART/mailbox delays and gimbal settling on hardware; defaults
  are placeholders: [relay](README.md#mcb_relay), [aiming](README.md#point_to_cv_target).
- Firing receipt/indexer timing and HP/heat/power gates remain unverified;
  [robot work](../docs/ROADMAP.md#later-needs-a-robot). Source the referee UART spec
  before hardware firing-timing work: [game context](../docs/ARCC_2026_SENTRY_CONTEXT.md).
- Tracker accuracy: [track G](../docs/ROADMAP.md#g-estimation-accuracy).
  Frame design: [pose_translator](README.md#pose_translator); wire frames:
  [UART_PROTOCOL](../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types).
