# thornbots_pkg

Follow [workspace rules](../AGENTS.md) and [CI](../docs/CI.md).
Read [node graph](README.md#nodes), [CV boundary](README.md#cv-interface) and
[design rationale](README.md#notes) before changing behavior.
`auto.launch.py` includes localization; do not launch it separately.
CV selection/tracking/aiming comes first; do not front-run firing work unless asked.

## Scope

Own robot description, pose translation, target selection/tracking and aiming.
Only `mcb_relay` publishes outgoing aim/relocalization to the UART bridge;
`thornbots_nav`'s `waypoint_nav` publishes NAV_GOAL.
Localization belongs to `sentry_localization`, worlds to `sim`.
Keep runtime nodes C++; Python is for launch, with ROS-free cores unit-tested.

## Open

Software commitments: [ROADMAP](../docs/ROADMAP.md#additional-mini-work).
Physical validation: [ROBOT_TESTS](../docs/ROBOT_TESTS.md#consolidated-hardware-checks).
