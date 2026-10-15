# Per-frame hit permission verification, 2026-10-15

[PR #14](https://github.com/Thornbots/thornbots_pkg/pull/14) removes the aim
node's REF_SYS subscription and its `gun_yaw + hit_angle_sign * delta`
conversion. REF_SYS is chassis-relative; current POSE does not provide the
chassis heading needed to convert that delta into an odom bearing. Firmware
owns hit turns under the unchanged UART format.

Every outgoing frame clears bit 2 while any target estimate is held,
including unconverged and coasting/predicted estimates during occlusion.
No-estimate patrol frames set it. Reacquisition clears it on the next frame,
with no cooldown. A failed aim solve does not release permission while an
estimate remains held. With no target and patrol disabled, no point is sent.

## Unit and repository checks

All eight registered suites pass: 128 cases, zero failures/errors/skips.
Node tests cover target, coasting, unconverged state, no target, patrol,
failed solve with a held estimate, timeout, immediate reacquisition, and the
absence of the aim node's REF_SYS subscription. The old `turn_to_hit`,
`hit_angle_sign` and `hit_turn_s` parameters and patrol hit state are removed.
`ref_sys_topic` remains in the full launch for `target_selector` team data.

Host Python 3.12 repository quality and actionlint pass without baseline
growth. A read-only diff review found no merge blockers.

## Hosted drive and match

The comparison isolates #14 from the later report-latching changes:

| Component | Nightly comparison | Candidate comparison |
| --- | --- | --- |
| thornbots behavior | nightly `bd3d94f8` | #14 `2b662a6` |
| MCBV3 | angle-only #93 `881dda7` | same binary |
| sim | angle-only #21 `3bb12a6` | same runtime |

Both use the angle-only contract from bridge #12; the bridge executable has
no implementation difference from the installed translator. The common
hosted firmware SHA-256 is
`0cd492fd533f6c72cf40c177f75dc1975c5662633010681f7aa5bb9ae956f4bd`.
The validated `t31/hit-angle` branches were not modified or pushed.

ROS Jazzy runs in an owned arm64 Mac container with a separate ROS domain
and gz partition. Track A's processes are checked before each launch; its
container is neither stopped nor reused. The GUI remains enabled; Gazebo's
four-robot field and RViz robot/target displays were inspected. The firmware
uses GCC 14, gz's 1 ms physics step, default lockstep, shotgating and stage
settings. The full spawn-to-center drive and four-robot match stages run
through the real UART bridge and compiled firmware.

Three fresh runs per stage and variant alternate nightly/candidate. Each
must pass the native GTest runner and `check_bench_log` over both runner and
stack logs. Prefixes and the firmware digest are checked per run. Pool native
MCB hits/shots directly from `shots.jsonl`; match uses `enemy_hit`, excluding
ghost shots. Percentages below are pooled counts, not averages of rates.

| Stage | Nightly + angle | #14 + angle | Difference |
| --- | --- | --- | --- |
| `mcb_drive` | 84/94 (89.4%) | 91/95 (95.8%) | +6.4 pp |
| `mcb_match` | 60/111 (54.1%) | 60/111 (54.1%) | 0.0 pp |

All twelve runs passed with runner/checker status 0/0; no attempts were
excluded or repeated. No pooled stage hit-rate regression is observed.

| Stage / variant | Run 1 | Run 2 | Run 3 |
| --- | --- | --- | --- |
| Drive / nightly + angle | 27/32 | 27/31 | 30/31 |
| Drive / #14 + angle | 28/30 | 29/31 | 34/34 |
| Match / nightly + angle | 20/37 | 20/37 | 20/37 |
| Match / #14 + angle | 20/37 | 20/37 | 20/37 |

Native checks require fresh lockstep status and zero timeouts. Shutdown-only
notes include parameter bridge exits -11, -6 or 255 and RViz exits -9 or -11;
the native checker accepted them. Candidate match run 2 recorded 20 damage
to the sentry; the other five match runs recorded zero. All six matches
recorded zero friendly intersections.

These are software regression guards with synthetic detections, not a
physical armor-direction or calibrated combat-accuracy measurement.

Artifacts are retained at
`/Users/blaise/ros2_ws/t31-artifacts-2026-10-15/results/`, with directory names
`angle-{nightly,candidate}-{mcb_drive,mcb_match}-r{1,2,3}`. Each contains run,
stack and checker logs, result codes, package prefixes, firmware digest,
shots, states, route telemetry and stage summaries. Native-accepted shutdown
notes are retained rather than treated as clean individual node exits.

This comparison does not require MCBV3 #94, sim #22 or bridge #13. Those
report-latching PRs remain separate from the angle-only set plus #14.
