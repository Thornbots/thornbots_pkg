# Managed target handoff validation — 2026-11-12

The implementation follows [target_manager](../README.md#target_manager) and
the [UART assignment contract](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#target-assignment-update-and-clear).
Workspace base: `fbd95763`; firmware context: MCBV3 `e7605e5`; sim base:
`b40ada7`. Deploy the package PRs together with the firmware implementation.

## Detection-stream comparison

`TrackerRosTest.SimDetectionStreamHandoffMeasurements` sends simulated panel
detections over ROS through the actual tracker factory and manager node, using
their normal subscriptions, publications and ROS timers. It uses seed 76,
5 mm Gaussian position noise, YOLO score 0.85, 40 ms capture-to-delivery delay,
60 Hz detections on a 5 ms clock grid, and camera-facing plate handoffs.
The ten-second stream parks for three seconds, translates at 0.5 m/s for three,
then spins at 6 rad/s for four. Detections then stop completely. This is a
synthetic detection fixture, not a camera/YOLO calibration or physical UART bench.

Three repeats after a clean build were identical:

| Run | Raw estimates | Assign | Update | Refresh | Clear | Model sends/s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 600 | 1 | 57 | 30 | 1 | 8.8 |
| 2 | 600 | 1 | 57 | 30 | 1 | 8.8 |
| 3 | 600 | 1 | 57 | 30 | 1 | 8.8 |

The previous per-detection handoff requests 600 model sends in this stream
(60 Hz). The manager requests 88 (8.8 Hz), an 85.3% reduction; the subsequent
clear is a separate event. Counts measure manager publications, not accepted
physical UART writes. The ROS manager→relay→bridge PTY regression separately
checks preserved model stamps, assign flags `0x03`, clear flags `0x06` with
patrol enabled, the 80-byte payload and zero clear detection age.

All repeats assign on the first tracker result: 40 ms after its contributing
capture and in the delivery callback. Clear is 190 ms after the last delivery,
230 ms after its capture. The chosen decay constant remains 0.30 s: a score of
0.85 delivered after 40 ms has confidence 0.744, above assignment, and reaches
0.40 at `0.30 * ln(0.85 / 0.40) = 226.1 ms` after capture. The observed clear
is within one 5 ms decision period. This choice balances prompt assignment and
loss detection in this fixture; recorded robot detections still need calibration.

## Validation

- Own container `isaac-agent-jetson-aim`, four CPUs, domain 168. Recreated after
  the parent removed the idle original container; rebuilt bridge, thornbots_pkg
  and sim before testing. Smoke verified package resolution to this overlay.
- thornbots_pkg: colcon reports 294 tests, zero errors/failures/skips, including
  18 confidence/manager core cases, 10 manager node cases, six tracker ROS cases
  and the manager/relay/bridge PTY case.
- Bridge: colcon reports 79 tests, zero errors/failures, 13 existing skips.
- Sim with freshly built hosted firmware: colcon reports 264 tests, zero
  errors, four reported failures and one skip. The failures are exactly the
  three pre-existing `PitchBoot/McbPitchBoot` cases `/0`–`/2`, plus their CTest
  wrapper. The remaining skip requires `T17_FIRMWARE_BINARY`. No new failure.
- The launched estimation bench passed all four stationary/45-degree, flat and
  staggered cells three times. `check_bench_log` accepted every launch log;
  no lockstep timeout occurred. The two 45-degree cells still have no accuracy
  limits; this fixture run establishes liveness and the existing scored gates.
- Host Python 3.12 repository quality checks and actionlint pass in all three
  changed package repositories; no quality baseline changed.

## Integration decisions and limits

- The design says filter reset releases the assignment. Position reacquisition
  after repeated outliers is treated as such a reset: reset confidence/count,
  expose the reset edge, and emit clear before an immediately confident same-ID
  reassignment. This uses two ordinary messages; the approved protocol adds no
  acknowledgement/retry, so loss of the clear cannot guarantee a receiver reset.
- The manager forwards state, covariance and detection stamps unchanged. Only
  the internal drift comparison predicts with MCB constant acceleration and
  rigid spin. No outgoing covariance propagation is needed or claimed.
- Manager input acknowledgments use the driving `input_stamp`, falling back to
  `detection_stamp` for truth fixtures, rather than the held model's stamp. Empty
  inputs must not inflate a count and release a later nonempty input's bench gate.
  The estimation fixture gates on these stamps; TF drops acknowledge no-model
  consumption separately. The stand-in retains its existing managed-state ack
  interface. An input ack proves manager consumption, not downstream stand-in
  consumption; the parent should reconcile downstream lockstep ordering with A.
- A's stand-in/emulator must consume managed `/cv/target_state`, honor `clear`
  before `valid`, and retain the bench tick/managed-state ack interface. No
  stand-in/emulator C++ is changed by this work.
- The existing range-squared measurement-noise model can give extremely distant
  synthetic outliers low statistical weight instead of rejecting them. The
  rejection fixture uses a lateral outlier at normal range; noise retuning is
  outside this change.

No workspace PR or gitlink integration is included. The parent owns the combined
firmware/Jetson end-to-end bench and workspace integration.
