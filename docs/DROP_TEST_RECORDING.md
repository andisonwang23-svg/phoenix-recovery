# PHOENIX Drop Test Recording Mode

This mode lets the ground LoRa dashboard arm an inert drop-test recording before release while keeping the payload as the timing authority.

## What Flashing Now Includes

- The ground station dashboard at `http://192.168.8.1/` has a **Drop Test Recording** panel.
- **Arm Drop Test** sends a LoRa `ARM_DROP_TEST` request to the payload.
- The payload accepts the arm request only when IMU, barometer, and servos are healthy, no failsafe is active, and any reported battery voltage is acceptable. GNSS is recorded as available or unavailable; it is not required for arming.
- When accepted, the payload starts a fresh flash log, assigns a drop-test ID, records the arm time, and locks the servos neutral.
- Release and landing markers are derived from onboard payload sensor time, not ground receive time.
- Release must persist through a candidate state; brief barometer spikes are rejected and logged.
- The payload records a **suspected** canopy inflation/load signature and a separate stable-descent observation. Neither marker proves that the canopy is fully open; correlate them with video and inspection.
- Landing requires persistent low barometric vertical speed, low IMU motion, a small altitude span, and low GPS ground speed when a valid GPS fix exists.
- **Abort And Neutral** sends `ABORT_DROP_TEST`, commands neutral, marks the test aborted, and closes the partial log when recording is active.
- Telemetry reports drop-test state, test ID, arm time, release confirmation time, landing confirmation time, recording status, and neutral-lock status.
- Telemetry also reports the suspected-canopy and stable-descent timestamps. The local log contains the higher-rate synchronized sensor record.
- During active drop recording, payload telemetry is reduced to a low-rate preview. The raw test record is the onboard flash log, so LoRa delay or dashboard disconnects do not change the measured event timing.
- The ground station blocks manual steering, target updates, and bench servo commands while drop recording is active to avoid extra LoRa traffic. Abort and neutral commands remain available.

## Operator Flow

1. Power the payload and ground station.
2. Connect to `PHOENIX-GROUND`.
3. Open `http://192.168.8.1/`.
4. Confirm payload link, IMU, barometer, and servo health.
5. Press **Arm Drop Test** before moving to the release point.
6. Wait for telemetry to show `ARMED_RECORDING` and a nonzero test ID.
7. Release the inert article without further dashboard interaction.
8. After recovery, inspect the drop-test state and saved payload log.

## Telemetry Delay Mitigation

- The payload is the timing authority. Dashboard event times are payload milliseconds, not ground receive time.
- LoRa is treated as preview/supervision during the drop test, not as the primary data recorder.
- Active drop-test preview telemetry is throttled to `DROP_TEST_TELEMETRY_RATE_HZ`; idle drop-test status is throttled to `DROP_TEST_IDLE_TELEMETRY_RATE_HZ`.
- Commands that can create avoidable radio traffic are rejected while recording is active.

## Safety Boundary

Drop-test mode is for inert testing. While recording is active, the dedicated
drop-test controller unconditionally keeps both servos neutral so LoRa delay or
a configuration mistake cannot steer the article or affect the outcome.

This is not yet a complete reliable log-download protocol. `REQUEST_LOG_INDEX` is now defined and visible in the dashboard as a command path, but chunked download and whole-file checksum verification still need to be implemented before post-test download is considered complete.

## Research basis

See `docs/DROP_TEST_RESEARCH_TRACEABILITY.md` for the primary NASA papers,
source-to-requirement evidence, implementation mapping, calibration limits, and
progressive validation gates.

Normal launch recovery remains separate and automatic. See
`docs/LAUNCH_AND_BENCH_RECOVERY_MODE.md` for the launch sequence and the guarded
20%/40% sensor-independent servo-test procedure.
