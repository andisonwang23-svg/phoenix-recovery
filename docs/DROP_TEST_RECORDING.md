# PHOENIX Drop Test Recording Mode

This mode lets the ground LoRa dashboard arm an inert drop-test recording before release while keeping the payload as the timing authority.

## What Flashing Now Includes

- The ground station dashboard at `http://192.168.8.1/` has a **Drop Test Recording** panel.
- **Arm Drop Test Now** immediately sends one LoRa `ARM_DROP_TEST` request to
  the payload without a browser confirmation dialog or second arming action.
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
- The ground station automatically mirrors each new drop-test telemetry packet to a persistent CSV. It starts when a nonzero payload test ID reports recording, closes on `TEST_COMPLETE` or `TEST_ABORTED`, and requires no key press or open browser.
- The dashboard shows the automatic CSV filename and row count. **Open Saved CSV Logs** lists files stored on the ground ESP32 and downloads them through `PHOENIX-GROUND`.
- Every ground CSV row contains both the payload timestamp and ground receive timestamp, sensor validity/age, navigation values, requested servo commands/pulses, event timestamps, configuration version, available power measurements, and LoRa RSSI/SNR.
- The ground logger opens, appends, and closes the file for each packet so a dashboard disconnect does not stop recording and a sudden ground-station power loss cannot strand a long-lived file buffer.
- The ground station blocks manual steering, target updates, and bench servo commands while drop recording is active to avoid extra LoRa traffic. Abort and neutral commands remain available.

## Operator Flow

1. Power the payload and ground station.
2. Connect to `PHOENIX-GROUND`.
3. Open `http://192.168.8.1/`.
4. Confirm payload link, IMU, barometer, and servo health.
5. Press **Arm Drop Test Now** before moving to the release point. This sends
   the request immediately.
6. Wait for telemetry to show `ARMED_RECORDING` and a nonzero test ID.
7. Release the inert article without further dashboard interaction.
8. After recovery, open **Saved CSV Logs** on the ground dashboard and download the automatic LoRa copy.
9. For scientific analysis, also recover the payload's higher-rate onboard log; the ground CSV is a preview/backup and can have gaps when the LoRa link is interrupted.

## Telemetry Delay Mitigation

- The payload is the timing authority. Dashboard event times are payload milliseconds, not ground receive time.
- LoRa is treated as preview/supervision during the drop test, not as the primary data recorder.
- Active drop-test preview telemetry is throttled to `DROP_TEST_TELEMETRY_RATE_HZ`; idle drop-test status is throttled to `DROP_TEST_IDLE_TELEMETRY_RATE_HZ`.
- Commands that can create avoidable radio traffic are rejected while recording is active.

## Safety Boundary

Drop-test mode is for inert testing. While recording is active, the dedicated
drop-test controller unconditionally keeps both servos neutral so LoRa delay or
a configuration mistake cannot steer the article or affect the outcome.

The ground CSV is automatically downloadable, but it is not a replacement for a reliable transfer of the payload's raw binary log. `REQUEST_LOG_INDEX` is defined and visible in the dashboard; chunked payload-log download and whole-file checksum verification still need to be implemented before remote retrieval of the authoritative onboard log is considered complete.

## Research basis

See `docs/DROP_TEST_RESEARCH_TRACEABILITY.md` for the primary NASA papers,
source-to-requirement evidence, implementation mapping, calibration limits, and
progressive validation gates.

Normal launch recovery remains separate and automatic. See
`docs/LAUNCH_AND_BENCH_RECOVERY_MODE.md` for the launch sequence and the guarded
20%/40% sensor-independent servo-test procedure.

The calibration-locked automatic-return profile and the automatic Servo 1/Servo
2 extension-and-return-to-neutral bench sequence are documented in
`docs/GUIDED_DROP_RECOVERY_TEST.md`. When guided testing is explicitly unlocked,
**Arm Drop Test Now** requests the conservative 20% automatic-return stage. With
the current guided locks still disabled, it arms the neutral recorder only.
