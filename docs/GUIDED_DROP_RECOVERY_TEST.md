# PHOENIX Guided Drop Recovery Test

## Scope

This software supports two deliberately separate inert-test profiles:

1. **Neutral Drop Recorder** — the existing conservative profile. It records
   release, suspected canopy loading, stable descent, and landing while both
   servo commands remain neutral.
2. **Automatic Return Drop** — a GPS return-to-target profile that can release
   neutral only after every deployment, stability, navigation, altitude, and
   actuator gate passes.

The automatic-return flight profile is present in the firmware and host tests,
but is **disabled by default**. `GUIDED_DROP_TEST_ENABLED` and
`GUIDED_DROP_CALIBRATION_CONFIRMED` must both be true. A real physical arm input
must also be assigned to `PIN_GUIDED_DROP_ARM_SWITCH`; a compile-time assertion
prevents guidance from being enabled without it. The 20%, 40%, and validated
authority stages have independent unlocks, all false by default. Flare remains
disabled.

This is an inert recovery-module test profile. It does not authorize powered
launch testing.

## Research basis and requirement mapping

| Primary source | Relevant evidence | Software decision |
|---|---|---|
| NASA, *Further Development and Flight Test of an Autonomous Precision Landing System Using a Parafoil* | The demonstrated system combines GPS navigation, onboard flight control, and recorded flight-test data. | Use GPS course over ground and target bearing; keep synchronized onboard records; validate progressively. |
| NASA, *Method for Extracting Aerodynamic Coefficients and Winds On-the-Fly Using Real-Time GPS Data* | Real parafoil response, asymmetry, wind, and system delay must be estimated from measured flight maneuvers rather than assumed. | Calibration-lock every authority stage and only unlock 20%, then 40%, then the validated measured limit after reviewing recorded evidence. |
| U.S. Air Force, *Recovery Systems Design Guide* | Recovery performance depends on deployment dynamics, inflation, loads, and test verification. | Hold both brakes neutral through release and a dedicated deployment wait; treat sensor canopy signatures as observations, not proof. |
| NASA GSFC-STD-1000 | Flight requirements need traceable verification and end-to-end test scenarios. | Put gates in a hardware-free controller and replay the same controller in host tests before hardware use. |
| NASA X-38 guided parafoil flight-test reports | Instrumented drop testing is used to validate models and guidance assumptions. | Retain a neutral baseline profile and progress to guided inert drops only after repeatable neutral tests. |

Source links:

- <https://ntrs.nasa.gov/citations/19940029489>
- <https://ntrs.nasa.gov/citations/20200011546>
- <https://www.govinfo.gov/app/details/GOVPUB-D301-PURL-LPS103309>
- <https://standards.nasa.gov/sites/default/files/standards/GSFC/H/0/GSFC-STD-1000RevH_Approved.pdf>
- <https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070026249.pdf>

## Automatic servo extension/retraction test

The ground dashboard now includes **Automatic 20% Extend + Retract Test** in
the Bench Test card. This is the first physical test to run.

Requirements:

- inert payload on a secure bench;
- brake lines disconnected and servo horns/linkage clear;
- payload and ground LoRa link live;
- servo controller healthy;
- no launch, arming, or drop recording;
- normal servo power rail and common ground.

The button requires two taps within five seconds. After the payload accepts the
command, it owns the sequence locally:

1. Servo 1 commands 20% for 700 ms.
2. Servo 1 commands neutral for 900 ms.
3. Servo 2 commands 20% for 700 ms.
4. Servo 2 commands neutral for 900 ms.
5. The controller reports `COMPLETE` with both commands neutral.

The sequence uses the same NVS-restored calibration, calibrated pulse limits,
slew limiting, PWM pins, and `setBenchServoCommands()` actuator path as the
working manual Servo 1/Servo 2 buttons. It does not require GPS, IMU, or
barometer health. A Neutral command or a failed preflight interlock stops the
sequence at neutral.

This verifies automatic command and return-to-neutral behavior. It does not
measure brake-line load, prove a servo's physical position, or validate canopy
turn response because the current hardware has no servo position feedback.

## Automatic return drop sequence

When the calibration lock is eventually released, the inert guided profile is:

1. Ground captures Home only after five consecutive plausible GPS fixes, or
   sets and verifies another nonzero target.
2. An observer confirms the pre-opened canopy is inflated and untangled.
3. The physical arm input must be active and all health/logging gates must pass.
4. Ground selects an unlocked authority stage, sends **Prepare**, then sends
   **Confirm Arm** within five seconds. Changed prerequisites invalidate it.
5. Payload opens a new local log, records a configuration snapshot, and freezes
   target changes.
6. Release detection uses IMU acceleration as a fast candidate and requires
   persistent barometric downward speed plus altitude loss for confirmation.
7. Both brakes stay neutral for `GUIDED_DROP_DEPLOYMENT_NEUTRAL_MS`.
8. Both brakes remain neutral until stable descent and low angular rate persist.
9. GPS position, course, ground speed, target, barometer, IMU, and servos must
   remain valid through the navigation confirmation interval.
10. Guidance begins with proportional, rate-limited, reversal-guarded steering,
   capped by both the calibrated maximum and selected authority stage.
11. Final approach uses the smaller
   `GUIDED_DROP_FINAL_MAX_STEERING` authority.
12. A short GPS loss slews the last brake command toward neutral; a prolonged
    loss latches fault-neutral. IMU, barometer, or actuator loss is immediately
    latched fault-neutral.
13. Sustained excess rotation and repeated heading reversals reduce authority;
    persistent instability latches fault-neutral.
14. Landing, abort, timeout, reboot uncertainty, or any latched fault commands
    both brakes neutral.

Wi-Fi and LoRa are not controller inputs after arming. A ground-link loss does
not stop onboard sensing or guidance. The target cannot be changed during a
recording.

After a successful landing has closed the log, the payload can start its own
recovery Wi-Fi page at `/recovery`. `/api/logs` returns stored log names, sizes,
and CRC32 checksums; `/api/log/download?name=flight_N.log` downloads an exact
binary log. USB remains the fallback. Run `tools/export_drop_log.py` on a
downloaded version-3 log to produce CSV telemetry, event/configuration JSON,
replay JSON, integrity hashes, a summary, and plots.

## Calibration unlock checklist

Do not set `GUIDED_DROP_CALIBRATION_CONFIRMED=true` until all items have recorded
evidence:

- both servo neutrals and safe pulse limits measured;
- physical arm switch installed and verified fail-open;
- positive Servo 1/Servo 2 direction confirmed against brake-line geometry;
- automatic extension/retraction bench sequence repeated without reset,
  brownout, twitch, binding, or incomplete neutral return;
- unloaded and restrained linkage tests completed;
- neutral-drop deployment is repeatable;
- deployment-neutral delay measured with synchronized video and telemetry;
- minimum useful brake, conservative turn brake, response delay, turn rate,
  sink rate, glide ratio, and left/right asymmetry measured;
- GPS dropout, GPS jump, barometer bias, sensor reset, asymmetric brake, and
  stalled-servo replays pass;
- adult range-safety review and site authorization complete.

Every motion-dependent constant in `firmware/src/config.h` remains marked
`REQUIRES EXPERIMENTAL CALIBRATION`.

## Verification performed in software

The native test suite covers:

- automatic Servo 1 and Servo 2 extension/retraction order;
- neutral return between commands and at completion;
- start and in-motion interlocks;
- disabled guided profile stays neutral;
- fail-closed canopy, physical-arm, readiness, staged-authority, and two-command
  arming gates;
- release and deployment-neutral gates;
- stabilization, target, GPS course, and altitude gates;
- limited nonzero steering only after all gates;
- gradual neutral unwind on short GPS loss and latched neutral on timeout;
- GPS position-jump rejection and multi-fix recovery;
- sustained-instability latching and heading-oscillation damping;
- immediate latched neutral on critical sensor/actuator failure;
- neutral landing and abort behavior.

The current host suite contains 131 passing controller/integration cases, and
both payload and ground firmware compile successfully. This is software
verification only, not authorization for an aerial test.

Passing host tests and successful firmware compilation are not evidence that a
real parafoil will turn, remain stable, or reach the target. Those claims need
measured inert-drop data.
