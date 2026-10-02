# PHOENIX Inert Drop-Test Recovery System — Research and Traceability

Revision date: 2026-10-01

## Scope and safety boundary

This is an **inert test and evidence-collection system**. During an armed drop
test the payload records locally and both servos remain neutral in every state.
It does not authorize autonomous steering, flare, pyrotechnics, a rocket motor,
or a drop from an improvised elevated location. Physical tests require an
approved site, a qualified adult/club test lead, a controlled exclusion zone,
an inert article, and a test-specific risk review.

The software is not flight-qualified. Passing host tests proves deterministic
software behavior for the tested inputs; it does not validate real parafoil
deployment, loads, stability, or landing performance.

## Evidence-backed design pattern

NASA programs do not support choosing one universal height, inflation delay, or
angular-rate threshold for a new parafoil. The repeatable pattern in the source
material is:

1. define a test objective and configuration;
2. predict the test with simulation;
3. progress through ground and flight test beds;
4. record synchronized onboard sensor data;
5. use independent imagery and inspection;
6. reconstruct the test afterward; and
7. update the model and thresholds from measured evidence.

PHOENIX therefore treats all motion thresholds in `config.h` as **REQUIRES
EXPERIMENTAL CALIBRATION**. The papers support the architecture and measurement
method, not the current numeric values.

## Implemented test timeline

```text
IDLE
  -> ARMED_RECORDING             local flash log open; servos neutral
  -> RELEASE_CANDIDATE           descent + altitude-loss condition begins
  -> DROP_CONFIRMED              condition persists for confirmation time
  -> CANOPY_EVENT_SUSPECTED      optional deceleration/acceleration signature
  -> STABLE_DESCENT_OBSERVED     optional quiet-rate/descent window
  -> LANDING_CONFIRM             low motion and small altitude span persist
  -> POST_LANDING                keep recording after touchdown
  -> TEST_COMPLETE               final record written and log closed

Any time -> TEST_ABORTED         immediate neutral; partial log closed
Timeout  -> TEST_COMPLETE        neutral; timeout event; log closed
```

`CANOPY_EVENT_SUSPECTED` does **not** mean “canopy proven open.” A change in
vertical speed or acceleration can also come from line stretch, impact,
pendulum motion, sensor orientation, or filtering. Full opening must be
corroborated with synchronized video and post-test line/canopy inspection.

## Detection rules

All inputs must be finite and from a valid source before they participate in a
trigger.

| Marker | Required evidence | Persistence / rejection |
|---|---|---|
| Release candidate | valid barometer, downward vertical speed, altitude loss from arm point | must persist; a broken condition is logged as rejected |
| Drop confirmed | release candidate remains true | configurable confirmation time |
| Canopy signature suspected | post-release reduction from peak descent rate **or** IMU linear-acceleration transient | observational only; never enables steering |
| Stable descent observed | valid barometer and IMU, established descent, angular rate below limit, vertical-speed range below limit | continuous stable window |
| Landing candidate | valid barometer and IMU, low vertical speed, low angular rate, low GPS ground speed when GPS is valid | altitude span and all conditions must remain within limits |
| Landing confirmed | landing candidate remains true | configurable confirmation time, followed by post-landing recording |

GPS is useful corroboration outdoors but is not required to arm or detect an
indoor inert drop. When a valid GPS fix exists, excessive ground speed prevents
a false landing declaration. Loss of telemetry or ground Wi-Fi does not stop
local recording and does not alter the servo interlock.

## Recorded evidence

Each local snapshot now contains:

- payload sample timestamp and configuration version;
- each sensor's valid bit, age, and last update timestamp;
- GPS position, altitude, HDOP, satellite count, speed, and course;
- barometric pressure, temperature, altitude AGL, and vertical speed;
- IMU attitude, three gyro axes, total angular rate, and vertical acceleration;
- requested and actual servo commands and actual pulse widths;
- battery and servo-rail voltage when physical measurement hardware exists;
- drop-test ID, state, and release/canopy/stable/landing timestamps.

LoRa is a low-rate preview. The payload flash log is the primary measurement
record, because a radio link can lose samples. A camera view should include an
LED or other visible synchronization cue if quantitative video timing is
required.

## Source-to-requirement proof

These are primary NASA technical records. The “supports” column is deliberately
narrow: it states what the source supports without claiming that a NASA vehicle
and PHOENIX have the same dynamics.

| Source | Direct finding in the source | What it supports in PHOENIX |
|---|---|---|
| [Murray et al., *Further Development and Flight Test of an Autonomous Precision Landing System Using a Parafoil*, NASA-TM-4599 (1994)](https://ntrs.nasa.gov/citations/19940029489) | Describes a phased parafoil flight-test program using GPS navigation, a flight computer, compass, yaw-rate gyro, and onboard data recorder; navigation was tested while autoland was still being developed. | Phased development; onboard logging; do not treat landing/flare as validated early. |
| [Strahan, *Testing Status of the X-38 Large Parafoil Autonomous GN&C System* (2001)](https://ntrs.nasa.gov/citations/20100033368) | Maps flight objectives into ground/flight requirements and multiple test beds before the orbital article. | Bench/suspended/inert progression and objective-based gates. |
| [Machin et al., *Parachute Testing for the NASA X-38 Crew Return Vehicle*](https://ntrs.nasa.gov/api/citations/20060056201/downloads/20060056201.pdf) | Reports GPS, compass, barometric altitude, flight computer, uplink/downlink, and several progressively representative GN&C test beds. | Multiple sensors and progressive integration; radio is not the only measurement path. |
| [Madsen et al., *An Overview of the Guided Parafoil System Derived from X-38 Experience*](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070026249.pdf) | States that instrumentation on the load was used to validate simulation/load predictions and reconstruct trajectory/performance after flight. | Synchronized local logging and post-test reconstruction. |
| [Moore et al., *Simulating New Drop Test Vehicles and Test Techniques for the Orion CEV Parachute Assembly System*](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20110011397.pdf) | Describes preflight prediction and post-test reconstruction simulations evolving with a multi-year parachute test campaign. | Replay the actual detector with recorded data; calibrate from measurements. |
| [Dutta, *ASPIRE Parachute Modeling and Comparison to Post-Flight Reconstruction* (2020)](https://ntrs.nasa.gov/citations/20200002925) | Describes validating preflight dynamics simulations against reconstructed trajectories after each test and changing later predictions. | Iterative simulate-test-reconstruct-update workflow. |
| [Shafner et al., *Dragonfly Aeroshell/Parachute Dynamics through Subscale Drop Tests* (2024)](https://ntrs.nasa.gov/citations/20240009300) | Onboard instrumentation measured rotation rates used to determine attitude; tests quantified settling and oscillation amplitudes. | Record angular rate/orientation and define stability from measured motion, not appearance alone. |
| [Hoffman, *Evaluation of a Parachute Load Distribution Measuring System During Low Altitude Drop Tests*, NASA-TM-X-1832 (1969)](https://ntrs.nasa.gov/citations/19690023304) | Correlated motion-picture frames with instrument records; the report also documents telemetry loss during instrumented drops. | Independent video correlation and payload-local data retention rather than reliance on LoRa. |
| [Brown et al., *Determination of Barometric Altimeter Errors for Orion EFT-1* (2012)](https://ntrs.nasa.gov/citations/20120012825) | Analyzes pressure-to-altitude error sources and reports aerodynamic effects as the largest single contributor in that vehicle's error budget. | Reject single-sample state changes; treat barometric thresholds as vehicle-specific calibration values. |
| [Barth et al., *Post-Flight Analysis of GN&C Performance During Orion EFT-1* (2015)](https://ntrs.nasa.gov/citations/20150001919) | Describes redundant IMU/GPS/barometric sensing, preflight 6-DOF analysis with sensor/effector failures, and postflight reconstruction. | Fault injection, synchronized multi-sensor evidence, and reconstruction before claiming readiness. |

## Requirement-to-code traceability

| Requirement | Implementation | Verification |
|---|---|---|
| Test is payload-owned and automatic after arming | `logic/drop_test_controller.*`; `main.cpp::updateDropTestRecording` | host release/landing sequence tests |
| No steering during inert drop | controller `neutral_lock` invariant plus main-loop `emergencyNeutral()` priority | every controller test checks neutral; hardware check still required |
| Reject a brief release spike | persistent `RELEASE_CANDIDATE` and rejection event | `release_requires_persistent_speed_and_altitude_loss` |
| Reject NaN/impossible trigger inputs | finite-value gates | `impossible_values_cannot_trigger_release` |
| Do not claim sensor-only canopy proof | state/event explicitly named `CANOPY_EVENT_SUSPECTED` | `canopy_signature_is_observational_and_timestamped` plus video review |
| Measure settling/oscillation | angular-rate and vertical-speed-range window | `stable_descent_requires_a_quiet_persistent_window` |
| Robust landing detection | persistent low motion + altitude span + GPS corroboration when available | landing and GPS-motion host tests |
| Survive ground-link loss | local controller and flash log have no Wi-Fi/LoRa dependency | code inspection plus radio-off bench test |
| Preserve evidence after touchdown | configurable post-landing interval, final snapshot, close log | landing completion host test |
| Deterministic abort/timeout | neutral lock, final event, close-log output | timeout/abort host test |

## Progressive validation gates

Do not skip a gate because the software passes.

1. **Host/replay:** all tests pass; malformed and stale data cannot produce a
   release or landing marker; output is always neutral.
2. **Powered bench:** stationary data are recorded for longer than the planned
   arm period; no false release; unplugging ground LoRa does not stop logging.
3. **Suspended inert article:** gentle controlled vertical motion confirms sign,
   timestamps, sensor axes, and rejection of short disturbances. No free drop.
4. **Low-energy dummy-mass test:** approved rig/site and soft containment;
   compare payload log with video; inspect battery restraint, lines, and frame.
5. **Pre-opened canopy test:** measure steady descent, oscillation, sink rate,
   and logging without packing/deployment as a confounding variable.
6. **Packed-canopy deployment test:** separately evaluate extraction, line
   staging, inflation signature, and damage. Height is selected by a qualified
   test lead from measured deployment time and site constraints—not by software.
7. **Replay/calibration:** replay each log through the same controller, quantify
   timing error and false markers, then revise the explicitly experimental
   thresholds under configuration version control.

Active steering and flare remain outside these gates. They require a separate
test plan after repeatable neutral descent and reliable reconstruction are
demonstrated.

## Minimum acceptance report for each test

- configuration version and firmware commit;
- test article mass/configuration and measured release height;
- weather/wind or indoor conditions;
- local log file and checksum;
- synchronized video filename;
- arm, release candidate, release confirmation, suspected canopy signature,
  stable descent, landing candidate, landing confirmation, and log-close times;
- sensor validity/age gaps and any reset or brownout;
- maximum angular rate, vertical speed, acceleration transient, servo pulse
  deviation from neutral, and telemetry loss percentage;
- post-test inspection findings;
- explicit pass/fail against the test objective; and
- anomalies and required corrective action before repeating or progressing.
