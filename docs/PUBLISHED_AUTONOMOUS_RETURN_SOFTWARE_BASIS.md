# PHOENIX Published Autonomous-Return Software Basis

## Purpose

PHOENIX uses an independent, fail-neutral implementation informed by published
autonomous parafoil projects. It does **not** copy a drone motor controller or
assume that multirotor return-to-home behavior applies to a parafoil. A parafoil
continuously descends, responds after a delay, and turns by asymmetric brake-line
input.

The current implementation is intended for simulation, replay, and progressive
inert testing. It does not establish that the real canopy is controllable or
that the target is reachable.

## Primary published basis

### RAPTOR — small autonomous parafoil return project

- [RAPTOR project repository](https://github.com/raptorshc/RAPTOR)
- [RAPTOR 2019 AIAA paper](https://github.com/raptorshc/RAPTOR/blob/master/information/papers/RAPTOR%20AIAA%20Paper%202019.pdf)
- [Published RAPTOR software snapshot](https://github.com/raptorshc/software/tree/4b5aa1df2f86da6daa911b445d8325e54b07ef93)

Applicable ideas:

- separate deployment/descent/landed behavior;
- neutral time for parafoil deployment and inflation;
- GPS position plus desired bearing to a target;
- an initial turn followed by a straight-flight heading window;
- local sensor and flight-state logging; and
- persistent flight-state information for reset recovery.

RAPTOR's published control code uses continuous-rotation servos/winches and
coarse left/straight/right decisions. PHOENIX uses positional BW90S servos and
therefore retains calibrated pulse limits, proportional brake commands, slew
limiting, reversal delay, GPS quality gates, and fault-neutral behavior.

The RAPTOR repositories do not contain an explicit software license in the
reviewed snapshot. PHOENIX therefore contains no copied RAPTOR source. The
implementation below was written independently from the published behavior and
technical descriptions.

### NASA X-38-derived guided parafoil system

- [Madsen, Stein, and Strahan, *An Overview of the Guided Parafoil System Derived from X-38 Experience*](https://ntrs.nasa.gov/api/citations/20070026249/downloads/20070026249.pdf)

The NASA paper describes target acquisition, a homing leg, an energy-management
circle, a final approach selected for the wind, and a dynamically triggered
flare. It also describes extensive simulation, manual characterization
maneuvers, instrumentation, and post-test reconstruction.

PHOENIX currently implements only the portions that can be used conservatively
without a validated aerodynamic model:

1. `TARGET_ACQUISITION` — turn gradually toward the target.
2. `HOMING` — remain in the straighter target-tracking phase after alignment is
   sustained.
3. `FINAL_APPROACH` — reduce steering authority close to the target or ground.

The energy-management circle, wind-selected landing direction, and automatic
flare are not enabled. They require measured airspeed, sink rate, glide ratio,
turn rate, actuator delay, wind estimation, and sufficient altitude.

### NASA autonomous parafoil development and test evidence

- [Murray et al., *Further Development and Flight Test of an Autonomous Precision Landing System Using a Parafoil*](https://ntrs.nasa.gov/citations/19940029489)
- [Chin et al., *Flight Analysis of an Autonomously Navigated Experimental Lander for High Altitude Recovery*](https://ntrs.nasa.gov/api/citations/20160004964/downloads/20160004964.pdf)

These records support GPS-based navigation, onboard recording, progressive
flight testing, performance reconstruction, and keeping deployment/inflation
separate from guided flight.

## Code mapping

| Published behavior | PHOENIX implementation | Additional PHOENIX safety behavior |
|---|---|---|
| Deployment before guidance | `GuidedDropController::DEPLOYMENT_NEUTRAL` | Both brakes remain neutral until release, canopy-event observation, stable descent, sensor freshness, navigation confirmation, altitude, and authority gates pass. |
| Target acquisition turn | `GuidedDropStatus::TARGET_ACQUISITION` | Initial authority is capped and still passes through pulse limits, command slew limiting, reversal delay, and oscillation damping. |
| Straight homing leg | `GuidedDropStatus::HOMING` | Alignment must remain inside a configurable window for a confirmation period; a wider re-entry threshold prevents phase chatter. |
| Final approach | `GuidedDropStatus::FINAL_APPROACH` | Separate smaller steering limit and larger effective deadband. |
| Reachability/energy awareness | Distance is compared with `altitude × measured_glide_ratio` | An apparently unreachable target receives only a small configurable brake limit; stability is prioritized. |
| Onboard autonomy | Payload controller owns the sequence after release | Wi-Fi and LoRa are not required control inputs; communication loss cannot command a turn. |
| Flight-test reconstruction | Payload-local synchronized log plus replay tools | GPS validity/age, sensor validity/age, requested and actual servo output, configuration version, and event times are retained. |

## New configuration values

Every motion-dependent value below is marked `REQUIRES EXPERIMENTAL
CALIBRATION` in `firmware/src/config.h`:

- `GUIDED_DROP_HOMING_ALIGNMENT_DEG`
- `GUIDED_DROP_ACQUISITION_REENTRY_DEG`
- `GUIDED_DROP_HOMING_CONFIRM_MS`
- `GUIDED_DROP_TARGET_ACQUISITION_MAX_STEERING`
- `GUIDED_DROP_UNREACHABLE_MAX_STEERING`
- `GUIDED_DROP_NOMINAL_GLIDE_RATIO`

The nominal glide ratio is only a placeholder until it is measured from inert
drop data. It must not be used as evidence that a particular release point is
safe or reachable.

## Required validation order

1. Host tests and recorded-data replay.
2. Powered bench test with unloaded servos and confirmed neutral return.
3. Suspended/tethered inert article.
4. Neutral pre-opened-canopy drops.
5. Measured single-side 10–20% brake maneuvers.
6. Identification of response delay, turn rate, sink rate, glide ratio, and
   left/right asymmetry.
7. Replay with wind, GPS loss/jumps, sensor resets, servo saturation, and
   asymmetric response.
8. Limited target-acquisition and homing tests at an authorized field.
9. Final-approach experiments only after the preceding stages are repeatable.

`GUIDED_DROP_TEST_ENABLED`, every staged authority unlock, and automatic flare
remain false by default. A carrier drone may provide repeatable release
conditions, but it does not validate the recovery controller by itself.
