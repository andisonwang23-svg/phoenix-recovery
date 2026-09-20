# PHOENIX RECOVERY — Test Plan

Layered test strategy: **host unit tests** (fast, no hardware) → **bench test
mode** (real board, no rocket) → **simulated flight** (synthetic trajectories on
real pipeline) → **ground tests** → **parafoil drop tests** → supervised flight.
No phase is skipped; no hardware result is claimed until actually observed.

---

## 1. Host unit tests (`pio test -e native`)

Pure logic in `firmware/src/logic/` — no Arduino/ESP32 includes — so these run
on the laptop. Test files live in `tests/` (PlatformIO `test_dir`).

| Test file | Covers (spec §20) |
|---|---|
| `test_nav_math.cpp` | `bearing()`, `distance()`, `normalizeAngle()` (`190→-170`, `-190→170`), heading error (`course 350°, target 10° → +20°`) |
| `test_filters.cpp` | moving average, low-pass, vertical-speed estimator (rejects two-sample noise), filter parameters |
| `test_state_machine.cpp` | all 12 states, every transition, guard persistence, fail transitions (incl. failsafe from every state) |
| `test_guidance.cpp` | proportional steering, deadband, clamp (`MAX_BRAKE_COMMAND`), slew-rate limiting, no rapid alternation, `mode` selection |
| `test_detection.cpp` | launch (2/3-signal confirm, no single-sample), apogee (filtered VS + persist), landing (combination + sustain) |
| `test_data_quality.cpp` | GPS jump/speed reject, barometer spike reject, NaN reject, stale-hold behavior, GPS timeout |
| `test_scenarios.cpp` | the 10 simulation scenarios (spec §19) — see §3 |

Run: `~/pio-venv/bin/pio test -e native`

---

## 2. Bench test mode (`BENCH_TEST_MODE`)

Compile-time enabled in `config.h`. Never detects launch, never auto-guides.
USB serial CLI via `bench/bench_cli.*`:

| Command | Action |
|---|---|
| `status` | board, state, sensors, self-test summary |
| `imu` | BNO08x quaternion / accel / gyro (2 s) |
| `baro` | pressure / temp / altitude AGL (2 s) |
| `gps` | lat/lon/sats/HDOP/fix |
| `lora` | packet counter, RSSI, SNR, loss estimate |
| `servo neutral` | both servos → neutral |
| `servo left 0.25` | left brake fraction 0.25 (clamped to `MAX_BRAKE_COMMAND`) |
| `servo right 0.25` | right brake fraction 0.25 |
| `servo brake 0.25` | symmetric brake 0.25 |
| `target` | show/override TARGET_LAT/LON |
| `state` | current flight state + mode |
| `health` | watchdog, loop rate, data ages, failsafe code |
| `reset` | software reset (leaves FAILSAFE) |

Bench procedure (first servo test, horns removed): see
`docs/GROUND_TEST_CHECKLIST.md` §4–5.

---

## 3. Simulation scenarios (spec §19)

Synthetic trajectories fed through the same estimator/state-machine/guidance
pipeline. Two entry points share the same logic:

* **Host:** `tests/test_scenarios.cpp` (asserts outcomes).
* **On-board:** `SIMULATION_MODE` in `config.h` (rehearses the real firmware on
  the bench).

| # | Scenario | Pass criterion |
|---|---|---|
| 1 | Normal launch → apogee → deploy → guided descent → landing | correct state chain; lands within target radius in sim |
| 2 | GPS loss during descent | no steering commanded from stale/zero GPS; guidance holds or fails safe deterministically |
| 3 | Temporary GPS glitch (drop, then restore) | recovers without uncontrolled turns |
| 4 | Barometer spike | rejected by plausibility; no false apogee/landing |
| 5 | IMU failure | self-test/health → FAILSAFE, neutral servos |
| 6 | LoRa outage | flight continues; telemetry skipped, no control impact |
| 7 | Overshoot past target | heading error wraps correctly; no turn-forever |
| 8 | Heading oscillation | slew/rate limiting prevents rapid alternation |
| 9 | Servo saturation | commands clamped to `MAX_BRAKE_COMMAND`; no out-of-range µs |
| 10 | Landing detection | sustained combination triggers LANDED exactly once |

---

## 4. Hardware integration tests (phases 4–12)

| Phase | Test | Pass criterion |
|---|---|---|
| 4 | LoRa rocket ↔ ground | packet sequence increments, RSSI/SNR visible, loss < threshold |
| 5 | One servo (horn off) | neutral then small deflection; no brownout |
| 6 | Two servos (horns off) | both independent; simultaneous; reverse/neutral calibration stored |
| 7 | Estimator bench | altitude/VS sane against known pressure change (sealed bag squeeze) |
| 8 | State machine bench | manual state forcing (bench CLI) exercises all transitions |
| 9–10 | Ground-test guidance | checklist §5 |
| 11 | 25 foot inert drop test | supervised low-height drop; restraint, release, line routing, battery protection and logging verified |
| 12 | Enable real guidance | only after 1–11 green |

---

## 5. Failsafe injection tests (bench)

| Fault injected | Expected |
|---|---|
| Disconnect GPS (unplug TX line) | `FAIL_GPS_TIMEOUT` after `GPS_LOSS_TIMEOUT_MS`; neutral servos; packet shows failsafe code |
| Disconnect IMU (hold SDA low / remove INT) | `FAIL_IMU`; `emergencyNeutral()` |
| Inject invalid value (bench `baro` spike / NaN) | data-quality reject; no guidance reaction |
| Pull GPS pin to wrong baud | `FAIL_GPS_TIMEOUT`, last-good position held, marked stale |

Every injected fault must show its unique `FAIL_*` code on the ground station.

---

## 6. Phase gates summary

| Gate | Required green before proceeding |
|---|---|
| G1 | `pio test -e native` all pass |
| G2 | Bench: sensors detected, servo neutral/deflections, LoRa packets |
| G3 | Ground checklist complete (sensors, servos, telemetry, failsafe) |
| G4 | 25 foot drop-test procedure passes; higher-altitude or tethered testing still required for brake authority and flare remains disabled |
| G5 | Supervised flight test with neutral-until-deploy, guidance armed |

Gate checklist: `docs/GROUND_TEST_CHECKLIST.md`. Low-height drop procedure:
`docs/25_FOOT_DROP_TEST_PROCEDURE.md`.

Drop-test recording mode and the ground dashboard arm flow are described in
`docs/DROP_TEST_RECORDING.md`.
