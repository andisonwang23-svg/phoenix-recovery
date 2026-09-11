# Phoenix Autonomous Recovery Work Plan

## Objective

Build and validate an autonomous parafoil recovery system that can:

1. Detect launch and apogee reliably.
2. Wait for passive recovery deployment.
3. Confirm that the parafoil is descending stably.
4. Navigate toward a configured GPS landing target.
5. Limit brake commands to experimentally validated values.
6. Return to a predictable fail-neutral state after faults.
7. Record enough data to reconstruct every test.

The current goal is a stable and survivable recovery—not minimum landing error or an automatic flare.

## Current status

- [x] ESP32-S3 firmware builds successfully.
- [x] Native navigation, filtering, and state-machine tests pass.
- [x] IMU and barometer hardware have been detected during bench testing.
- [x] GPS, LoRa, Wi-Fi dashboard, servo, state-machine, and logging modules exist.
- [x] Flare defaults to disabled.
- [ ] Guidance is connected correctly to the active state-machine mode.
- [ ] Failsafe detection reliably triggers a physical servo response.
- [ ] Saved servo calibration is restored after reboot.
- [ ] State-transition timestamps are populated.
- [ ] A valid target is required before guidance can activate.
- [ ] Persistent flight logging works through a complete test.
- [ ] Control limits and parafoil behavior have been measured.
- [ ] The complete firmware has passed replay, HITL, and inert drop testing.

## Safety boundaries

- Use an inert test article for software, bench, suspended, and drop testing.
- Do not connect software to motor ignition or pyrotechnic deployment.
- Keep `FLARE_ENABLED = false` until a separate flare campaign is completed.
- Default to neutral brake commands after invalid data, reboot uncertainty, or control failure.
- Do not command an intentional spiral, full brake, or maximum travel without measured canopy data.
- Do not use the powered rocket until the full-mass inert recovery article passes the required gates.

---

## Phase 1 — Repair the flight-control integration

**Priority: Work on this phase now.**

### 1.1 Connect state-machine mode to guidance

- [ ] Update `GuidanceController` whenever the state machine changes guidance mode.
- [ ] Confirm `GUIDED_DESCENT` selects `HEADING_TO_TARGET`.
- [ ] Confirm `FINAL_APPROACH` selects `FINAL_APPROACH`.
- [ ] Confirm all other states disable guidance or select fail-neutral behavior.
- [ ] Add a test proving guided descent produces a nonzero command for a valid heading error.

### 1.2 Require a valid landing target

- [ ] Add an explicit `target_valid` or `guidance_ready` field.
- [ ] Treat `(0, 0)`, non-finite coordinates, and out-of-range coordinates as invalid.
- [ ] Prevent invalid targets from reporting zero distance.
- [ ] Prevent transition to final approach when the target is invalid.
- [ ] Prevent servo steering when the target is invalid.
- [ ] Expose target readiness through telemetry and the dashboard.

### 1.3 Populate state timestamps

- [ ] Set `armed_ms` when the system becomes armed.
- [ ] Set `launch_ms` when launch is confirmed.
- [ ] Set `stabilization_start_ms` upon entering parafoil stabilization.
- [ ] Update `servo_last_update_ms` whenever physical servo outputs are refreshed.
- [ ] Add rollover-safe timeout tests.

### 1.4 Consolidate failsafe behavior

- [ ] Make every detected failsafe condition call `trigger()` with a specific reason.
- [ ] Apply failsafe commands directly to `ServoController`.
- [ ] Ensure later main-loop logic cannot overwrite an active failsafe command.
- [ ] Initially map every recoverable in-flight fault to neutral brakes.
- [ ] Remove or disable unvalidated spiral-descent and full-brake responses.
- [ ] Make preflight health policy different from in-flight health policy.
- [ ] Allow GPS to acquire a fix before treating it as an in-flight failure.
- [ ] Latch serious in-flight failures until landing or deliberate ground reset.

### 1.5 Restore calibration correctly

- [ ] Load NVS servo calibration before attaching the servos.
- [ ] Validate neutral, minimum, maximum, travel, and reversal values before use.
- [ ] Reject corrupted or physically impossible calibration.
- [ ] Display the active calibration source and version on the dashboard.
- [ ] Add a reboot test proving saved values are restored.

### Phase 1 exit criteria

- [ ] A valid synthetic descent enters guided mode and moves the correct brake.
- [ ] An invalid target never activates guidance.
- [ ] GPS, IMU, barometer, and control-loop failures command neutral brakes.
- [ ] Failsafe output cannot be overwritten within the same or following loop.
- [ ] A reboot restores measured calibration and keeps servos neutral.
- [ ] All integration tests pass repeatedly.

---

## Phase 2 — Make data logging trustworthy

### 2.1 Resolve logging configuration

- [ ] Make `FLASH_EVENT_LOG_ENABLED` control actual logger initialization.
- [ ] Choose one filesystem implementation and use its name consistently.
- [ ] Start a log at arming or the beginning of an inert test.
- [ ] Close and finalize the log after landing or test termination.
- [ ] Preserve previous logs until an explicit ground command removes them.
- [ ] Add log versioning, entry length validation, and integrity checking.

### 2.2 Record required data

- [ ] Timestamp and test ID.
- [ ] Flight state, guidance mode, event code, and failure code.
- [ ] GPS position, validity, satellites, HDOP, speed, and course.
- [ ] Barometric altitude AGL and vertical speed.
- [ ] Roll, pitch, yaw, angular rate, and linear acceleration.
- [ ] Requested and actual left/right servo commands and pulse widths.
- [ ] Measured left/right brake-line travel in the test configuration.
- [ ] Battery and servo-rail voltage when measurement hardware is available.
- [ ] LoRa packet sequence, RSSI, and SNR.
- [ ] Active configuration version/hash.

### 2.3 Retrieve and replay logs

- [ ] Download logs through the ground interface.
- [ ] Convert logs to a documented CSV or binary format.
- [ ] Plot altitude, vertical speed, heading, steering, and faults.
- [ ] Replay a recorded test through the flight coordinator.
- [ ] Confirm replay produces deterministic results.

### Phase 2 exit criteria

- [ ] A complete inert test produces a readable, time-aligned log.
- [ ] State transitions and servo outputs can be explained from the log.
- [ ] Power loss during logging does not destroy earlier completed logs.
- [ ] Replay of the same log produces the same decisions.

---

## Phase 3 — Build integration tests and HITL

### 3.1 Flight-sequence tests

- [ ] Boot → self-test → pad-safe.
- [ ] Pad-safe → launch → ascent.
- [ ] Ascent → apogee → deployment wait.
- [ ] Deployment wait → stabilization → guided descent.
- [ ] Guided descent → final approach → landed.
- [ ] Flare remains unreachable while disabled.

### 3.2 Fault-injection tests

- [ ] No GPS fix before launch.
- [ ] Short and sustained GPS loss during descent.
- [ ] GPS position jump and invalid coordinates.
- [ ] Barometer spike, bias, freeze, and complete failure.
- [ ] IMU dropout, reset, NaN, and excessive angular rate.
- [ ] Servo saturation, delay, stuck servo, and asymmetric travel.
- [ ] Low battery and servo-rail brownout indication.
- [ ] LoRa, Wi-Fi, and ground-station loss without loss of onboard control.
- [ ] ESP32 reboot during every flight state.
- [ ] Watchdog reset and uncertain post-reboot state.

### 3.3 Improve the simulator

- [ ] Run the real state machine and guidance functions rather than duplicated Python rules.
- [ ] Model separate left and right brake response.
- [ ] Model command delay and slew rate.
- [ ] Model airspeed, sink rate, glide ratio, wind, and gusts.
- [ ] Model asymmetric turns and increased sink rate during turns.
- [ ] Add explicit pass/fail assertions to every scenario.
- [ ] Fail the test suite when landing error or safety limits exceed their gates.

### Phase 3 exit criteria

- [ ] The normal sequence passes end to end.
- [ ] Every named fault scenario actually injects the stated fault.
- [ ] All critical faults produce a deterministic fail-neutral response.
- [ ] No test requires unmeasured parafoil parameters to pass.

---

## Phase 4 — Bench calibration

### 4.1 Electrical and sensor checks

- [ ] Confirm the final pin map on the assembled vehicle.
- [ ] Verify the IMU and barometer share GPIO 47/48 without bus conflicts.
- [ ] Confirm GNSS wiring, baud rate, power control, and outdoor fix.
- [ ] Confirm LoRa frequency and packet reception at the ground station.
- [ ] Measure logic, battery, and servo-rail voltage under peak servo load.
- [ ] Check for ESP32 resets or sensor corruption during simultaneous servo movement and LoRa transmission.

### 4.2 Servo geometry

- [ ] Remove or disconnect loaded brake lines before initial movement tests.
- [ ] Measure left and right neutral pulse widths.
- [ ] Verify left/right reversal settings.
- [ ] Measure actual line travel at small pulse-width increments.
- [ ] Identify mechanical limits, binding, horn flex, and line slipping.
- [ ] Store pulse width and brake-line travel in millimeters.
- [ ] Set a conservative software limit below the measured mechanical limit.
- [ ] Reboot and verify that calibration persists.

### 4.3 Power endurance

- [ ] Run sensors, GPS, LoRa, Wi-Fi, logging, and both servos together.
- [ ] Exercise conservative steering for longer than the expected recovery time.
- [ ] Record minimum battery and servo-rail voltage.
- [ ] Check regulator, wiring, battery, and servos for overheating.
- [ ] Confirm no watchdog reset, brownout, or filesystem corruption.

### Phase 4 exit criteria

- [ ] All hardware remains stable under worst expected electrical load.
- [ ] Servo direction and neutral are verified physically.
- [ ] Calibration survives reboot.
- [ ] No command exceeds measured conservative travel.

---

## Phase 5 — Characterize the parafoil with an inert test article

Use the final parafoil, harness, line lengths, servos, regulator, electronics, and representative recovery mass.

### 5.1 Neutral descent

- [ ] Confirm repeatable inflation and line extension.
- [ ] Measure trimmed forward airspeed using reciprocal runs when practical.
- [ ] Measure straight-flight sink rate.
- [ ] Calculate median and conservative glide ratio.
- [ ] Measure pendulum motion and neutral angular-rate noise.

### 5.2 Steering characterization

- [ ] Find the minimum repeatable useful left brake input.
- [ ] Find the minimum repeatable useful right brake input.
- [ ] Identify a conservative gentle-turn command for each side.
- [ ] Measure response delay, turn rate, radius, and altitude loss.
- [ ] Measure recovery after gradually returning to neutral.
- [ ] Identify asymmetry between left and right response.
- [ ] Set initial flight limits to the gentle-turn values, not maximum stable travel.

### 5.3 Stability limits

- [ ] Observe warning signs without deliberately holding a collapsed canopy.
- [ ] Record tip folding, pressure loss, oscillation, servo stall, or sudden sink increase.
- [ ] Establish an operational limit comfortably below abnormal behavior.
- [ ] Keep flare disabled.

### Phase 5 exit criteria

- [ ] At least three repeatable valid runs support each accepted parameter.
- [ ] Important safety limits have at least five valid repetitions.
- [ ] Conservative airspeed, sink rate, glide ratio, response delay, and turn limits are documented.
- [ ] Firmware and simulator use the measured conservative values.

---

## Phase 6 — Progressive autonomous inert testing

### Stage 1: Static replay

- [ ] Feed recorded data through the complete firmware decision path.
- [ ] Verify expected state transitions and servo commands.

### Stage 2: Hardware-in-the-loop

- [ ] Drive the real controller with synthetic sensor inputs.
- [ ] Observe physical servo motion and telemetry.
- [ ] Inject sensor and communications faults.

### Stage 3: Suspended or tethered article

- [ ] Verify harness geometry, servo loading, line routing, and fail-neutral behavior.
- [ ] Stop for binding, line slack, oscillation, resets, or overheating.

### Stage 4: Neutral drop tests

- [ ] Keep autonomous steering disabled.
- [ ] Validate deployment, inflation, descent sensing, state transitions, and logging.
- [ ] For 25 foot drops, use `docs/25_FOOT_DROP_TEST_PROCEDURE.md` and treat
      the result as a restraint/release/logging check, not a guidance or flare
      validation.

### Stage 5: Limited-authority steering drops

- [ ] Enable only the measured gentle-turn command.
- [ ] Do not start this stage from 25 foot drops alone; use a suspended/tethered
      setup or a higher supervised inert-drop setup with enough altitude for
      stable canopy inflation.
- [ ] Use a broad target area and generous altitude gates.
- [ ] Confirm correct turn direction and neutral recovery.

### Stage 6: Target-guidance drops

- [ ] Freeze configuration before each validation series.
- [ ] Test different target bearings and safe wind conditions.
- [ ] Compare predicted and actual landing position.
- [ ] Record every result, including failures.

### Phase 6 exit criteria

- [ ] Multiple full-mass inert drops complete without unsafe behavior.
- [ ] No person, structure, road, or power line is placed under the test path.
- [ ] Guidance remains within calibrated authority.
- [ ] Fault injection produces neutral brakes.
- [ ] Landing performance is repeatable within the predefined acceptance area.

---

## Phase 7 — Powered-flight readiness gate

Do not proceed until every item below is complete.

- [ ] Independent adult/mentor review of mechanical, electrical, and software systems.
- [ ] Approved launch site and operational permission.
- [ ] Representative full-mass inert recovery validation completed.
- [ ] Deployment reliability demonstrated separately from guidance.
- [ ] Configuration frozen and versioned.
- [ ] Target and geofence verified at the site.
- [ ] Servo limits and neutral checked immediately before flight.
- [ ] Battery and servo rail pass load test.
- [ ] GPS fix and sensor health pass readiness checks.
- [ ] LoRa telemetry and onboard logging verified.
- [ ] Flare remains disabled.
- [ ] Recovery system does not control ignition or pyrotechnics.
- [ ] Abort criteria, exclusion zone, and recovery procedure are defined.

---

## Current work queue

Complete these tasks before beginning physical parafoil tuning:

1. [ ] Fix guidance-mode propagation.
2. [ ] Add target-valid and guidance-readiness gates.
3. [ ] Repair failsafe triggering and servo-command ownership.
4. [ ] Populate state and actuator timestamps.
5. [ ] Restore NVS calibration during boot.
6. [ ] Add coordinator-level integration tests.
7. [ ] Repair and verify persistent logging.
8. [ ] Replace placeholder simulator cases with real fault injection.
9. [ ] Run bench calibration with unloaded servo lines.
10. [ ] Begin inert parafoil characterization only after the software exit gates pass.

## Definition of the next software milestone

The next milestone is complete when:

> Synthetic or replayed sensor data drives the real firmware through deployment and stabilization into guided descent; the correct limited brake command is produced for a valid target; every injected critical fault commands neutral brakes; calibration survives reboot; and the complete decision history is recorded in a retrievable log.
