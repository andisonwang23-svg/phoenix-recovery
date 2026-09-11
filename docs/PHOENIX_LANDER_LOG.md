# Phoenix Lander Project Log

This log tracks Phoenix lander development events, successes, mistakes, and fixes.

## Status key

- **Success** — the work produced the intended result or passed verification.
- **Mistake** — a fault, bad configuration, or failed result was found.
- **Mixed** — useful progress was made, but important work remains unfinished.
- **N/A** — no mistake needed to be fixed.

## Project history

| Date | What happened | What it does | Mistake or success? | Was the mistake fixed? |
| --- | --- | --- | --- | --- |
| 2026-08-07 | The initial software architecture, safety state machine, and test plan were documented. | Defines how the lander software is organized, behaves safely, and will be tested. | Success | N/A |
| 2026-08-08 | The ESP32 Arduino framework installation was found with about 3,855 zero-byte files, which prevented a trustworthy build. | The framework supplies the ESP32 code and tools needed to compile the lander firmware. | Mistake | **Yes.** The damaged framework package was removed, downloaded again by PlatformIO, and the rocket firmware built successfully. |
| 2026-08-08 | The PlatformIO project used settings in locations that PlatformIO 6.x did not honor, including per-environment source and variant configuration. | The corrected configuration tells PlatformIO which firmware and Heltec board files to build. | Mistake | **Yes.** `src_dir` was moved to the global section, the Heltec variant path was corrected, and the ground-station firmware was separated into its own project. |
| 2026-08-09 | The first complete autonomous parafoil-recovery firmware structure was documented, including sensors, state estimation, guidance, servos, telemetry, safety systems, and logging. | Provides the main software pieces needed to sense flight, steer the parafoil, communicate, and fail safely. | Success | N/A |
| 2026-08-10 | Native tests were added for navigation math and filters. The documented result was 49 passing tests. | Checks heading, distance, smoothing, and command-limiting logic without needing the flight hardware. | Success | N/A |
| 2026-08-11 | Autonomous-flight behavior and ground-testing procedures were documented. | Explains the intended flight sequence and how to test it safely on the ground. | Success | N/A |
| 2026-08-16 | A firmware build was documented at 15.2% RAM use and 27.2% flash use, with the major flight modules present. | Confirms the firmware fits on the controller with substantial memory remaining. | Success | N/A |
| 2026-08-18 | The Heltec V4 build configuration and IMU/LoRa integration were updated. Bench history records successful detection of the BNO08X IMU and BMP388 barometer on GPIO 47/48. | Lets the controller read motion and altitude sensors and use the LoRa radio. | Success | N/A |
| 2026-08-20 | GPS, guidance, failsafe, telemetry, simulation, and descent-control tests were expanded. | Improves navigation, fault handling, status reporting, and safe software-only testing. | Success | N/A |
| 2026-08-27 | A repository review found flight-integration risks: guidance-mode wiring, invalid targets, missing timestamps, failsafe output handling, calibration restoration, and persistent logging still needed proof or repair. | Identifies problems that could prevent correct steering, recovery, or reconstruction of a test. | Mistake | **Partly.** The repository now contains native tests covering the main control and safety cases, but hardware, persistent-log, replay, HITL, and inert-drop verification are still required. |
| 2026-08-27 | Safety goals were tightened: flare was disabled, faults were required to return brakes to neutral, and powered testing was gated behind inert recovery testing. | Keeps the parafoil commands conservative until physical testing proves more aggressive behavior is safe. | Success | N/A |
| 2026-08-27 | Flight-coordinator tests were added for the full guided sequence, missing targets, GPS loss, critical sensor failures, calibration restoration, log closure, communications loss, and latched failures. | Checks that the main decision logic steers only when ready and returns to safe commands after faults. | Success | N/A |
| 2026-08-29 | Barometer, sensor-manager, main-loop, and Wi-Fi dashboard integration were updated. | Connects sensor readings to flight decisions and displays lander status over Wi-Fi. | Mixed | N/A; native verification exists, but complete hardware verification is still pending. |
| 2026-08-30 | Servo control and the Wi-Fi dashboard were updated. | Moves the parafoil brake servos and exposes controls and status through the dashboard. | Mixed | N/A; bench testing with the final servo geometry and electrical load is still pending. |
| 2026-08-30 | The left and right servo names were replaced with **Servo 1** and **Servo 2**. | Gives the two servo channels neutral numbered names, making them easier to identify before their physical direction and brake-line mapping are confirmed. | Success | N/A |
| 2026-08-30 | The Wi-Fi problem was fixed. | Restores the lander's Wi-Fi connection so the dashboard and its status or bench-test controls can be accessed. | Mistake → Success | **Yes.** The Wi-Fi is now working. |
| 2026-08-30 | The full native test suite was run: **74 of 74 tests passed** across navigation math, filters, descent control, and the flight coordinator. | Verifies the hardware-independent navigation, filtering, descent, and safety decision logic. | Success | N/A |
| 2026-08-31 | Servo-controller initialization was corrected to check whether each ESP32 servo is actually attached instead of treating the returned channel number as a true/false result. Saved calibration is also restored before attachment, and unchanged pulse values are no longer rewritten continuously. | Prevents a valid channel numbered zero from disabling the controller, preserves bench calibration after reboot, and reduces unnecessary timer updates that can look like servo twitch. | Mistake → Mixed | **Software fix added; hardware confirmation is still pending.** |
| 2026-08-31 | Wi-Fi startup and health handling were strengthened: the access point starts early, retries if initial startup fails, and no longer disconnects clients automatically after a questionable health check. | Makes the Phoenix dashboard available sooner and avoids periodic connection drops caused by unnecessary access-point resets. | Success | N/A; the prior Wi-Fi issue remains recorded as fixed, with continued hardware observation recommended. |
| 2026-08-31 | The dashboard gained Servo 1/Servo 2 turn and estimated-angle readouts plus an IMU-based response-delay measurement for suspended ground tests. | Helps measure how far each servo is commanded and how long the safely suspended test article takes to begin moving. | Mixed | N/A; the feature exists in firmware, but physical calibration data has not yet been recorded. |
| 2026-08-31 | Flight-control fault handling was expanded with a limited barometer-loss fallback during guided descent and final approach, while loss of both the IMU and barometer still triggers a latched fail-neutral response. New coordinator tests cover the fallback and landing behavior. | Allows conservative navigation to continue briefly using valid GPS and IMU data after a barometer failure without weakening the response to multiple critical sensor failures. | Mixed | N/A; the code and tests were added, but the updated suite and hardware behavior have not yet been verified in this log. |
| 2026-08-31 | Three payload-layout drawings were added: a cutaway sled diagram, a photo-based system revision, and a removable recovery-cartridge concept. | Shows proposed locations for the parafoil, electronics, battery, servos, line channels, structural load path, and serviceable cartridge. | Success | N/A; these are conceptual drawings, and final dimensions and structural hardware still require physical measurement and review. |
| 2026-08-31 | Four payload-restraint and servicing visuals were added, including labeled retention hardware, a representation using the real payload, and a removable battery tray for charging outside the rocket. | Illustrates centering plates, an anti-rotation key, hard-stop ring, retention pins, pull handle, switch access, and a safer way to remove and charge the 1S battery. | Success | N/A; the restraint and charging arrangement is still a concept and must be dimensioned, structurally reviewed, and physically tested before flight. |
| 2026-09-03 | The battery was cut/damaged, so the battery and electrical setup must be restarted with a safe replacement. | Prevents a damaged battery from powering the Phoenix electronics and requires power wiring and system checks to be repeated before testing resumes. | Mistake | **No, not yet.** Do not reuse, power, or charge the damaged battery. Replace it, inspect the leads and connectors for shorts or damage, then repeat polarity, voltage, power-up, and servo-load checks before reconnecting the flight electronics. |
| 2026-09-03 | A battery and payload safety procedure was added after the damaged-battery incident. | Provides rules and checklists for battery mounting, electrical inspection, charging, storage, damaged-battery disposal, and safer payload redesign. | Success | N/A; the procedure addresses prevention, but the damaged battery still needs safe disposal and the Phoenix power system still needs an undamaged replacement and repeat testing. |
| 2026-09-03 | Two battery-protection concepts were added: a smooth, rounded battery cover with a wire exit and air gap, plus a layout that places a flame blanket and wadding between the battery tray and ejection heat without wrapping the LiPo. | Shows how to shield the replacement battery from sharp edges, movement, and ejection heat while keeping it removable and inspectable. | Success | N/A; these are design concepts and still require compatible materials, dimensions, clearance checks, and physical validation before use. |
| 2026-09-04 | The Phoenix firmware files were successfully flashed to a new, working ESP32. | Moves the current lander software onto a functional flight controller so hardware bring-up and bench testing can continue. | Success | N/A; flashing succeeded, but sensor detection, Wi-Fi, LoRa, Servo 1/Servo 2, calibration persistence, and full-load power behavior still need to be checked on the new board. |
| 2026-09-05 | The sensor manager was changed to copy sensor-owned readings into the shared vehicle state instead of replacing the entire state, and sensor freshness timestamps were tightened. | Prevents sensor updates from erasing guidance, servo, health, telemetry, or state-machine results produced by other parts of the firmware, while making stale-sensor detection more trustworthy. | Mistake → Mixed | **Software fix added; updated tests and hardware verification on the new ESP32 are still pending.** |
| 2026-09-05 | Detailed BMP388 diagnostics and IMU-only bench bring-up support were added. | Reports barometer reads, failures, rejected samples, recovery attempts, raw measurements, and timestamps so the new ESP32 can still be diagnosed safely when the barometer is unavailable. | Mixed | N/A; this improves diagnosis and bench access but does not prove the BMP388 is working on the new board. |
| 2026-09-07 | A minimal IMU/I2C diagnostic program was added for the new ESP32. It scans GPIO 47/48, checks both common BNO08x addresses, prints wiring help, and reports roll, pitch, and yaw when the sensor starts. | Isolates the ESP32 and BNO085 from the rest of the rocket firmware so wiring, I2C communication, sensor startup, and tilt output can be checked one step at a time. | Mixed | N/A; the diagnostic is ready, but this log does not yet contain a successful hardware run. |
| 2026-09-07 | A supervised 25-foot inert drop-test procedure and reusable result form were added, and the main test plans were updated to reference this new gate. | Provides five progressive stages for checking the drop zone, payload restraint, battery protection, parafoil release, electronics survival, and readable logging without claiming guidance or flare performance. | Success | N/A; no drop-test result is recorded yet, and higher-altitude or suspended testing is still required for steering behavior. |
| 2026-09-08 | A preflight launch-readiness interlock and warning were added. Launch-like motion with an invalid barometer, GPS, or servo now latches a fail-neutral condition instead of allowing normal flight progression. | Stops the controller from treating an unhealthy vehicle as ready to launch and exposes the blocked-launch warning in telemetry and the Wi-Fi dashboard. | Success | N/A; the new native tests passed, but a hardware fault-injection check is still required. |
| 2026-09-08 | Guided descent gained a reduced-authority IMU-loss fallback, while combined critical-sensor loss still latches failsafe. Wi-Fi access-point startup and recovery handling were also expanded. | Permits conservative control from remaining valid navigation data after one sensor loss and improves dashboard recovery without allowing Wi-Fi to control flight behavior. | Mixed | N/A; software tests passed, but sensor-loss and Wi-Fi recovery behavior still need hardware verification. |
| 2026-09-08 | The IMU diagnostic was expanded to detect and read the BMP388, but an I2C pin-order conflict was found: `config.h` lists SDA 47/SCL 48 while `hardware_config.h` and the diagnostic list SDA 48/SCL 47. | The diagnostic can now isolate both major I2C sensors, but inconsistent pin definitions may cause it and the full firmware to test different wiring. | Mistake | **No.** Confirm the physical SDA/SCL wiring on the new ESP32, then make every configuration, diagnostic, and pin-map document use the same verified order before another hardware conclusion is recorded. |
| 2026-09-08 | The full firmware I2C definition was corrected to match the hardware configuration and diagnostic: SDA 48/SCL 47. The bus was also set to the scanner-verified 100 kHz rate with a 750 ms sensor timeout. | Makes the diagnostic and flight firmware use the same sensor wiring and more conservative communication settings. | Mistake → Mixed | **Software conflict fixed.** The assembled ESP32 still needs a physical sensor scan and readout before the wiring is considered fully verified. |
| 2026-09-08 | L76K GPS handling and the Wi-Fi dashboard gained separate NMEA-activity and valid-fix reporting, recovery filtering, richer fix data, and saved landing-target coordinates. | Helps distinguish a connected GPS that is sending data from one that has a usable satellite fix, and preserves the selected target after a reboot. | Mixed | N/A; the code is present and native tests pass, but an outdoor GPS fix and reboot-persistence check are still required on the new ESP32. |
| 2026-09-08 | The complete native suite was run after the flight-control changes: **81 of 81 tests passed**. | Verifies navigation math, filtering, descent control, launch-readiness blocking, sensor fallbacks, fail-neutral behavior, calibration restoration, and flight-log closure without flight hardware. | Success | N/A |
| 2026-09-09 | The GPS was tested on the new ESP32 and confirmed working. | Confirms the lander can receive GPS data for position, navigation, and recovery tracking. | Success | **Yes for the previously pending GPS hardware check.** Outdoor accuracy, target persistence after reboot, and full-system flight behavior remain separate tests. |

## Current unresolved items

These are not recorded as fixed until they pass the required physical or end-to-end test:

- Verify the final pin map and the remaining sensors on the assembled vehicle; the GPS was confirmed working on 2026-09-09.
- Measure servo neutral points, direction, limits, and actual brake-line travel.
- Confirm saved calibration survives a real ESP32 reboot.
- Confirm every critical fault physically commands both servos to neutral.
- Complete a persistent flight log, retrieve it, and replay it successfully.
- Complete replay, hardware-in-the-loop, suspended/tethered, and inert-drop tests.
- Measure parafoil airspeed, sink rate, glide ratio, response delay, and conservative steering limits.
- Keep automatic flare disabled until a separate flare test campaign is completed.

## Add a new entry

Copy this row when something new happens:

| Date | What happened | What it does | Mistake or success? | Was the mistake fixed? |
| --- | --- | --- | --- | --- |
| YYYY-MM-DD | Describe what happened and include the test or evidence. | Briefly explain the purpose or effect of the change. | Mistake / Success / Mixed | Yes, with the fix described / No / N/A |

## Evidence used for past entries

- `docs/BUILD_TOOLCHAIN.md`
- `docs/FIRMWARE_SUMMARY.md`
- `docs/IMPLEMENTATION_SUMMARY.md`
- `docs/AUTONOMOUS_RECOVERY_WORK_PLAN.md`
- `docs/INERT_RECOVERY_CALIBRATION_AND_TESTING.md`
- `docs/BATTERY_PAYLOAD_SAFETY_PROCEDURE.md`
- `docs/CHAT_HISTORY.md`
- File modification dates in this project folder
- Native PlatformIO test run completed on 2026-09-08: 81 of 81 tests passed

> Note: this repository does not yet have Git commits. Dates before 2026-08-30 are reconstructed from dated project files and project documentation, not from commit history.
