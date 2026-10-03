# Website Servo-Control Repair: Evidence and Verification

Date: 2026-10-02

## Reproduced evidence

- The ground dashboard could display `Sent` after its own LoRa transmit even when the
  payload had not accepted the command. This was a false-positive user interface.
- The last live payload snapshot was already approximately 137 seconds stale. It
  showed both servo commands at zero, so it cannot demonstrate a received servo
  command or physical movement.
- The payload's drop-test arm flag was set when a recording was armed but was not
  cleared in the common publication path after abort or completion. Because the bench
  gate rejects every command while `armed`, one earlier drop-test arm could block all
  later website servo tests until reboot.
- The control signal path itself uses the ESP32-S3 PWM hardware at 50 Hz, calibrated
  1000–2000 microsecond bounds, 1500 microsecond neutral, slew limiting, and a short
  command timeout. Manual operation previously established that the configured GPIOs
  and servo controller can move the servos, but that observation does not prove the
  radio/dashboard path.

## Published and primary evidence

1. Ross et al., *Investigation into soft-start techniques for driving servos*,
   Mechatronics 24(2), 79–86, DOI
   [10.1016/j.mechatronics.2013.11.014](https://doi.org/10.1016/j.mechatronics.2013.11.014),
   describes the usual 20 ms period and roughly 1–2 ms position pulse and reports that
   stepping through intermediate commands reduces abrupt motion, mechanical wear, and
   current inrush. This supports retaining the bounded slew ramp rather than changing
   the fix to an instantaneous pulse jump.
2. Aragon-Jurado et al., *Low-Cost Servomotor Driver for PFM Control*, Sensors 18(1),
   93, DOI [10.3390/s18010093](https://doi.org/10.3390/s18010093), documents a 50 Hz,
   1–2 ms example operating range. This supports the present timing envelope, while
   actual endpoints remain hardware-specific and require measurement.
3. Espressif's ESP32-S3 MCPWM documentation confirms that the S3 provides multiple
   independent PWM outputs and supports setting pulse duty in microseconds:
   <https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/mcpwm.html>.
4. A published ESP32 design reports that wireless startup current peaks required an
   adequate regulator and capacitors to prevent supply-drop resets:
   [10.3390/en15228707](https://doi.org/10.3390/en15228707). Servo motion adds a much
   larger and model-dependent transient load, so software confirmation of a PWM command
   cannot prove that the external servo rail is adequate.

## Implemented repair

- Telemetry wire format v7 reports:
  - last payload command sequence;
  - accepted and rejected command counts;
  - payload servo-controller health;
  - whether bench actuation is active;
  - payload arm/interlock state;
  - payload reset reason.
- The drop recorder's global arm interlock now follows `recording`; abort and completion
  release it instead of leaving it permanently latched.
- The ground website enables bench buttons only with fresh telemetry, healthy attached
  servo outputs, an unarmed payload, and a preflight state.
- The website says `Ground transmitted` until payload telemetry confirms activity. It
  no longer equates a successful ground-radio transmit with physical movement.
- Commands remain limited to explicit 20% or 40% unloaded tests, ramp toward the target,
  expire automatically, and neutralize on stale command or interlock.

## Verification status

- 108 host tests pass.
- Payload firmware builds successfully.
- Ground firmware builds and has been flashed successfully to ground-board MAC
  `A4:CB:8F:A7:03:6C`.
- Physical actuation is **not yet validated**. Matching payload firmware must be flashed,
  then a 20% unloaded test must show all of the following simultaneously:
  1. fresh payload telemetry;
  2. accepted count increments and rejected count does not;
  3. `bench_servo_active=true`;
  4. commanded pulse ramps from 1500 us toward 1600 us for the default calibration;
  5. the selected servo moves and returns to 1500 us;
  6. payload reset reason and uptime do not change during motion.

If steps 1–4 occur but the servo does not physically move, the remaining cause is in
the power, shared ground, connector, or servo hardware—not the website command path.
Measure the servo rail during a restrained/unloaded command before increasing travel.
