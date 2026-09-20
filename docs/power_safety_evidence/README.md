# PHOENIX Power Safety Evidence Package

This folder is the evidence package for the PHOENIX recovery module power
system. Its purpose is to prove, with recorded checks, that the battery, UBEC,
servo rail, ESP32, sensors, wiring, and firmware brownout behavior are safe
enough for bench testing and inert drop testing.

This package is not a flight approval by itself. It is a place to collect
evidence. The recovery system should stay out of powered rocket testing until
the forms here are completed, reviewed by an experienced adult, and the inert
recovery tests also pass.

## Files in this package

| File | Purpose |
| --- | --- |
| `POWER_SAFETY_EVIDENCE_INDEX.md` | Main index and pass or fail summary |
| `01_PRE_POWER_INSPECTION.md` | Checks before connecting any battery |
| `02_VOLTAGE_AND_POLARITY_TEST.md` | Multimeter evidence for correct voltage and polarity |
| `03_SERVO_LOAD_AND_BROWNOUT_TEST.md` | Servo-load and ESP32 brownout evidence |
| `04_BATTERY_CHARGE_STORAGE_LOG.md` | Battery charging, storage, and damage log |
| `05_GO_NO_GO_REVIEW.md` | Final review before powered bench or inert drop tests |

## Required evidence before powered bench testing

- Battery is undamaged, correctly charged, and mechanically restrained.
- Battery connector polarity is verified before plugging into the system.
- UBEC output is verified near 5 V before connecting electronics.
- ESP32, sensors, GPS, LoRa, and Wi-Fi stay powered during servo motion.
- Servo rail voltage does not collapse during Servo 1 and Servo 2 movement.
- Firmware reset or brownout behavior is recorded.
- Servos return neutral after reboot, command timeout, and failsafe.
- No wire, servo arm, screw, strap, or payload edge can damage the battery.

## Required evidence before powered drop testing

Powered drop testing must not start until the powered bench test is passed and
recorded. Then repeat the relevant voltage and reset checks with the real
payload configuration installed, but keep the test article inert.

## Related project documents

- `docs/BATTERY_PAYLOAD_SAFETY_PROCEDURE.md`
- `docs/PIN_MAP.md`
- `docs/25_FOOT_DROP_TEST_PROCEDURE.md`
- `docs/25_FOOT_DROP_TEST_FORM.md`
- `docs/INERT_RECOVERY_CALIBRATION_AND_TESTING.md`
- `docs/SAFETY_STATE_MACHINE.md`
