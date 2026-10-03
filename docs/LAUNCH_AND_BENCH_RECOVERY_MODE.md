# Launch Recovery and Sensor-Independent Servo Bench Tests

## Launch recovery

Normal launch recovery is automatic and does not require pressing **Arm Drop
Test**. The flight coordinator continues to use this sequence:

```text
powered ascent (neutral servos)
  -> apogee confirmed
  -> deployment wait (neutral servos)
  -> parafoil stabilization (neutral servos)
  -> guided descent, only when target and required data are valid
  -> final approach
  -> landed (neutral servos)
```

The payload opens its flight log at boot. The dashboard now labels an airborne
normal flight as **LAUNCH RECOVERY · AUTO LOG**. The inert drop-test recorder is
still a separate mode: arming it forces neutral for the entire test and must not
be used to enable steering.

Making the bench servo test independent of sensors does not weaken launch
readiness. GPS, IMU, and barometer faults still produce the configured warning,
degraded response, or fail-neutral response in the real launch coordinator.

## Sensor-independent servo tests

The 20% and 40% buttons may be used when GPS, IMU, and barometer are unavailable
because the bench gate does not consult those sensors. The following independent
interlocks still apply:

- an explicit **Bench Test** command is required;
- both servo outputs must be initialized and attached;
- the payload must never have detected launch;
- the inert drop recorder must not be armed or recording;
- the vehicle must not be armed;
- the command may be repeated at any uptime while launch has never been detected;
- the payload command expires after 600 ms unless the ground unit repeats it;
- the ground unit stops repeating after one second and sends neutral; and
- any expired or invalid condition immediately commands neutral.

Use these controls only with the brake lines disconnected and no mechanical
load. A 40% unloaded shaft-motion check is not approval to use 40% brake travel
under a canopy.

## Percentage versus degrees

The buttons command a percentage of the stored, calibrated maximum servo travel,
not an absolute angle. With the placeholder calibration of 1000–2000 microseconds
for 180 degrees and 500 microseconds maximum brake travel:

- 20% is approximately 100 microseconds or 18 degrees from neutral;
- 40% is approximately 200 microseconds or 36 degrees from neutral.

Actual values depend on the saved neutral, endpoints, maximum-brake travel, servo
model, linkage, and supply voltage. The dashboard displays the payload's
estimated current deflection in degrees so the observed value can be recorded.

## Flash compatibility

Servo-angle telemetry changes the payload/ground telemetry packet to version 4.
Flash the payload and ground station from the same build; a version-3 ground
station will intentionally reject version-4 payload packets.
