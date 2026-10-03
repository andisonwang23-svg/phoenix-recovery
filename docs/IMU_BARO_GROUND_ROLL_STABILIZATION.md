# IMU/Barometer Ground Roll-Stabilization Test

## Scope

This feature is a short, explicitly commanded **ground or restrained suspended
test**. It is not enabled during launch, deployment, autonomous descent, flare,
or inert drop recording. It does not use GPS.

The BNO085 measures payload roll angle and roll rate. The BMP388 does **not**
measure tilt; it supplies vertical-speed evidence used to reject a moving,
falling, or launch-like test article. Differential parafoil brake input can
influence roll and yaw, but cannot guarantee that a suspended payload becomes
perfectly upright. Canopy, harness, payload, brake geometry, airspeed, and delay
all affect the sign and magnitude of the response.

## Scientific basis

1. NASA TM-4599, *Further Development and Flight Test of an Autonomous
   Precision Landing System Using a Parafoil* (1994), documents differential
   control-line step and pulse tests used to identify directional dynamics. It
   combines angle and rate information for feedback and describes a PID heading
   tracker. Most importantly for PHOENIX, NASA identified the real vehicle
   response from test data before finalizing the controller.
   <https://ntrs.nasa.gov/api/citations/19940029489/downloads/19940029489.pdf>

2. NASA's X-38-derived guided-parafoil report documents two winches controlling
   trailing-edge deflection, shows that control-line deflection changes drag and
   lift-to-drag ratio, and reports substantial coning dynamics during the
   transition to forward flight. This supports keeping PHOENIX neutral during
   deployment and stabilization rather than using attitude correction early.
   <https://ntrs.nasa.gov/api/citations/20070026249/downloads/20070026249.pdf>

3. NASA's 2016 experimental lander flight analysis reports autonomous line
   actuation using GPS and inertial sensors. It also reports small oscillating
   stroke commands and a wavy ground track, attributing the behavior to stroke
   size, turn logic, and sling coupling. This is why PHOENIX uses a low command
   cap, a deadband, rate limiting, and a reversal delay.
   <https://ntrs.nasa.gov/api/citations/20160004964/downloads/20160004964.pdf>

These sources establish an engineering method; they do not validate the gains,
limits, or correction sign for the 1.5 kg PHOENIX article. Those values require
experimental calibration.

## Implemented control law

When the operator presses **Start 15 s Test**, the current roll is captured as
the reference. The signed differential-brake request is:

```text
command = correction_sign × (Kp × roll_error + Kd × roll_rate)
```

The command then passes through:

- a 3 degree roll deadband;
- a 15% brake-command cap;
- a command slew-rate limit;
- a one-second turn-reversal guard; and
- the existing calibrated servo pulse and mechanical limits.

Only one physical brake direction is requested at a time. The saved mapping
routes physical left/right brake requests to Servo 1 and Servo 2.

Every numeric value above is a conservative placeholder and **REQUIRES
EXPERIMENTAL CALIBRATION**.

## Required gates

The controller remains or returns neutral unless all conditions are true:

- an explicit ground command is refreshed through LoRa;
- the payload is in `SELF_TEST` or `PAD_SAFE`;
- launch has never been detected;
- the vehicle and inert drop recorder are not armed;
- the drop recorder is not active;
- the servo controller is healthy;
- the BNO085 and BMP388 are valid and fresh;
- vertical speed is within the stationary-test limit;
- vertical acceleration is below the launch/motion limit;
- roll error, pitch, and roll rate are inside the test envelope; and
- the five-minute post-boot service window and 15-second test duration have not
  expired.

Loss of the repeating LoRa command neutralizes the test within 750 ms. Sensor,
motion, attitude, duration, and interlock failures request immediate neutral.

## Calibration sequence

1. Disconnect both brake lines and remove servo loads.
2. Verify that the dashboard shows healthy IMU and barometer data.
3. Hold the payload in its intended upright mounting orientation.
4. Press **Start 15 s Test**. The displayed reference should match the current
   roll, and the command should remain near zero.
5. Tilt the payload slowly 5 degrees in one direction. Record which physical
   servo moves and the displayed command. Repeat in the other direction.
6. If the physical response would increase the tilt, press **Stop & Neutral**
   and reverse `TILT_STABILIZER_CORRECTION_SIGN` before any connected-line test.
7. Verify sensor removal, barometric motion, excessive angle, command timeout,
   and the Stop button all produce immediate neutral.
8. Only after the unloaded direction test passes, perform a restrained,
   low-energy suspended inert test with a safety observer and a mechanical means
   to prevent a fall. Begin with slack or lightly coupled lines and measure the
   actual delay, roll-rate change, oscillation, and brake travel.

Do not use this mode on a powered rocket or interpret a successful unloaded
servo response as proof of parafoil stability.
