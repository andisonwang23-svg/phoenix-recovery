# PHOENIX inert recovery-module calibration

This procedure is for an inert test article only. It does not validate powered
rocket flight, parafoil performance, or flare behavior. Keep `FLARE_ENABLED`
false throughout this program.

## Required calibration record

Record the configuration version, hardware revision, servo model, battery and
servo-rail voltage, pulse width, and measured brake-line travel for every run.
Do not infer line travel from servo angle.

1. Disconnect the canopy load and command neutral. Measure and store the left
   and right neutral pulse widths.
2. Move one channel at a time in small increments. Confirm that left command
   pulls only the left brake and right command pulls only the right brake.
   Correct the persisted reversal flags before continuing.
3. Establish mechanical pulse limits without a loaded canopy. Leave clearance
   before linkage binding, servo stall, line over-travel, and structure contact.
4. Set the software maximum to the smaller conservative travel demonstrated by
   both sides. The current 35% ceiling is a pre-calibration limit, not a result.
5. With a suspended or tethered inert assembly, measure minimum useful brake,
   gentle-turn brake, command-to-motion delay, turn rate, sink rate, and glide
   ratio. Repeat at relevant servo-rail voltages and record asymmetry.

## Progressive test gates

Proceed only when the previous gate is repeatable and its log explains every
command and fault:

1. unloaded bench test;
2. suspended/tethered module;
3. controlled low-energy inert drop;
4. full-mass inert drop;
5. powered vehicle only after repeatable deployment, stable neutral descent,
   conservative steering, and reliable logging have all been demonstrated.

For each gate inject GPS loss, IMU loss, barometer loss, and a reboot. Both
brakes must return to neutral, flare must remain disabled, and a latched
failsafe must not be overwritten. Prefer a stable landing away from the target
over aggressive late correction.

## Logged fields needed for calibration

The firmware snapshot records sensor values, validity and age, state/mode/fail
code, GPS course, target bearing, requested and actual brake commands, pulse
widths, configuration version, and optional battery/servo-rail voltages. The
voltage-valid bits must remain false unless appropriate divider hardware is
installed and calibrated.
