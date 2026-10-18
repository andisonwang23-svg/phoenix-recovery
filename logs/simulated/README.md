# PHOENIX simulated five-foot drop

`drop_SIM.csv` is synthetic engineering data. It
was not recorded by the payload, ground station, sensors, servos, or a physical
drop. It must not be presented as flight or test evidence.

All future synthetic dashboard records use the `drop_SIM` filename prefix.

## Scenario

- Release height: 5 ft (1.524 m) AGL.
- Sensors are modeled as healthy, with a valid GNSS fix.
- A target is modeled about 30 m away.
- The hypothetical guided profile has 20% authority selected.
- Both servo outputs begin and remain neutral at 1500 microseconds.
- Flare is disabled.
- Configuration version `65535` is reserved in this file to identify synthetic
  data rather than a real payload configuration.

## Simulated result

The article reaches the ground in roughly 0.56 seconds. The release detector
cannot confirm the drop until the altitude loss is nearly the entire 1.524 m.
The simulator-specific 75 m activation floor is disabled. Guidance still does
not activate because the article lands before the deployment-neutral,
stable-descent, and navigation-confirmation timers can finish. The independent
12 m low-altitude steering cutoff also remains in place. The servo commands
therefore remain neutral.

The modeled impact transient is deliberately recorded as
`CANOPY_EVENT_SUSPECTED`, illustrating that the sensor signature is not proof
of an open canopy and must be checked against video. Landing confirmation then
requires five seconds of low motion, followed by ten seconds of post-landing
recording.

## Conclusion

This is useful for checking CSV display and analysis, but a five-foot drop
cannot validate automatic return, canopy deployment, steering response, sink
rate, or glide ratio. Use only an inert article and an adult-supervised,
approved test plan for physical testing.
