# Coordinator replay and HITL contract

Replay must feed timestamped sensor snapshots through
`logic::FlightCoordinator::step`; it must not reproduce the guidance algorithm
in a separate model. A replay row contains timestamp/delta time, position,
target, AGL altitude, vertical speed/acceleration, ground speed/course, angular
rate, vertical-speed jitter, validity flags, and servo health.

The plant surrounding the real coordinator may inject wind/course disturbance,
delayed response, left/right brake scaling, saturation or stall, GPS dropout,
barometer bias/spikes, sensor reset windows, and angular oscillation. Requested
and simulated actual outputs must remain separate.

Every run reports final target error, maximum requested/actual brake, reversal
count, GPS-loss neutralization, failsafe state/code, oscillation detection, and
the glide-ratio reachability estimate. Recorded logs use the same fields, so a
captured flight can replace synthetic rows without changing controller code.

The native `test_flight_coordinator` suite is the first deterministic replay:
it drives the actual coordinator through launch, apogee, deployment,
stabilization, guidance, GPS dropout, critical sensor failures, reboot
configuration restoration, and landing/log closure. Add captured rows to this
suite as inert test data becomes available; do not tune landing accuracy from
the current synthetic model.
