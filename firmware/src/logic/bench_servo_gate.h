#pragma once

#include <cstdint>
#include "state_machine.h"

namespace logic {

struct BenchServoGateInput {
    FlightState flight_state = FlightState::BOOT;
    uint32_t launch_ms = 0;
    bool drop_test_recording = false;
    bool armed = false;
    bool servo_healthy = false;
};

// This gate deliberately does not depend on GPS, IMU, or barometer health.
// It is only for an explicitly requested, unloaded preflight bench movement.
inline bool benchServoTestAllowed(const BenchServoGateInput& in) {
    const bool preflight_state =
        in.flight_state == FlightState::BOOT ||
        in.flight_state == FlightState::SELF_TEST ||
        in.flight_state == FlightState::PRE_LAUNCH ||
        in.flight_state == FlightState::PAD_SAFE ||
        // A sensor-only preflight failure may place the coordinator here. It is
        // still a bench condition only when launch has never been detected.
        in.flight_state == FlightState::FAILSAFE_DESCENT;
    return preflight_state &&
           in.launch_ms == 0 &&
           !in.drop_test_recording &&
           !in.armed &&
           in.servo_healthy;
}

} // namespace logic
