#include "state_machine.h"
#include <cmath>

namespace logic {
static bool held(bool condition, uint32_t now, uint32_t duration, StateMachineRuntime& runtime) {
    if (!condition) { runtime.condition_since_ms = 0; return false; }
    if (runtime.condition_since_ms == 0) runtime.condition_since_ms = now;
    return now - runtime.condition_since_ms >= duration;
}

GuidanceMode guidanceModeForState(FlightState state) {
    switch (state) {
        case FlightState::GUIDED_DESCENT: return GuidanceMode::HEADING_TO_TARGET;
        case FlightState::FINAL_APPROACH: return GuidanceMode::FINAL_APPROACH;
        case FlightState::FLARE: return GuidanceMode::FLARE;
        case FlightState::FAILSAFE_DESCENT: return GuidanceMode::MODE_FAILSAFE;
        default: return GuidanceMode::MODE_DISABLED;
    }
}

StateMachineResult updateStateMachine(const StateMachineContext& c, StateMachineRuntime& runtime,
                                      FlightState state, FailCode failure) {
    StateMachineResult out{state, guidanceModeForState(state), failure, false};
    auto transition = [&](FlightState next) {
        out.new_state = next;
        out.guidance_mode = guidanceModeForState(next);
        out.state_changed = next != state;
        if (out.state_changed) runtime.enter(c.now_ms);
    };

    const bool descent_state = state == FlightState::GUIDED_DESCENT ||
        state == FlightState::FINAL_APPROACH || state == FlightState::FLARE ||
        state == FlightState::FAILSAFE_DESCENT;
    if (descent_state) {
        const bool landed_with_baro = c.baro_valid && std::fabs(c.vertical_speed_mps) < c.landed_vs_mps &&
            (!c.gps_valid || c.ground_speed_mps < c.landed_gnd_speed_mps) &&
            (!c.imu_valid || c.angular_rate_dps < c.landed_angular_rate_dps);
        const bool landed_without_baro = !c.baro_valid && c.gps_valid && c.imu_valid &&
            state == FlightState::FINAL_APPROACH &&
            c.ground_speed_mps < c.landed_gnd_speed_mps &&
            c.angular_rate_dps < c.landed_angular_rate_dps;
        const bool landed = landed_with_baro || landed_without_baro;
        if (held(landed, c.now_ms, c.landed_confirm_ms, runtime)) {
            transition(FlightState::LANDED);
            return out;
        }
    }
    if (failure != FailCode::FAIL_NONE) {
        transition(FlightState::FAILSAFE_DESCENT);
        return out;
    }

    if ((state == FlightState::GUIDED_DESCENT || state == FlightState::FINAL_APPROACH) && !c.gps_valid) {
        if (runtime.gps_loss_since_ms == 0) runtime.gps_loss_since_ms = c.now_ms;
        if (c.now_ms - runtime.gps_loss_since_ms >= c.gps_loss_timeout_ms) {
            out.fail_code = FailCode::FAIL_GPS_TIMEOUT;
            transition(FlightState::FAILSAFE_DESCENT);
            return out;
        }
    } else if (c.gps_valid) runtime.gps_loss_since_ms = 0;

    switch (state) {
        case FlightState::BOOT: transition(FlightState::SELF_TEST); break;
        case FlightState::SELF_TEST:
            if (c.baro_valid && c.imu_valid) transition(FlightState::PAD_SAFE);
            break;
        case FlightState::PRE_LAUNCH: transition(FlightState::PAD_SAFE); break;
        case FlightState::PAD_SAFE:
        case FlightState::ARMED: {
            const int votes = (c.vertical_accel_mps2 > c.launch_accel_mps2) +
                (c.vertical_speed_mps > c.launch_vs_mps) +
                (c.altitude_agl_m > c.launch_alt_delta_m);
            if (held(votes >= 2, c.now_ms, c.launch_confirm_ms, runtime)) {
                runtime.launch_seen = true;
                transition(FlightState::ASCENT);
            }
            break;
        }
        case FlightState::ASCENT: {
            const bool apogee = c.baro_valid && c.altitude_agl_m > c.minimum_launch_altitude_m &&
                c.vertical_speed_mps < 0.0f;
            if (held(apogee, c.now_ms, c.apogee_confirmation_time_ms, runtime)) {
                runtime.apogee_seen = true;
                transition(FlightState::APOGEE_CONFIRMED);
            }
            break;
        }
        case FlightState::APOGEE_DETECT: transition(FlightState::APOGEE_CONFIRMED); break;
        case FlightState::APOGEE_CONFIRMED: transition(FlightState::DEPLOYMENT_WAIT); break;
        case FlightState::DEPLOYMENT_WAIT:
            if (c.now_ms - runtime.state_entry_ms >= c.deployment_wait_ms)
                transition(FlightState::PARAFOIL_STABILIZATION);
            break;
        case FlightState::PARAFOIL_STABILIZATION: {
            const bool descending = c.baro_valid && c.vertical_speed_mps < -c.min_descent_speed_mps;
            const bool stable = descending && c.imu_valid &&
                c.angular_rate_dps < c.max_stabilization_gyro_rate_dps &&
                c.vs_jitter_mps < c.stabilize_vs_jitter_mps;
            if (c.now_ms - runtime.state_entry_ms >= c.max_stabilization_time_ms) {
                out.fail_code = FailCode::FAIL_STABILIZATION_TIMEOUT;
                transition(FlightState::FAILSAFE_DESCENT);
            } else if (held(stable, c.now_ms, c.min_stabilization_time_ms, runtime)) {
                transition(FlightState::GUIDED_DESCENT);
            }
            break;
        }
        case FlightState::GUIDED_DESCENT:
            if ((c.baro_valid && c.altitude_agl_m <= c.final_approach_altitude_m) ||
                c.distance_to_target_m <= c.final_approach_radius_m)
                transition(FlightState::FINAL_APPROACH);
            break;
        case FlightState::FINAL_APPROACH:
            if (c.flare_enabled && c.baro_valid && c.imu_valid &&
                c.altitude_agl_m <= c.flare_altitude_m)
                transition(FlightState::FLARE);
            break;
        default: break;
    }
    return out;
}
}  // namespace logic
