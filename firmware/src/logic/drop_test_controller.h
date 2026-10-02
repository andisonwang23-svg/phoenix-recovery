// ============================================================================
// PHOENIX RECOVERY — Inert drop-test event detector.
//
// Hardware-free by design so recorded data can be replayed through exactly the
// same logic on a host computer.  This controller NEVER authorizes steering.
// Sensor-derived canopy events are observations, not proof of full deployment.
// ============================================================================
#pragma once

#include <cstdint>

namespace logic {

enum class DropTestState : uint8_t {
    IDLE = 0,
    ARMED_RECORDING = 1,
    RELEASE_CANDIDATE = 2,
    DROP_CONFIRMED = 3,
    CANOPY_EVENT_SUSPECTED = 4,
    STABLE_DESCENT_OBSERVED = 5,
    LANDING_CONFIRM = 6,
    POST_LANDING = 7,
    TEST_COMPLETE = 8,
    TEST_ABORTED = 9
};

inline const char* dropTestStateName(DropTestState state) {
    switch (state) {
        case DropTestState::IDLE: return "IDLE";
        case DropTestState::ARMED_RECORDING: return "ARMED_RECORDING";
        case DropTestState::RELEASE_CANDIDATE: return "RELEASE_CANDIDATE";
        case DropTestState::DROP_CONFIRMED: return "DROP_CONFIRMED";
        case DropTestState::CANOPY_EVENT_SUSPECTED: return "CANOPY_EVENT_SUSPECTED";
        case DropTestState::STABLE_DESCENT_OBSERVED: return "STABLE_DESCENT_OBSERVED";
        case DropTestState::LANDING_CONFIRM: return "LANDING_CONFIRM";
        case DropTestState::POST_LANDING: return "POST_LANDING";
        case DropTestState::TEST_COMPLETE: return "TEST_COMPLETE";
        case DropTestState::TEST_ABORTED: return "TEST_ABORTED";
    }
    return "UNKNOWN";
}

enum DropTestEvent : uint16_t {
    DROP_EVENT_NONE = 0,
    DROP_EVENT_RELEASE_ONSET = 1U << 0,
    DROP_EVENT_RELEASE_REJECTED = 1U << 1,
    DROP_EVENT_RELEASE_CONFIRMED = 1U << 2,
    DROP_EVENT_CANOPY_SIGNATURE = 1U << 3,
    DROP_EVENT_STABLE_DESCENT = 1U << 4,
    DROP_EVENT_LANDING_CANDIDATE = 1U << 5,
    DROP_EVENT_LANDING_REJECTED = 1U << 6,
    DROP_EVENT_LANDING_CONFIRMED = 1U << 7,
    DROP_EVENT_COMPLETE = 1U << 8,
    DROP_EVENT_TIMEOUT = 1U << 9,
    DROP_EVENT_ABORTED = 1U << 10
};

struct DropTestConfig {
    // Every numeric motion threshold below REQUIRES EXPERIMENTAL CALIBRATION.
    float release_speed_mps = -1.0f;
    float release_altitude_loss_m = 1.5f;
    uint32_t release_confirm_ms = 400;

    // A change in descent rate or a linear-acceleration transient may mark an
    // inflation/load event.  It is intentionally called "suspected" until
    // corroborated by test video and post-test inspection.
    uint32_t canopy_observation_delay_ms = 150;
    float canopy_deceleration_delta_mps = 1.5f;
    float canopy_accel_signature_mps2 = 3.0f;

    uint32_t stable_descent_window_ms = 1000;
    float stable_descent_max_vs_range_mps = 1.5f;
    float stable_descent_max_angular_rate_dps = 15.0f;
    float stable_descent_min_down_speed_mps = 0.25f;

    float landing_max_vertical_speed_mps = 0.25f;
    float landing_max_angular_rate_dps = 8.0f;
    float landing_max_ground_speed_mps = 1.0f;
    float landing_max_altitude_span_m = 0.75f;
    uint32_t landing_confirm_ms = 5000;

    uint32_t post_landing_record_ms = 10000;
    uint32_t max_duration_ms = 10UL * 60UL * 1000UL;
};

struct DropTestInput {
    uint32_t now_ms = 0;
    float altitude_agl_m = 0.0f;
    float vertical_speed_mps = 0.0f;
    float vertical_accel_mps2 = 0.0f;
    float angular_rate_dps = 0.0f;
    float ground_speed_mps = 0.0f;
    bool barometer_valid = false;
    bool imu_valid = false;
    bool gps_valid = false;
};

struct DropTestOutput {
    DropTestState state = DropTestState::IDLE;
    uint16_t events = DROP_EVENT_NONE;
    bool recording = false;
    bool neutral_lock = true;
    bool close_log = false;
    bool canopy_signature_observed = false;
    bool stable_descent_observed = false;
    uint32_t armed_ms = 0;
    uint32_t release_onset_ms = 0;
    uint32_t release_confirm_ms = 0;
    uint32_t canopy_signature_ms = 0;
    uint32_t stable_descent_ms = 0;
    uint32_t landing_candidate_ms = 0;
    uint32_t landing_confirm_ms = 0;
    uint32_t log_closed_ms = 0;
};

class DropTestController {
public:
    explicit DropTestController(const DropTestConfig& config = DropTestConfig());

    DropTestOutput arm(uint32_t now_ms, float baseline_altitude_m);
    DropTestOutput abort(uint32_t now_ms);
    DropTestOutput step(const DropTestInput& input);
    const DropTestOutput& output() const { return output_; }

private:
    DropTestConfig config_;
    DropTestOutput output_;
    float arm_altitude_m_ = 0.0f;
    float fastest_descent_mps_ = 0.0f;
    float stable_min_vs_mps_ = 0.0f;
    float stable_max_vs_mps_ = 0.0f;
    float landing_min_altitude_m_ = 0.0f;
    float landing_max_altitude_m_ = 0.0f;
    uint32_t release_candidate_since_ms_ = 0;
    uint32_t stable_window_start_ms_ = 0;
    uint32_t landing_candidate_since_ms_ = 0;
    uint32_t post_landing_until_ms_ = 0;
    DropTestState pre_landing_state_ = DropTestState::DROP_CONFIRMED;

    static bool elapsed(uint32_t now, uint32_t since, uint32_t duration);
    void resetStableWindow(const DropTestInput& input);
    bool isPostReleaseState() const;
};

} // namespace logic
