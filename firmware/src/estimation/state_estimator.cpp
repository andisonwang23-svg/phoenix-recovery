// ============================================================================
// PHOENIX RECOVERY — State Estimator Implementation.
// ============================================================================
#include "state_estimator.h"

namespace estimation {

StateEstimator::StateEstimator()
    : vs_estimator_(cfg::VS_FILTER_GAIN_ALT, cfg::VS_FILTER_GAIN_VS),
      vs_filter_(0.1f),
      course_filter_(0.05f) {}

void StateEstimator::begin() {
    vs_estimator_.reset(0.0f, 0.0f);
    alt_ma_.reset();
    vs_filter_.reset(0.0f);
    course_filter_.reset(0.0f);
    initialized_ = true;
}

void StateEstimator::update(phoenix::VehicleState& state, float dt_s) {
    if (!initialized_) return;

    // Moving average on barometric altitude
    alt_ma_.add(state.altitude_agl_m);
    float filtered_alt = alt_ma_.value();

    // Vertical speed estimation (complementary filter)
    float vs = vs_estimator_.update(filtered_alt, dt_s);
    vs = vs_filter_.update(vs);

    state.vertical_speed_mps = vs;
    state.altitude_agl_m = vs_estimator_.altitude(); // use estimator's filtered altitude

    // Course filtering (when GPS speed is sufficient)
    if (state.gps_valid && state.ground_speed_mps > cfg::COURSE_SPEED_THRESHOLD_MPS) {
        float filtered_course = course_filter_.update(state.gps_course_deg);
        // Note: we don't overwrite state.gps_course_deg - keep raw for reference
        // Could add a filtered_course field to VehicleState if needed
    }
}

void StateEstimator::resetGroundReference(float altitude_agl_m) {
    vs_estimator_.reset(altitude_agl_m, 0.0f);
    alt_ma_.reset();
    alt_ma_.add(altitude_agl_m);
    vs_filter_.reset(0.0f);
}

} // namespace estimation