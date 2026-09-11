// ============================================================================
// PHOENIX RECOVERY — State Estimator (altitude filter, vertical speed, heading).
// Hardware-free logic that processes sensor data into fused estimates.
// ============================================================================
#pragma once

#include "config.h"
#include "vehicle_state.h"
#include "logic/filters.h"
#include "logic/nav_math.h"

namespace estimation {

class StateEstimator {
public:
    StateEstimator();
    ~StateEstimator() = default;

    // Initialize estimator
    void begin();

    // Update with latest vehicle state - call at estimator rate
    void update(phoenix::VehicleState& state, float dt_s);

    // Get estimated vertical speed (m/s, +up)
    float getVerticalSpeed() const { return vs_estimator_.verticalSpeed(); }

    // Get filtered altitude (m AGL)
    float getFilteredAltitude() const { return vs_estimator_.altitude(); }

    // Reset with new ground reference (call after ground pressure calibration)
    void resetGroundReference(float altitude_agl_m = 0.0f);

private:
    logic::VerticalSpeedEstimator vs_estimator_;
    logic::MovingAverage<float, cfg::ALT_MA_WINDOW> alt_ma_;
    logic::LowPassFilter<float> vs_filter_;
    logic::LowPassFilter<float> course_filter_;

    float last_altitude_ = 0.0f;
    bool initialized_ = false;
};

} // namespace estimation