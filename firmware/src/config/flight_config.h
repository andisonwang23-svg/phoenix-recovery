// ============================================================================
// PHOENIX RECOVERY — flight configuration
// State-machine timing, launch/apogee/landing thresholds, update rates.
// Every value here is a placeholder for bench calibration unless marked
// REQUIRES EXPERIMENTAL CALIBRATION.
// ============================================================================
#pragma once

#include <cstdint>

namespace cfg {

// ---------------------------------------------------------------------------
// Phase timing (ms)
// ---------------------------------------------------------------------------
constexpr uint32_t LAUNCH_CONFIRM_MS            = 400;
constexpr uint32_t APOGEE_CONFIRMATION_TIME_MS  = 700;
constexpr uint32_t DEPLOYMENT_WAIT_MS           = 1500; // motor ejection time
constexpr uint32_t PARAFOIL_STABILIZATION_MS    = 3000; // canopy inflation + settle
constexpr uint32_t LANDED_CONFIRM_MS            = 4000;
constexpr uint32_t STABILIZATION_TIMEOUT_MS     = 15000; // abandon wait -> failsafe/neutral

// ---------------------------------------------------------------------------
// Launch detection (all conditions must persist for LAUNCH_CONFIRM_MS)
// ---------------------------------------------------------------------------
constexpr float    LAUNCH_ACCEL_MPS2        = 8.0f;   // sustained vertical accel
constexpr float    LAUNCH_VS_MPS            = 5.0f;   // rising vertical speed
constexpr float    LAUNCH_ALT_DELTA_M       = 10.0f;  // altitude gained vs ref

// ---------------------------------------------------------------------------
// Apogee detection
// ---------------------------------------------------------------------------
constexpr float    APOGEE_VS_THRESHOLD_MPS  = 1.5f;   // |vs| below this near apogee
constexpr float    APOGEE_ALT_FLAT_M        = 2.0f;   // max climb across window
constexpr float    MINIMUM_LAUNCH_ALTITUDE_M = 8.0f;  // reject nonsense apogee claims

// ---------------------------------------------------------------------------
// Landing detection
// ---------------------------------------------------------------------------
constexpr float    LANDED_ALT_CHANGE_M      = 1.0f;
constexpr float    LANDED_VS_MPS            = 1.0f;
constexpr float    LANDED_GND_SPEED_MPS     = 2.0f;
constexpr float    LANDED_ACCEL_JITTER_G    = 0.15f;

// ---------------------------------------------------------------------------
// Guidance / control timing
// ---------------------------------------------------------------------------
constexpr int      RATE_IMU_HZ       = 50;
constexpr int      RATE_BARO_HZ      = 25;
constexpr int      RATE_ESTIMATOR_HZ = 50;
constexpr int      RATE_GUIDANCE_HZ  = 15;
constexpr int      RATE_HEALTH_HZ    = 8;
constexpr int      RATE_TELEMETRY_HZ = 5; // default; per-state map below

// Telemetry rate per flight state (Hz)
constexpr int      TLM_RATE_PAD_HZ     = 1;
constexpr int      TLM_RATE_FLIGHT_HZ  = 5;   // ascent / guided
constexpr int      TLM_RATE_LANDED_HZ  = 1;

// ---------------------------------------------------------------------------
// Stale-data / health timeouts
// ---------------------------------------------------------------------------
constexpr uint32_t SENSOR_TIMEOUT_MS          = 200;  // IMU/baro report deadline
constexpr uint32_t GPS_LOSS_TIMEOUT_MS        = 5000; // in guided states only
constexpr uint32_t HEALTH_WATCHDOG_LOOP_MS    = 250;  // main loop must tick < this
constexpr uint32_t WDT_TIMEOUT_MS             = 1000;
constexpr bool     ENABLE_HARDWARE_WDT        = true;

// ---------------------------------------------------------------------------
// Estimator / filters
// ---------------------------------------------------------------------------
constexpr float    ALT_FILTER_TAU_S      = 0.3f;
constexpr float    VS_FILTER_GAIN_ALT    = 0.20f;  // complementary-estimator gains
constexpr float    VS_FILTER_GAIN_VS     = 0.10f;
constexpr uint32_t BARO_CAL_SAMPLES      = 50;     // launch-ground pressure average
constexpr uint8_t  ALT_MA_WINDOW         = 8;      // moving-average window (samples)
constexpr float    COURSE_SPEED_THRESHOLD_MPS = 3.0f; // min GPS speed to trust course

// ---------------------------------------------------------------------------
// Data-quality gates (a sample outside these bounds is rejected)
// ---------------------------------------------------------------------------
constexpr float    GPS_MAX_SPEED_MPS     = 120.0f;  // absurd ground speed
constexpr float    GPS_MAX_JUMP_M        = 50.0f;   // position jump in one fix
constexpr float    BARO_MAX_ALT_STEP_M   = 30.0f;   // altitude jump in one sample
constexpr float    QUAT_MIN_NORM         = 0.85f;   // IMU quaternion sanity
constexpr float    QUAT_MAX_NORM         = 1.15f;

} // namespace cfg
