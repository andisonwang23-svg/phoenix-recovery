// ============================================================================
// PHOENIX RECOVERY — Sensor Manager Implementation.
// ============================================================================
#include "sensor_manager.h"

#if SIMULATION_MODE
#include <cmath>
#endif

namespace sensors {

SensorManager::SensorManager()
    : vs_estimator_(cfg::VS_FILTER_GAIN_ALT, cfg::VS_FILTER_GAIN_VS),
      vs_filter_(0.1f) {}

SensorManager::~SensorManager() = default;

bool SensorManager::begin() {
    // Initialize I2C with custom pins (GPIO47/48) - NOT default pins!
    Wire.begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL);
    Wire.setClock(cfg::I2C_FREQ_HZ);

    // Initialize sensors
    bool imu_ok = imu_.begin(Wire);
    bool baro_ok = baro_.begin(Wire);
    bool gps_ok = gps_.begin(Serial2);

    // Initialize estimator
    vs_estimator_.reset(0.0f, 0.0f);
    alt_ma_.reset();
    vs_filter_.reset(0.0f);

    state_.imu_valid = imu_ok;
    state_.barometer_valid = baro_ok;
    state_.gps_valid = false; // UART initialization is not a satellite fix.

    // GPS may not have fix yet. The barometer is allowed to be absent during
    // bench bring-up; flight-state logic still refuses real guidance without
    // enough valid sensors.
    return imu_ok;
}

bool SensorManager::begin(const SensorConfig& config) {
    // Initialize I2C with custom pins (GPIO47/48) - NOT default pins!
    Wire.begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL);
    Wire.setClock(cfg::I2C_FREQ_HZ);

    // Initialize sensors based on config
    bool imu_ok = true, baro_ok = true, gps_ok = true;

    if (config.imu_enabled) {
        imu_ok = imu_.begin(Wire);
    }
    if (config.baro_enabled) {
        baro_ok = baro_.begin(Wire);
    }
    if (config.gps_enabled) {
        gps_ok = gps_.begin(Serial2);
    }

    // Initialize estimator
    vs_estimator_.reset(0.0f, 0.0f);
    alt_ma_.reset();
    vs_filter_.reset(0.0f);

    state_.imu_valid = imu_ok;
    state_.barometer_valid = baro_ok;
    state_.gps_valid = false; // UART initialization is not a satellite fix.

    // GPS may not have fix yet. The barometer is optional for IMU-only bench
    // bring-up on a new board; flight-state logic still prevents real flight.
    return (imu_ok || !config.imu_enabled);
}

void SensorManager::update(phoenix::VehicleState& state) {
    // Navigation depends on configuration owned by the shared vehicle state.
    // Never replace the full VehicleState here: doing so erases guidance,
    // servo, health, telemetry, and state-machine outputs from other modules.
    state_.target_latitude = state.target_latitude;
    state_.target_longitude = state.target_longitude;
    update();

    state.timestamp_ms = state_.timestamp_ms;
    state.latitude = state_.latitude;
    state.longitude = state_.longitude;
    state.gps_altitude_m = state_.gps_altitude_m;
    state.ground_speed_mps = state_.ground_speed_mps;
    state.gps_course_deg = state_.gps_course_deg;
    state.hdop = state_.hdop;
    state.satellite_count = state_.satellite_count;
    state.gps_valid = state_.gps_valid;
    state.gps_nmea_active = state_.gps_nmea_active;
    state.gps_last_valid_ms = state_.gps_last_valid_ms;
    state.gps_nmea_age_ms = state_.gps_nmea_age_ms;
    state.barometric_altitude_m = state_.barometric_altitude_m;
    state.barometric_pressure_hpa = state_.barometric_pressure_hpa;
    state.barometric_temperature_c = state_.barometric_temperature_c;
    state.altitude_agl_m = state_.altitude_agl_m;
    state.vertical_speed_mps = state_.vertical_speed_mps;
    state.vertical_accel_mps2 = state_.vertical_accel_mps2;
    state.barometer_valid = state_.barometer_valid;
    state.baro_last_valid_ms = state_.baro_last_valid_ms;
    state.roll_deg = state_.roll_deg;
    state.pitch_deg = state_.pitch_deg;
    state.yaw_deg = state_.yaw_deg;
    state.gyro_x_dps = state_.gyro_x_dps;
    state.gyro_y_dps = state_.gyro_y_dps;
    state.gyro_z_dps = state_.gyro_z_dps;
    state.angular_rate_dps = state_.angular_rate_dps;
    state.imu_valid = state_.imu_valid;
    state.imu_last_valid_ms = state_.imu_last_valid_ms;
    state.distance_to_target_m = state_.distance_to_target_m;
    state.target_bearing_deg = state_.target_bearing_deg;
    state.heading_error_deg = state_.heading_error_deg;
    state.gps_age_ms = state_.gps_age_ms;
    state.baro_age_ms = state_.baro_age_ms;
    state.imu_age_ms = state_.imu_age_ms;
    state.imu_last_update_ms = state_.imu_last_update_ms;
    state.baro_last_update_ms = state_.baro_last_update_ms;
    state.gps_last_update_ms = state_.gps_last_update_ms;
}

void SensorManager::update() {
    uint32_t now = millis();
    state_.timestamp_ms = now;

#if SIMULATION_MODE
    // Run simulation to generate synthetic sensor data
    runSimulation(now);
#else
    // Update all sensors
    bool imu_new = imu_.update();
    bool baro_new = baro_.update();
    bool gps_new = gps_.update();

    // Freshness is authoritative. Drivers may retain the last plausible value
    // through a short read error, but only a newly accepted sample refreshes
    // the timestamps below.
    state_.imu_valid = imu_.getData().valid;
    state_.barometer_valid = baro_.getData().valid;
    state_.gps_valid = gps_.getData().valid;

    // Track validity timestamps
    if (imu_new && imu_.getData().valid) {
        last_imu_valid_ms_ = now;
        state_.imu_last_valid_ms = now;
        state_.imu_last_update_ms = now;
        state_.imu_valid = true;
    }
    if (baro_new && baro_.getData().valid) {
        last_baro_valid_ms_ = now;
        state_.baro_last_valid_ms = now;
        state_.baro_last_update_ms = now;
        state_.barometer_valid = true;
    }
    if (gps_new && gps_.getData().valid) {
        last_gps_valid_ms_ = now;
        state_.gps_last_valid_ms = now;
        state_.gps_last_update_ms = now;
        state_.gps_valid = true;
    }
#endif

    // Compute data ages
    state_.imu_age_ms = now - last_imu_valid_ms_;
    state_.baro_age_ms = now - last_baro_valid_ms_;
    state_.gps_age_ms = now - last_gps_valid_ms_;
    const auto& latest_gps = gps_.getData();
    state_.gps_nmea_age_ms = latest_gps.last_nmea_ms > 0 ? now - latest_gps.last_nmea_ms : UINT32_MAX;
    state_.gps_nmea_active = latest_gps.last_nmea_ms > 0 && state_.gps_nmea_age_ms < 3000;

    // Update vehicle state from sensors
    updateVehicleState();

    // Compute vertical speed
    computeVerticalSpeed();

    // Compute navigation (bearing, distance to target)
    computeNavigation();

    // Check sensor health
    checkSensorHealth();
}

void SensorManager::updateVehicleState() {
    // IMU data
    auto imu_data = imu_.getData();
    state_.roll_deg = imu_data.roll_deg;
    state_.pitch_deg = imu_data.pitch_deg;
    state_.yaw_deg = imu_data.yaw_deg;
    state_.gyro_x_dps = imu_data.gyro_x * 180.0f / M_PI;
    state_.gyro_y_dps = imu_data.gyro_y * 180.0f / M_PI;
    state_.gyro_z_dps = imu_data.gyro_z * 180.0f / M_PI;
    state_.angular_rate_dps = std::sqrt(
        state_.gyro_x_dps * state_.gyro_x_dps +
        state_.gyro_y_dps * state_.gyro_y_dps +
        state_.gyro_z_dps * state_.gyro_z_dps);
    state_.vertical_accel_mps2 = imu_data.linear_accel_z;

    // Barometer data
    auto baro_data = baro_.getData();
    state_.barometric_altitude_m = baro_data.altitude_m;
    state_.barometric_pressure_hpa = baro_data.pressure_hpa;
    state_.barometric_temperature_c = baro_data.temperature_c;

    // Moving average on altitude
    alt_ma_.add(state_.barometric_altitude_m);
    state_.altitude_agl_m = alt_ma_.value();

    // GPS data
    auto gps_data = gps_.getData();
    state_.latitude = gps_data.latitude;
    state_.longitude = gps_data.longitude;
    state_.gps_altitude_m = gps_data.altitude_m;
    state_.ground_speed_mps = gps_data.speed_mps;
    state_.gps_course_deg = gps_data.course_deg;
    state_.hdop = gps_data.hdop;
    state_.satellite_count = gps_data.satellites;
    state_.gps_nmea_age_ms = gps_data.last_nmea_ms > 0 ? millis() - gps_data.last_nmea_ms : UINT32_MAX;
    state_.gps_nmea_active = gps_data.last_nmea_ms > 0 && state_.gps_nmea_age_ms < 3000;
}

void SensorManager::computeVerticalSpeed() {
    float dt = 0.02f; // 50 Hz estimator rate
    state_.vertical_speed_mps = vs_estimator_.update(state_.altitude_agl_m, dt);

    // Additional low-pass on vertical speed for smoothing
    state_.vertical_speed_mps = vs_filter_.update(state_.vertical_speed_mps);
}

void SensorManager::computeNavigation() {
    if (!state_.gps_valid || state_.target_latitude == 0.0 || state_.target_longitude == 0.0) {
        state_.distance_to_target_m = 0.0f;
        state_.target_bearing_deg = 0.0f;
        state_.heading_error_deg = 0.0f;
        return;
    }

    using namespace logic;
    state_.distance_to_target_m = distanceMeters(
        state_.latitude, state_.longitude,
        state_.target_latitude, state_.target_longitude);

    state_.target_bearing_deg = bearingDegrees(
        state_.latitude, state_.longitude,
        state_.target_latitude, state_.target_longitude);

    if (state_.ground_speed_mps > cfg::COURSE_SPEED_THRESHOLD_MPS) {
        state_.heading_error_deg = headingErrorDegrees(
            state_.gps_course_deg, state_.target_bearing_deg);
    }
}

void SensorManager::checkSensorHealth() {
    // Check for sensor timeouts
    uint32_t now = millis();

    if (now - last_imu_valid_ms_ > cfg::SENSOR_TIMEOUT_MS) {
        state_.imu_valid = false;
    }
    if (last_baro_valid_ms_ == 0 || now - last_baro_valid_ms_ > cfg::SENSOR_TIMEOUT_MS) {
        state_.barometer_valid = false;
    }
    // GPS timeout handled by state machine (longer timeout)
}

bool SensorManager::calibrateGroundPressure() {
    if (!baro_.isAvailable()) return false;

    // Average BARO_CAL_SAMPLES samples
    const uint32_t samples = cfg::BARO_CAL_SAMPLES;
    float sum = 0.0f;
    uint32_t count = 0;

    for (uint32_t i = 0; i < samples; ++i) {
        baro_.update();
        auto data = baro_.getData();
        if (data.valid) {
            sum += data.pressure_hpa;
            count++;
        }
        delay(20); // 50 Hz
    }

    if (count == 0) return false;

    float ground_pressure = sum / count;
    baro_.setGroundPressure(ground_pressure);
    ground_calibrated_ = true;

    // Reset estimator with new ground reference
    vs_estimator_.reset(0.0f, 0.0f);
    alt_ma_.reset();
    vs_filter_.reset(0.0f);

    return true;
}

bool SensorManager::isHealthy() const {
    return state_.imu_valid && state_.barometer_valid;
}

#if SIMULATION_MODE

void SensorManager::runSimulation(uint32_t now) {
    // Initialize simulation on first run
    if (sim_time_ms_ == 0) {
        sim_time_ms_ = now;
        sim_phase_start_ms_ = now;
        // Use target coordinates if set, otherwise use a default location
        if (state_.target_latitude != 0.0 && state_.target_longitude != 0.0) {
            sim_latitude_ = state_.target_latitude - 0.001;  // Start 100m south of target
            sim_longitude_ = state_.target_longitude;
        } else {
            sim_latitude_ = 37.7749;  // San Francisco default
            sim_longitude_ = -122.4194;
        }
    }

    // Advance simulation time
    float dt = (now - sim_time_ms_) / 1000.0f;
    sim_time_ms_ = now;

    // Advance flight phase based on time and altitude
    advanceSimPhase(now, dt);

    // Generate simulated sensor data
    generateSimIMU(now);
    generateSimBarometer(now);
    generateSimGPS(now);

    // Mark all sensors as valid in simulation mode
    last_imu_valid_ms_ = now;
    last_baro_valid_ms_ = now;
    last_gps_valid_ms_ = now;
    state_.imu_valid = true;
    state_.barometer_valid = true;
    state_.gps_valid = true;
}

void SensorManager::advanceSimPhase(uint32_t now, float dt) {
    uint32_t phase_elapsed = now - sim_phase_start_ms_;

    switch (sim_phase_) {
        case 0:  // PAD_SAFE - waiting on pad
            if (phase_elapsed > 5000) {  // After 5 seconds, simulate launch
                sim_phase_ = 1;  // ASCENT
                sim_phase_start_ms_ = now;
                sim_vertical_speed_ = 0.0f;
            }
            break;
        case 1:  // ASCENT - rocket going up
            sim_vertical_speed_ += 15.0f * (phase_elapsed / 1000.0f);  // Accelerating
            if (sim_vertical_speed_ > 80.0f) sim_vertical_speed_ = 80.0f;
            sim_altitude_ += sim_vertical_speed_ * dt;  // Approximate integration
            if (sim_altitude_ > 300.0f && phase_elapsed > 8000) {  // Reached apogee altitude
                sim_phase_ = 2;  // APOGEE
                sim_phase_start_ms_ = now;
            }
            break;
        case 2:  // APOGEE - at peak, transitioning
            sim_vertical_speed_ -= 5.0f * (phase_elapsed / 1000.0f);  // Decelerating
            if (sim_vertical_speed_ < 0) sim_vertical_speed_ = 0;
            sim_altitude_ += sim_vertical_speed_ * dt;
            if (phase_elapsed > 3000) {  // Parafoil deployment wait
                sim_phase_ = 3;  // DESCENT_GUIDED
                sim_phase_start_ms_ = now;
                sim_vertical_speed_ = -8.0f;  // Descending under parafoil
            }
            break;
        case 3:  // DESCENT_GUIDED - under parafoil, guided
            // Gentle descent with some variation
            sim_vertical_speed_ = -6.0f + 2.0f * std::sin(now / 10000.0f);
            sim_altitude_ += sim_vertical_speed_ * 0.02f;

            // Simulate guided navigation toward target
            if (sim_altitude_ > 0) {
                // Calculate bearing to target
                double target_lat = (state_.target_latitude != 0.0) ? state_.target_latitude : sim_latitude_ + 0.001;
                double target_lon = (state_.target_longitude != 0.0) ? state_.target_longitude : sim_longitude_;

                // Simple navigation simulation - drift toward target
                float lat_diff = (target_lat - sim_latitude_) * 111320.0;  // meters per degree lat
                float lon_diff = (target_lon - sim_longitude_) * 111320.0 * std::cos(sim_latitude_ * M_PI / 180.0);
                float distance = std::sqrt(lat_diff * lat_diff + lon_diff * lon_diff);

                if (distance > 5.0f) {
                    float bearing = std::atan2(lon_diff, lat_diff) * 180.0f / M_PI;
                    // Turn toward target (simulate parafoil turning)
                    float heading_error = bearing - sim_heading_;
                    while (heading_error > 180.0f) heading_error -= 360.0f;
                    while (heading_error < -180.0f) heading_error += 360.0f;

                    sim_heading_ += heading_error * 0.1f;  // Slow turn rate
                    while (sim_heading_ >= 360.0f) sim_heading_ -= 360.0f;
                    while (sim_heading_ < 0.0f) sim_heading_ += 360.0f;

                    // Move toward target
                    float speed = 8.0f;  // m/s ground speed
                    sim_latitude_ += (speed * std::cos(sim_heading_ * M_PI / 180.0f) / 111320.0) * 0.02f;
                    sim_longitude_ += (speed * std::sin(sim_heading_ * M_PI / 180.0f) / (111320.0 * std::cos(sim_latitude_ * M_PI / 180.0))) * 0.02f;
                }
            }

            if (sim_altitude_ <= 30.0f) {  // Final approach
                sim_phase_ = 4;  // FINAL_APPROACH
                sim_phase_start_ms_ = now;
            }
            break;
        case 4:  // FINAL_APPROACH - close to ground
            sim_vertical_speed_ = -3.0f + 1.0f * std::sin(now / 5000.0f);
            sim_altitude_ += sim_vertical_speed_ * 0.02f;

            // Flare at low altitude
            if (sim_altitude_ <= 8.0f) {
                sim_phase_ = 5;  // FLARE
                sim_phase_start_ms_ = now;
                sim_vertical_speed_ = -1.5f;  // Gentle flare descent
            }
            break;
        case 5:  // FLARE - final flare before touchdown
            sim_vertical_speed_ = -1.0f;
            sim_altitude_ += sim_vertical_speed_ * 0.02f;
            if (sim_altitude_ <= 0.5f) {
                sim_phase_ = 6;  // LANDED
                sim_phase_start_ms_ = now;
                sim_altitude_ = 0.0f;
                sim_vertical_speed_ = 0.0f;
            }
            break;
        case 6:  // LANDED
            sim_altitude_ = 0.0f;
            sim_vertical_speed_ = 0.0f;
            break;
    }

    // Clamp altitude
    if (sim_altitude_ < 0.0f) sim_altitude_ = 0.0f;
}

void SensorManager::generateSimIMU(uint32_t now) {
    // Generate realistic IMU data based on flight phase
    auto& imu_data = imu_.getData();  // Get reference to internal data

    float dt = 0.02f;  // 50 Hz

    // Simulate attitude based on flight phase
    switch (sim_phase_) {
        case 0:  // PAD - stable on ground
            sim_roll_ = 0.5f * std::sin(now / 10000.0f);  // Slight vibration
            sim_pitch_ = 0.3f * std::cos(now / 8000.0f);
            sim_yaw_ = sim_heading_;
            break;
        case 1:  // ASCENT - vertical with slight wobble
            sim_roll_ = 2.0f * std::sin(now / 3000.0f);
            sim_pitch_ = -5.0f + 3.0f * std::cos(now / 4000.0f);  // Nose up
            sim_yaw_ = sim_heading_;
            break;
        case 2:  // APOGEE - tumbling slightly
            sim_roll_ = 10.0f * std::sin(now / 2000.0f);
            sim_pitch_ = 5.0f * std::cos(now / 2500.0f);
            sim_yaw_ = sim_heading_ + 20.0f * std::sin(now / 5000.0f);
            break;
        case 3:  // DESCENT_GUIDED - parafoil flying
            sim_roll_ = 15.0f * std::sin(now / 6000.0f);  // Banking turns
            sim_pitch_ = -3.0f + 2.0f * std::cos(now / 7000.0f);
            sim_yaw_ = sim_heading_;
            break;
        case 4:  // FINAL_APPROACH
            sim_roll_ = 10.0f * std::sin(now / 5000.0f);
            sim_pitch_ = -5.0f;  // Nose up for flare prep
            sim_yaw_ = sim_heading_;
            break;
        case 5:  // FLARE
            sim_roll_ = 5.0f * std::sin(now / 4000.0f);
            sim_pitch_ = 10.0f;  // Flare pitch up
            sim_yaw_ = sim_heading_;
            break;
        case 6:  // LANDED
            sim_roll_ = 0.0f;
            sim_pitch_ = 0.0f;
            sim_yaw_ = sim_heading_;
            break;
    }

    // Convert to quaternion (simplified)
    float cr = std::cos(sim_roll_ * M_PI / 360.0f);
    float sr = std::sin(sim_roll_ * M_PI / 360.0f);
    float cp = std::cos(sim_pitch_ * M_PI / 360.0f);
    float sp = std::sin(sim_pitch_ * M_PI / 360.0f);
    float cy = std::cos(sim_yaw_ * M_PI / 360.0f);
    float sy = std::sin(sim_yaw_ * M_PI / 360.0f);

    imu_data.quat_w = cr * cp * cy + sr * sp * sy;
    imu_data.quat_x = sr * cp * cy - cr * sp * sy;
    imu_data.quat_y = cr * sp * cy + sr * cp * sy;
    imu_data.quat_z = cr * cp * sy - sr * sp * cy;

    // Normalize
    float norm = std::sqrt(imu_data.quat_w * imu_data.quat_w +
                           imu_data.quat_x * imu_data.quat_x +
                           imu_data.quat_y * imu_data.quat_y +
                           imu_data.quat_z * imu_data.quat_z);
    imu_data.quat_w /= norm;
    imu_data.quat_x /= norm;
    imu_data.quat_y /= norm;
    imu_data.quat_z /= norm;

    // Linear acceleration (gravity removed)
    float ax = 0.1f * std::sin(now / 1000.0f);
    float ay = 0.1f * std::cos(now / 1200.0f);
    float az = 0.0f;

    switch (sim_phase_) {
        case 1: az = 12.0f; break;  // ASCENT - high acceleration
        case 2: az = 0.5f; break;   // APOGEE - near zero
        case 3: az = -0.5f; break;  // DESCENT - slight negative
        case 4: az = -0.3f; break;  // FINAL - gentle
        case 5: az = 2.0f; break;   // FLARE - deceleration
        default: az = 0.0f; break;
    }

    imu_data.linear_accel_x = ax;
    imu_data.linear_accel_y = ay;
    imu_data.linear_accel_z = az;

    // Raw acceleration (with gravity)
    float gx = ax;
    float gy = ay;
    float gz = az + 9.81f;
    // Rotate gravity by attitude (simplified)
    imu_data.accel_x = gx;
    imu_data.accel_y = gy;
    imu_data.accel_z = gz;

    // Gyro rates
    imu_data.gyro_x = (sim_phase_ >= 1 && sim_phase_ <= 5) ? 0.05f * std::sin(now / 2000.0f) : 0.0f;
    imu_data.gyro_y = (sim_phase_ >= 1 && sim_phase_ <= 5) ? 0.03f * std::cos(now / 2500.0f) : 0.0f;
    imu_data.gyro_z = (sim_phase_ == 1) ? 0.02f : ((sim_phase_ == 3) ? 0.1f * std::sin(now / 6000.0f) : 0.0f);

    // Gravity vector
    imu_data.gravity_x = 0.0f;
    imu_data.gravity_y = 0.0f;
    imu_data.gravity_z = 9.81f;

    // Euler angles
    imu_data.roll_deg = sim_roll_;
    imu_data.pitch_deg = sim_pitch_;
    imu_data.yaw_deg = sim_yaw_;

    imu_data.timestamp_ms = now;
    imu_data.valid = true;

    // Update the internal state (need to bypass private member - we'll use updateVehicleState instead)
    // The updateVehicleState reads from imu_.getData()
}

void SensorManager::generateSimBarometer(uint32_t now) {
    auto& baro_data = baro_.getData();

    // Standard atmosphere model
    // Pressure = P0 * (1 - L*h/T0)^(g*M/(R*L))
    // Simplified: pressure drops ~1 hPa per 8 meters at sea level
    float pressure = sim_ground_pressure_ - sim_altitude_ / 8.3f;

    // Add some noise
    pressure += 0.1f * std::sin(now / 5000.0f);

    // Temperature
    float temperature = 20.0f - sim_altitude_ * 0.0065f;  // Lapse rate

    baro_data.pressure_hpa = pressure;
    baro_data.temperature_c = temperature;
    baro_data.altitude_m = sim_altitude_;
    baro_data.valid = true;
    baro_data.timestamp_ms = now;
}

void SensorManager::generateSimGPS(uint32_t now) {
    auto& gps_data = gps_.getData();

    // Only update GPS at ~1 Hz
    static uint32_t last_gps_update = 0;
    if (now - last_gps_update < 1000) return;
    last_gps_update = now;

    gps_data.latitude = sim_latitude_;
    gps_data.longitude = sim_longitude_;
    gps_data.altitude_m = sim_altitude_;

    // Ground speed based on phase
    switch (sim_phase_) {
        case 0: gps_data.speed_mps = 0.0f; break;
        case 1: gps_data.speed_mps = 5.0f; break;  // Some drift during ascent
        case 2: gps_data.speed_mps = 3.0f; break;
        case 3: gps_data.speed_mps = 8.0f; break;  // Parafoil forward speed
        case 4: gps_data.speed_mps = 6.0f; break;
        case 5: gps_data.speed_mps = 2.0f; break;
        case 6: gps_data.speed_mps = 0.0f; break;
    }

    gps_data.course_deg = sim_heading_;
    gps_data.hdop = 1.2f;
    gps_data.satellites = 8;
    gps_data.fix_valid = true;
    gps_data.valid = true;
    gps_data.timestamp_ms = now;
}

#endif // SIMULATION_MODE

} // namespace sensors
