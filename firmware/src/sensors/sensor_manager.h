// ============================================================================
// PHOENIX RECOVERY — Sensor Manager (combines all sensors + plausibility).
// ============================================================================
#pragma once

#include "config.h"
#include "vehicle_state.h"
#include "sensors/imu.h"
#include "sensors/barometer.h"
#include "sensors/gps.h"
#include "logic/data_quality.h"
#include "logic/filters.h"
#include "logic/nav_math.h"

namespace sensors {

// Sensor configuration
struct SensorConfig {
    bool imu_enabled = true;
    bool baro_enabled = true;
    bool gps_enabled = true;
};

class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    // Initialize all sensors
    bool begin();
    bool begin(const SensorConfig& config);

    // Update all sensors - call every loop iteration
    void update();
    void update(phoenix::VehicleState& state);

    // Get latest vehicle state
    phoenix::VehicleState& getVehicleState() { return state_; }
    const phoenix::VehicleState& getVehicleState() const { return state_; }

    // Check overall health
    bool isHealthy() const;

    // Force ground pressure calibration (call at PAD_SAFE entry)
    bool calibrateGroundPressure();

    // Get individual sensor pointers for direct access
    sensors::IMU* getIMU() { return &imu_; }
    sensors::Barometer* getBarometer() { return &baro_; }
    sensors::GPS* getGPS() { return &gps_; }

private:
    sensors::IMU imu_;
    sensors::Barometer baro_;
    sensors::GPS gps_;
    phoenix::VehicleState state_;

    logic::VerticalSpeedEstimator vs_estimator_;
    logic::MovingAverage<float, cfg::ALT_MA_WINDOW> alt_ma_;
    logic::LowPassFilter<float> vs_filter_;

    uint32_t last_update_ms_ = 0;
    uint32_t last_gps_valid_ms_ = 0;
    uint32_t last_baro_valid_ms_ = 0;
    uint32_t last_imu_valid_ms_ = 0;

    bool ground_calibrated_ = false;
    uint32_t cal_start_ms_ = 0;

#if SIMULATION_MODE
    // Simulation state
    uint32_t sim_time_ms_ = 0;
    float sim_altitude_ = 0.0f;
    float sim_vertical_speed_ = 0.0f;
    int sim_phase_ = 0;  // 0=pad, 1=ascent, 2=apogee, 3=descent, 4=flare, 5=landed
    double sim_latitude_ = 0.0;
    double sim_longitude_ = 0.0;
    float sim_heading_ = 0.0f;
    float sim_roll_ = 0.0f;
    float sim_pitch_ = 0.0f;
    float sim_yaw_ = 0.0f;
    uint32_t sim_phase_start_ms_ = 0;
    bool sim_ground_calibrated_ = false;
    float sim_ground_pressure_ = 1013.25f;

    void runSimulation(uint32_t now);
    void generateSimIMU(uint32_t now);
    void generateSimBarometer(uint32_t now);
    void generateSimGPS(uint32_t now);
    void advanceSimPhase(uint32_t now, float dt);
#endif

    void updateVehicleState();
    void computeVerticalSpeed();
    void computeNavigation();
    void checkSensorHealth();
};

} // namespace sensors
