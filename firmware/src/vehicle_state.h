// ============================================================================
// PHOENIX RECOVERY — Vehicle State (central data structure).
// Matches spec §35. Single shared object between all modules.
// ============================================================================
#pragma once

#include <cstdint>
#include "logic/state_machine.h"

namespace phoenix {

// ---------------------------------------------------------------------------
// VehicleState — all flight data in one place
// ---------------------------------------------------------------------------
struct VehicleState {
    // Timestamp
    uint32_t timestamp_ms = 0;
    uint32_t loop_dt_ms = 0;

    // Flight state machine
    logic::FlightState flight_state = logic::FlightState::BOOT;
    logic::GuidanceMode guidance_mode = logic::GuidanceMode::MODE_DISABLED;
    logic::FailCode failure_code = logic::FailCode::FAIL_NONE;

    // GPS
    double latitude = 0.0;
    double longitude = 0.0;
    float gps_altitude_m = 0.0f;
    float ground_speed_mps = 0.0f;
    float gps_course_deg = 0.0f;
    float hdop = 0.0f;
    int satellite_count = 0;
    bool gps_valid = false;
    bool gps_nmea_active = false;
    uint32_t gps_last_valid_ms = 0;
    uint32_t gps_nmea_age_ms = 0;

    // Barometer / altitude
    float barometric_altitude_m = 0.0f;
    float barometric_pressure_hpa = 0.0f;
    float barometric_temperature_c = 0.0f;
    float altitude_agl_m = 0.0f;
    float max_altitude_agl_m = 0.0f;
    float vertical_speed_mps = 0.0f;
    float vertical_accel_mps2 = 0.0f;
    bool barometer_valid = false;
    uint32_t baro_last_valid_ms = 0;

    // IMU
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float gyro_x_dps = 0.0f;
    float gyro_y_dps = 0.0f;
    float gyro_z_dps = 0.0f;
    float angular_rate_dps = 0.0f;
    bool imu_valid = false;
    uint32_t imu_last_valid_ms = 0;

    // Navigation / guidance
    double target_latitude = 0.0;
    double target_longitude = 0.0;
    float target_bearing_deg = 0.0f;
    float heading_error_deg = 0.0f;
    float distance_to_target_m = 0.0f;

    // Servo commands
    float left_servo_command = 0.0f;   // [-1, +1], fraction of brake travel
    float right_servo_command = 0.0f;  // [-1, +1]
    float requested_left_servo_command = 0.0f;
    float requested_right_servo_command = 0.0f;
    float left_servo_us = 1500.0f;     // actual pulse width (μs)
    float right_servo_us = 1500.0f;    // actual pulse width (μs)
    float left_servo_angle_deg = 90.0f;  // estimated shaft angle from calibrated pulse range
    float right_servo_angle_deg = 90.0f;
    float left_servo_turn_deg = 0.0f;    // estimated signed deflection from neutral
    float right_servo_turn_deg = 0.0f;

    // System health
    float loop_rate_hz = 0.0f;
    uint32_t free_heap_bytes = 0;
    uint32_t min_free_heap_bytes = 0;

    // Data ages (for stale detection)
    uint32_t gps_age_ms = 0;
    uint32_t baro_age_ms = 0;
    uint32_t imu_age_ms = 0;

    // Telemetry
    int8_t lora_rssi = 0;
    float lora_snr = 0.0f;
    uint32_t telemetry_packets_sent = 0;
    uint32_t telemetry_packets_lost = 0;
    uint32_t telemetry_sequence = 0;
    uint32_t telemetry_ack_count = 0;

    // LoRa remote-control supervision. A second LoRa can request descent
    // commands, but the flight coordinator still decides whether they are safe.
    bool lora_remote_enabled = false;
    bool lora_remote_link_active = false;
    bool lora_remote_command_allowed = false;
    bool lora_remote_manual_active = false;
    uint32_t lora_remote_last_rx_ms = 0;
    uint32_t lora_remote_command_age_ms = 0;
    uint16_t lora_remote_sequence = 0;
    uint32_t lora_remote_accepted_count = 0;
    uint32_t lora_remote_rejected_count = 0;
    float lora_remote_servo1_command = 0.0f;
    float lora_remote_servo2_command = 0.0f;

    // Debug / event log indices
    uint16_t event_log_head = 0;
    uint16_t event_log_tail = 0;

    // Timing
    uint32_t loop_time_us = 0;
    uint32_t armed_ms = 0;
    uint32_t launch_ms = 0;
    uint32_t stabilization_start_ms = 0;
    uint32_t apogee_ms = 0;
    uint32_t deployment_wait_start_ms = 0;
    uint32_t guidance_start_ms = 0;
    uint32_t final_approach_start_ms = 0;
    uint32_t flare_start_ms = 0;
    uint32_t landed_ms = 0;
    uint32_t failsafe_ms = 0;
    uint32_t state_entry_ms = 0;

    // Synchronized logging metadata. Voltage fields remain invalid unless the
    // corresponding ADC divider is physically installed and configured.
    uint32_t configuration_version = 0;
    float battery_voltage_v = 0.0f;
    float servo_rail_voltage_v = 0.0f;
    bool battery_voltage_valid = false;
    bool servo_rail_voltage_valid = false;
    bool target_valid = false;
    bool guidance_ready = false;
    bool gps_fallback_active = false;
    bool imu_fallback_active = false;
    bool barometer_fallback_active = false;
    bool degraded_guidance = false;
    bool launch_readiness_ok = false;
    bool preflight_launch_warning = false;

    // Sensor update times (for staleness detection)
    uint32_t imu_last_update_ms = 0;
    uint32_t baro_last_update_ms = 0;
    uint32_t gps_last_update_ms = 0;
    uint32_t servo_last_update_ms = 0;

    // Status flags
    bool armed = false;
    bool gps_ap_active = false;
    bool wifi_ap_active = false;
    uint8_t wifi_client_count = 0;

    // Servo positions
    uint16_t left_servo_pos = 1500;
    uint16_t right_servo_pos = 1500;
};

} // namespace phoenix
