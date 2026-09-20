// ============================================================================
// PHOENIX RECOVERY — Telemetry Packet (binary, CRC, versioned).
// Downlink format for LoRa telemetry.
// ============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include "vehicle_state.h"
#include "logic/state_machine.h"

namespace telemetry {

#pragma pack(push, 1)

// ---------------------------------------------------------------------------
// Telemetry packet v1 (fixed layout, 16-bit aligned)
// ---------------------------------------------------------------------------
struct TelemetryPacketV1 {
    // Header
    uint8_t  magic[2] = {0x50, 0x52};  // 'P','R' - Phoenix Recovery
    uint8_t  version = 2;
    uint8_t  sequence = 0;

    // Timestamp
    uint32_t timestamp_ms = 0;

    // Flight state
    uint8_t  flight_state = 0;    // logic::FlightState
    uint8_t  guidance_mode = 0;   // logic::GuidanceMode
    uint8_t  failsafe_code = 0;   // logic::FailCode

    // GPS
    int32_t  latitude_deg7 = 0;    // degrees * 1e7
    int32_t  longitude_deg7 = 0;   // degrees * 1e7
    int16_t  gps_altitude_m = 0;   // meters * 1 (int16: +/-32km)
    int16_t  ground_speed_mps = 0; // m/s * 100
    int16_t  gps_course_deg = 0;   // degrees * 100
    int16_t  hdop_x100 = 0;        // HDOP * 100
    uint8_t  satellites = 0;
    uint8_t  gps_valid = 0;

    // Altitude / vertical
    int16_t  altitude_agl_m = 0;       // meters * 10
    int16_t  vertical_speed_mps = 0;   // m/s * 100
    int16_t  vertical_accel_mps2 = 0;  // m/s^2 * 100

    // Attitude / canopy stability
    int16_t  roll_deg = 0;              // degrees * 100
    int16_t  pitch_deg = 0;             // degrees * 100
    int16_t  yaw_deg = 0;               // degrees * 100
    int16_t  angular_rate_dps = 0;      // degrees/s * 10

    // Navigation
    int32_t  target_latitude_deg7 = 0;
    int32_t  target_longitude_deg7 = 0;
    int16_t  distance_to_target_m = 0;
    int16_t  target_bearing_deg = 0;    // degrees * 100
    int16_t  heading_error_deg = 0;     // degrees * 100

    // Servos
    int16_t  left_servo_us = 1500;   // microseconds
    int16_t  right_servo_us = 1500;  // microseconds
    int16_t  left_servo_cmd = 0;     // * 10000
    int16_t  right_servo_cmd = 0;    // * 10000

    // Sensor health
    uint8_t  imu_valid = 0;
    uint8_t  baro_valid = 0;
    uint8_t  gps_fix_valid = 0;

    // System
    int16_t  loop_rate_hz = 0;       // * 10
    uint16_t free_heap_kb = 0;       // KB
    uint16_t lora_rssi = 0;
    int16_t  lora_snr_x10 = 0;       // dB * 10

    // Inert drop-test recording state
    uint8_t  drop_test_state = 0;     // phoenix::DropTestState
    uint8_t  drop_test_flags = 0;     // bit0 recording, bit1 neutral lock
    uint16_t drop_test_id = 0;
    uint32_t drop_test_armed_ms = 0;
    uint32_t drop_test_release_ms = 0;
    uint32_t drop_test_landing_ms = 0;

    // CRC (CCITT-16)
    uint16_t crc = 0;
};

#pragma pack(pop)

static_assert(sizeof(TelemetryPacketV1) % 2 == 0, "Packet must be 16-bit aligned");
static_assert(sizeof(TelemetryPacketV1) <= 250, "Packet must fit in LoRa payload");

// ---------------------------------------------------------------------------
// CRC-16 CCITT
// ---------------------------------------------------------------------------
uint16_t crc16_ccitt(const uint8_t* data, size_t len);

// ---------------------------------------------------------------------------
// Encode VehicleState into TelemetryPacketV1
// ---------------------------------------------------------------------------
void encodeTelemetry(const phoenix::VehicleState& state, TelemetryPacketV1& packet, uint8_t sequence);

} // namespace telemetry
