// ============================================================================
// PHOENIX RECOVERY — Telemetry Packet Implementation.
// ============================================================================
#include "telemetry_packet.h"

namespace telemetry {

// CRC-16 CCITT (polynomial 0x1021, init 0xFFFF)
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

void encodeTelemetry(const phoenix::VehicleState& state, TelemetryPacketV1& packet, uint8_t sequence) {
    packet.magic[0] = 0x50;
    packet.magic[1] = 0x52;
    packet.version = 2;
    packet.sequence = sequence;

    packet.timestamp_ms = state.timestamp_ms;

    packet.flight_state = static_cast<uint8_t>(state.flight_state);
    packet.guidance_mode = static_cast<uint8_t>(state.guidance_mode);
    packet.failsafe_code = static_cast<uint8_t>(state.failure_code);

    // GPS (degrees * 1e7)
    packet.latitude_deg7 = static_cast<int32_t>(state.latitude * 1e7);
    packet.longitude_deg7 = static_cast<int32_t>(state.longitude * 1e7);
    packet.gps_altitude_m = static_cast<int16_t>(state.gps_altitude_m);
    packet.ground_speed_mps = static_cast<int16_t>(state.ground_speed_mps * 100);
    packet.gps_course_deg = static_cast<int16_t>(state.gps_course_deg * 100);
    packet.hdop_x100 = static_cast<int16_t>(state.hdop * 100);
    packet.satellites = static_cast<uint8_t>(state.satellite_count);
    packet.gps_valid = state.gps_valid ? 1 : 0;

    // Altitude / vertical
    packet.altitude_agl_m = static_cast<int16_t>(state.altitude_agl_m * 10);
    packet.vertical_speed_mps = static_cast<int16_t>(state.vertical_speed_mps * 100);
    packet.vertical_accel_mps2 = static_cast<int16_t>(state.vertical_accel_mps2 * 100);
    packet.roll_deg = static_cast<int16_t>(state.roll_deg * 100);
    packet.pitch_deg = static_cast<int16_t>(state.pitch_deg * 100);
    packet.yaw_deg = static_cast<int16_t>(state.yaw_deg * 100);
    packet.angular_rate_dps = static_cast<int16_t>(state.angular_rate_dps * 10);

    // Navigation
    packet.target_latitude_deg7 = static_cast<int32_t>(state.target_latitude * 1e7);
    packet.target_longitude_deg7 = static_cast<int32_t>(state.target_longitude * 1e7);
    packet.distance_to_target_m = static_cast<int16_t>(state.distance_to_target_m);
    packet.target_bearing_deg = static_cast<int16_t>(state.target_bearing_deg * 100);
    packet.heading_error_deg = static_cast<int16_t>(state.heading_error_deg * 100);

    // Servos
    packet.left_servo_us = static_cast<int16_t>(state.left_servo_us);
    packet.right_servo_us = static_cast<int16_t>(state.right_servo_us);
    packet.left_servo_cmd = static_cast<int16_t>(state.left_servo_command * 10000);
    packet.right_servo_cmd = static_cast<int16_t>(state.right_servo_command * 10000);

    // Sensor health
    packet.imu_valid = state.imu_valid ? 1 : 0;
    packet.baro_valid = state.barometer_valid ? 1 : 0;
    packet.gps_fix_valid = state.gps_valid ? 1 : 0;

    // System
    packet.loop_rate_hz = static_cast<int16_t>(state.loop_rate_hz * 10);
    packet.free_heap_kb = static_cast<uint16_t>(state.free_heap_bytes / 1024);
    packet.lora_rssi = static_cast<uint16_t>(state.lora_rssi + 200); // offset for unsigned
    packet.lora_snr_x10 = static_cast<int16_t>(state.lora_snr * 10);
    packet.drop_test_state = static_cast<uint8_t>(state.drop_test_state);
    packet.drop_test_flags = (state.drop_test_recording ? 1U : 0U) |
                             (state.drop_test_neutral_lock ? 2U : 0U);
    packet.drop_test_id = state.drop_test_id;
    packet.drop_test_armed_ms = state.drop_test_armed_ms;
    packet.drop_test_release_ms = state.drop_test_release_confirm_ms;
    packet.drop_test_landing_ms = state.drop_test_landing_confirm_ms;

    // CRC (compute over everything except CRC field)
    size_t crc_len = sizeof(TelemetryPacketV1) - sizeof(uint16_t);
    packet.crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&packet), crc_len);
}

} // namespace telemetry
