// ============================================================================
// PHOENIX RECOVERY — hardware-free LoRa airtime/scheduling checks.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

namespace logic {

constexpr uint32_t ceilDivU32(uint32_t numerator, uint32_t denominator) {
    return (numerator + denominator - 1U) / denominator;
}

// Semtech LoRa packet-time equation, using explicit header and CRC by default.
// `coding_rate_denominator` is 5 for 4/5, 6 for 4/6, and so on.
inline uint32_t loraPacketAirtimeUs(size_t payload_bytes,
                                   uint8_t spreading_factor,
                                   uint32_t bandwidth_hz,
                                   uint8_t coding_rate_denominator,
                                   uint16_t preamble_symbols = 8,
                                   bool crc_enabled = true,
                                   bool explicit_header = true) {
    if (bandwidth_hz == 0U || spreading_factor < 5U || spreading_factor > 12U ||
        coding_rate_denominator < 5U || coding_rate_denominator > 8U) {
        return UINT32_MAX;
    }

    const uint32_t symbol_us = ceilDivU32((1UL << spreading_factor) * 1000000UL,
                                          bandwidth_hz);
    const uint32_t low_data_rate_optimization =
        (spreading_factor >= 11U && bandwidth_hz <= 125000U) ? 1U : 0U;
    const int32_t numerator = static_cast<int32_t>(8U * payload_bytes) -
        static_cast<int32_t>(4U * spreading_factor) + 28 +
        (crc_enabled ? 16 : 0) - (explicit_header ? 0 : 20);
    const uint32_t denominator = 4U *
        (static_cast<uint32_t>(spreading_factor) - 2U * low_data_rate_optimization);
    const uint32_t payload_blocks = numerator > 0 ?
        ceilDivU32(static_cast<uint32_t>(numerator), denominator) : 0U;
    const uint32_t payload_symbols = 8U +
        payload_blocks * static_cast<uint32_t>(coding_rate_denominator);

    // The LoRa preamble adds 4.25 symbols. Work in quarter-symbols to keep this
    // constexpr and avoid floating point.
    const uint32_t total_quarter_symbols =
        4U * payload_symbols + 4U * preamble_symbols + 17U;
    return ceilDivU32(total_quarter_symbols * symbol_us, 4U);
}

inline bool loraScheduleLeavesReceiveWindow(size_t payload_bytes,
                                            uint8_t spreading_factor,
                                            uint32_t bandwidth_hz,
                                            uint8_t coding_rate_denominator,
                                            uint32_t telemetry_rate_hz,
                                            uint32_t minimum_receive_window_ms) {
    if (telemetry_rate_hz == 0U) return false;
    const uint32_t interval_us = 1000000U / telemetry_rate_hz;
    const uint32_t airtime_us = loraPacketAirtimeUs(
        payload_bytes, spreading_factor, bandwidth_hz, coding_rate_denominator);
    return airtime_us != UINT32_MAX &&
        interval_us >= airtime_us + minimum_receive_window_ms * 1000U;
}

} // namespace logic
