// ============================================================================
// PHOENIX RECOVERY — LoRa Remote Command Protocol.
// ============================================================================
// Fixed-size packets used by a second Heltec LoRa board as the computer-side
// ground station. The rocket treats these as supervised requests, never as
// permission to bypass the flight coordinator or failsafe.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <cstdint>

namespace comms {

constexpr uint32_t LORA_REMOTE_MAGIC = 0x43584850UL; // "PHXC" little-endian
constexpr uint8_t LORA_REMOTE_VERSION = 1;

enum class LoRaRemoteCommandType : uint8_t {
    PING = 1,
    NEUTRAL = 2,
    MANUAL_BRAKE = 3,
    SET_TARGET = 4,
    DISABLE_REMOTE = 5,
    BENCH_SERVO = 6,
    ARM_DROP_TEST = 7,
    ABORT_DROP_TEST = 8,
    REQUEST_LOG_INDEX = 9,
    CANCEL_LOG_TRANSFER = 10
};

struct __attribute__((packed)) LoRaRemoteCommandPacket {
    uint32_t magic = LORA_REMOTE_MAGIC;
    uint8_t version = LORA_REMOTE_VERSION;
    uint8_t type = static_cast<uint8_t>(LoRaRemoteCommandType::PING);
    uint16_t sequence = 0;
    int16_t servo1_command_milli = 0;   // -1000..+1000
    int16_t servo2_command_milli = 0;   // -1000..+1000
    int32_t target_lat_e7 = 0;
    int32_t target_lon_e7 = 0;
    uint16_t crc = 0;
};

inline uint16_t loraRemoteChecksum(const LoRaRemoteCommandPacket& packet) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&packet);
    uint16_t sum = 0x51A7;
    for (size_t i = 0; i < sizeof(LoRaRemoteCommandPacket) - sizeof(packet.crc); ++i) {
        sum = static_cast<uint16_t>((sum << 5) | (sum >> 11));
        sum ^= bytes[i];
    }
    return sum;
}

inline void finalizeLoRaRemotePacket(LoRaRemoteCommandPacket& packet) {
    packet.magic = LORA_REMOTE_MAGIC;
    packet.version = LORA_REMOTE_VERSION;
    packet.crc = 0;
    packet.crc = loraRemoteChecksum(packet);
}

inline bool validLoRaRemotePacket(const LoRaRemoteCommandPacket& packet) {
    if (packet.magic != LORA_REMOTE_MAGIC) return false;
    if (packet.version != LORA_REMOTE_VERSION) return false;
    return packet.crc == loraRemoteChecksum(packet);
}

inline const char* loraRemoteCommandName(LoRaRemoteCommandType type) {
    switch (type) {
        case LoRaRemoteCommandType::PING: return "PING";
        case LoRaRemoteCommandType::NEUTRAL: return "NEUTRAL";
        case LoRaRemoteCommandType::MANUAL_BRAKE: return "MANUAL_BRAKE";
        case LoRaRemoteCommandType::SET_TARGET: return "SET_TARGET";
        case LoRaRemoteCommandType::DISABLE_REMOTE: return "DISABLE_REMOTE";
        case LoRaRemoteCommandType::BENCH_SERVO: return "BENCH_SERVO";
        case LoRaRemoteCommandType::ARM_DROP_TEST: return "ARM_DROP_TEST";
        case LoRaRemoteCommandType::ABORT_DROP_TEST: return "ABORT_DROP_TEST";
        case LoRaRemoteCommandType::REQUEST_LOG_INDEX: return "REQUEST_LOG_INDEX";
        case LoRaRemoteCommandType::CANCEL_LOG_TRANSFER: return "CANCEL_LOG_TRANSFER";
    }
    return "UNKNOWN";
}

} // namespace comms
