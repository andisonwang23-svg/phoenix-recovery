// ============================================================================
// PHOENIX RECOVERY — LoRa Telemetry (SX1262 via RadioLib).
// ============================================================================
#pragma once

#include <Arduino.h>
#include <RadioLib.h>
#include "config.h"
#include "vehicle_state.h"
#include "telemetry/telemetry_packet.h"

namespace comms {

class LoRaTelemetry {
public:
    LoRaTelemetry();
    ~LoRaTelemetry();

    // Initialize LoRa radio
    bool begin();

    // Send telemetry packet
    bool send(const telemetry::TelemetryPacketV1& packet);

    // Check if radio is healthy
    bool isHealthy() const { return initialized_; }

    // Get last RSSI/SNR
    int16_t getLastRSSI() const { return last_rssi_; }
    float getLastSNR() const { return last_snr_; }

    // Update - handle any background tasks
    void update();

private:
    bool initialized_ = false;
    SX1262* radio_ = nullptr;
    uint8_t sequence_ = 0;
    uint32_t last_send_ms_ = 0;
    int16_t last_rssi_ = 0;
    float last_snr_ = 0;
    uint32_t send_interval_ms_ = 1000; // 1 Hz default, adjusted by flight state

    bool configureRadio();
};

} // namespace comms