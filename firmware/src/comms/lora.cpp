// ============================================================================
// PHOENIX RECOVERY — LoRa Telemetry Implementation.
// ============================================================================
#include "lora.h"

namespace comms {

LoRaTelemetry::LoRaTelemetry() = default;

LoRaTelemetry::~LoRaTelemetry() {
    if (radio_) delete radio_;
}

bool LoRaTelemetry::begin() {
    // RadioLib v7: create Module first, then SX1262
    Module* mod = new Module(
        cfg::PIN_LORA_NSS,
        cfg::PIN_LORA_DIO1,
        cfg::PIN_LORA_RST,
        cfg::PIN_LORA_BUSY,
        SPI
    );

    radio_ = new SX1262(mod);

    if (!radio_) return false;

    // Initialize SPI
    SPI.begin(cfg::PIN_LORA_SCK, cfg::PIN_LORA_MISO, cfg::PIN_LORA_MOSI, cfg::PIN_LORA_NSS);

    // Configure radio
    if (!configureRadio()) {
        return false;
    }

    initialized_ = true;
    last_send_ms_ = millis();
    return true;
}

bool LoRaTelemetry::configureRadio() {
    int16_t state = radio_->begin(
        cfg::LORA_FREQ_MHZ,
        cfg::LORA_BW_KHZ,
        cfg::LORA_SF,
        cfg::LORA_CR,
        cfg::LORA_SYNC_WORD,
        cfg::LORA_TX_POWER,
        8,    // preamble length
        1.6   // SX1262 TCXO voltage
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] SX1262 begin failed, RadioLib code %d\n", state);
        return false;
    }

    return true;
}

bool LoRaTelemetry::send(const telemetry::TelemetryPacketV1& packet) {
    if (!initialized_) return false;

    // Send packet
    int16_t state = radio_->transmit(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(telemetry::TelemetryPacketV1)
    );

    if (state == RADIOLIB_ERR_NONE) {
        last_rssi_ = radio_->getRSSI();
        last_snr_ = radio_->getSNR();
        sequence_++;
        return true;
    }

    return false;
}

void LoRaTelemetry::update() {
    // Handle any background radio tasks
    // RadioLib handles most things in transmit()
}

} // namespace comms
