// ============================================================================
// PHOENIX RECOVERY — LoRa Telemetry Implementation.
// ============================================================================
#include "lora.h"

namespace comms {

namespace {
volatile bool remote_packet_received = false;

void IRAM_ATTR onRemotePacketReceived() {
    remote_packet_received = true;
}

float commandMilliToFraction(int16_t milli) {
    return constrain(static_cast<float>(milli) / 1000.0f, -1.0f, 1.0f);
}
} // namespace

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
    radio_->setPacketReceivedAction(onRemotePacketReceived);
    radio_->startReceive();
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

    // Transmit temporarily takes the radio out of receive mode. Discard any
    // stale IRQ indication, then immediately return to continuous reception.
    remote_packet_received = false;
    int16_t state = radio_->transmit(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(telemetry::TelemetryPacketV1)
    );

    if (state == RADIOLIB_ERR_NONE) {
        last_rssi_ = radio_->getRSSI();
        last_snr_ = radio_->getSNR();
        sequence_++;
        radio_->startReceive();
        return true;
    }

    radio_->startReceive();
    return false;
}

bool LoRaTelemetry::pollRemoteCommand(LoRaRemoteCommand& command) {
    if (!initialized_ || !radio_ || !remote_packet_received) return false;
    remote_packet_received = false;

    LoRaRemoteCommandPacket packet;
    int16_t state = radio_->readData(
        reinterpret_cast<uint8_t*>(&packet),
        sizeof(packet)
    );

    radio_->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        return false;
    }

    if (!validLoRaRemotePacket(packet)) return false;

    command.type = static_cast<LoRaRemoteCommandType>(packet.type);
    command.sequence = packet.sequence;
    command.servo1_command = commandMilliToFraction(packet.servo1_command_milli);
    command.servo2_command = commandMilliToFraction(packet.servo2_command_milli);
    command.target_latitude = static_cast<double>(packet.target_lat_e7) / 10000000.0;
    command.target_longitude = static_cast<double>(packet.target_lon_e7) / 10000000.0;
    command.rssi = radio_->getRSSI();
    command.snr = radio_->getSNR();
    last_rssi_ = command.rssi;
    last_snr_ = command.snr;
    return true;
}

void LoRaTelemetry::update() {
    // Handle any background radio tasks
    // RadioLib handles most things in transmit()
}

} // namespace comms
