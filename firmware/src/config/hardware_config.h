// ============================================================================
// PHOENIX RECOVERY — hardware configuration
// Pins, I2C addresses, radio hardware, GPS wiring, servo signals.
// Centralized here ONLY — see docs/PIN_MAP.md.
// ============================================================================
#pragma once

#include <cstdint>

namespace cfg {

// ---------------------------------------------------------------------------
// Sensor I2C bus (custom bus — board default SDA=3/SCL=4 is intentionally
// unused; GPIO4 is repurposed as the left servo).
// ---------------------------------------------------------------------------
constexpr int PIN_I2C_SDA        = 48;
constexpr int PIN_I2C_SCL        = 47;
constexpr uint32_t I2C_FREQ_HZ   = 400000;

// BNO085 IMU — module ADDR pin high => 0x4B on the GY-BNO08X.
constexpr uint8_t  BNO08X_ADDR   = 0x4B;
constexpr int      PIN_BNO08X_INT = 1;    // optional; -1 disables (polling works)

// BMP388 barometer — current PHOENIX module reports at 0x77.
// If SDO/SAO is strapped differently, the driver also probes 0x76.
constexpr uint8_t  BMP388_ADDR   = 0x77;

// ---------------------------------------------------------------------------
// Heltec L76K GNSS — GNSS connector. RX = ESP32 receives L76K TX.
// ---------------------------------------------------------------------------
constexpr int PIN_GPS_RX          = 39;   // ESP32 RX  <- L76K TX
constexpr int PIN_GPS_TX          = 38;   // ESP32 TX  -> L76K RX
constexpr int PIN_GPS_PPS         = 41;   // optional; -1 disables
constexpr int PIN_GNSS_POWER      = 34;   // Heltec VGNSS_CTRL, active LOW
constexpr int PIN_GNSS_RST        = 42;   // Heltec GNSS reset, active LOW; set HIGH to run
constexpr unsigned long GPS_BAUD  = 9600;

// ---------------------------------------------------------------------------
// Servos (parafoil brake lines). Signal only — power from external 5 V UBEC.
// ---------------------------------------------------------------------------
constexpr int PIN_SERVO_LEFT      = 4;    // ⚠ board-default I2C SCL — never
                                          //   call bare Wire.begin() (PIN_MAP §4)
constexpr int PIN_SERVO_RIGHT     = 6;

// ---------------------------------------------------------------------------
// LoRa SX1262 (Heltec V4 board-native — do NOT redefine; see PIN_MAP §1)
// ---------------------------------------------------------------------------
constexpr int PIN_LORA_NSS       = 8;
constexpr int PIN_LORA_SCK       = 9;
constexpr int PIN_LORA_MOSI      = 10;
constexpr int PIN_LORA_MISO      = 11;
constexpr int PIN_LORA_RST       = 12;
constexpr int PIN_LORA_BUSY      = 13;
constexpr int PIN_LORA_DIO1      = 14;

// Status LED on the Heltec V4 (optional indicator).
constexpr int PIN_LED            = 35;

} // namespace cfg
