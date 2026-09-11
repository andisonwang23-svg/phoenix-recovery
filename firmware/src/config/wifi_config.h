// ============================================================================
// PHOENIX RECOVERY — communications configuration
// WiFi AP (config/test UI only), web dashboard, and LoRa telemetry radio.
//
// REMINDER (§10): WiFi/LoRa are NEVER required for autonomous recovery.
// Everything here is ground-support tooling.
// ============================================================================
#pragma once

#include <cstdint>

namespace cfg {

// ---------------------------------------------------------------------------
// WiFi access point (config/test UI).  Radio is on the ground, not in the sky.
// ---------------------------------------------------------------------------
constexpr const char *WIFI_AP_SSID     = "PHOENIX_RECOVERY";
constexpr const char *WIFI_AP_PASSWORD = "CHANGE_ME_1234"; // ≥8 chars — set privately before flight/bench use
constexpr const char *WIFI_AP_IP       = "192.168.4.1"; // default SoftAP gateway
constexpr uint16_t    WIFI_HTTP_PORT   = 80;

// WiFi is only started when the rocket is on the pad (safe to serve a UI).
constexpr bool     WIFI_ENABLE_WHEN_ARMED = false;

// ---------------------------------------------------------------------------
// Web dashboard (responsive, SSE live data — see docs/WEB_DASHBOARD.md)
// ---------------------------------------------------------------------------
constexpr int      DASHBOARD_PUSH_RATE_HZ = 5;   // SSE update cadence
constexpr int      CONFIG_PAGE_MAX_LEN    = 2048;

// ---------------------------------------------------------------------------
// LoRa SX1262 telemetry (downlink only — recovery never depends on it)
// ---------------------------------------------------------------------------
constexpr float    LORA_FREQ_MHZ      = 915.0f;   // US ISM band
constexpr float    LORA_BW_KHZ        = 125.0f;
constexpr uint8_t  LORA_SF            = 7;
constexpr uint8_t  LORA_CR            = 5;        // 4/5
constexpr int8_t   LORA_TX_POWER      = 22;       // dBm — REQUIRES regional approval

// ---------------------------------------------------------------------------
// Ground-station radio (see ground_station/firmware) — same modem settings,
// separate sync word keeps it isolated from other LoRa users nearby.
// ---------------------------------------------------------------------------
constexpr uint8_t  LORA_SYNC_WORD     = 0x34;

} // namespace cfg
