// ============================================================================
// PHOENIX RECOVERY — single source of truth for pins, constants, tuning.
// This is the ONLY file that should need editing for normal integration work.
// Edit this file, keep docs/PIN_MAP.md in sync, then re-run SELF_TEST.
// ============================================================================
#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Feature switches (may also be supplied as -D build flags; these are defaults)
// ---------------------------------------------------------------------------
#ifndef BENCH_TEST_MODE
#define BENCH_TEST_MODE 0        // 1 = bench CLI, no launch detect, no auto guidance
#endif
#ifndef SIMULATION_MODE
#define SIMULATION_MODE 0        // 1 = synthetic sensor data through the real pipeline
#endif
#ifndef FLASH_EVENT_LOG_ENABLED
#define FLASH_EVENT_LOG_ENABLED 0 // 1 = optional LittleFS event log (boot/fault/landed)
#endif

namespace cfg {

// ---------------------------------------------------------------------------
// Pin map — Heltec WiFi LoRa 32 V4.3 / V4 R8 (ESP32-S3). See docs/PIN_MAP.md.
// ---------------------------------------------------------------------------
// Sensor I2C bus
constexpr int PIN_I2C_SDA        = 48;
constexpr int PIN_I2C_SCL        = 47;
constexpr int PIN_BNO08X_INT     = 1;   // optional; -1 disables (polling works)
// Heltec L76K GNSS — UART2 / GNSS connector
constexpr int PIN_GPS_RX         = 39;  // ESP32 RX  <- L76K TX
constexpr int PIN_GPS_TX         = 38;  // ESP32 TX  -> L76K RX
constexpr int PIN_GPS_PPS        = 41;  // optional; -1 disables
constexpr int PIN_GNSS_POWER     = 34;  // Heltec VGNSS_CTRL, active LOW
constexpr int PIN_GNSS_RST       = 42;  // Heltec GNSS reset, active LOW; set HIGH to run
constexpr unsigned long GPS_BAUD = 9600;
// Servos (brake lines)
constexpr int PIN_SERVO_LEFT     = 4;   // ⚠ GPIO4 is the board default I2C SCL —
                                        //   never call bare Wire.begin() (see PIN_MAP.md §4)
constexpr int PIN_SERVO_RIGHT    = 6;
// LoRa SX1262 (Heltec V4 board-native — do NOT redefine)
constexpr int PIN_LORA_NSS       = 8;
constexpr int PIN_LORA_SCK       = 9;
constexpr int PIN_LORA_MOSI      = 10;
constexpr int PIN_LORA_MISO      = 11;
constexpr int PIN_LORA_RST       = 12;
constexpr int PIN_LORA_BUSY      = 13;
constexpr int PIN_LORA_DIO1      = 14;

// ---------------------------------------------------------------------------
// Sensor addresses (verified during bring-up)
// ---------------------------------------------------------------------------
constexpr uint8_t  BNO08X_ADDR   = 0x4B; // GY-BNO08X, module ADDR pin high
constexpr uint8_t  BMP388_ADDR   = 0x77; // current PHOENIX barometer: SDO/SAO -> 3V3
constexpr uint32_t I2C_FREQ_HZ   = 100000; // conservative bench rate; scanner verified sensors at 100 kHz

// ---------------------------------------------------------------------------
// Radio
// ---------------------------------------------------------------------------
constexpr float    LORA_FREQ_MHZ = 915.0f;   // US 915 MHz band
constexpr float    LORA_BW_KHZ   = 125.0f;
constexpr uint8_t  LORA_SF       = 7;
constexpr uint8_t  LORA_CR       = 5;        // 4/5
constexpr int8_t   LORA_TX_POWER = 22;

// ---------------------------------------------------------------------------
// Landing target (REQUIRED: set to the launch-day landing zone)
// ---------------------------------------------------------------------------
constexpr double   TARGET_LATITUDE  = 0.0;   // <-- REQUIRED before any guided flight
constexpr double   TARGET_LONGITUDE = 0.0;   // <-- REQUIRED before any guided flight

// ---------------------------------------------------------------------------
// Flight timing / phase thresholds
// ---------------------------------------------------------------------------
constexpr uint32_t APOGEE_CONFIRMATION_TIME_MS   = 700;
constexpr uint32_t DEPLOYMENT_WAIT_MS            = 1500; // motor ejection time
constexpr uint32_t PARAFOIL_STABILIZATION_MS     = 3000; // canopy inflation + settle
constexpr uint32_t MIN_STABILIZATION_TIME_MS     = 3000; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr uint32_t MAX_STABILIZATION_TIME_MS     = 15000;// REQUIRES EXPERIMENTAL CALIBRATION
constexpr float    MAX_STABILIZATION_GYRO_RATE   = 15.0f;// REQUIRES EXPERIMENTAL CALIBRATION
constexpr float    MIN_DESCENT_SPEED             = 1.5f; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr uint32_t LANDED_CONFIRM_MS             = 4000;
constexpr uint32_t GPS_LOSS_TIMEOUT_MS           = 5000; // in guided states only
constexpr uint32_t SENSOR_TIMEOUT_MS             = 750;  // IMU/baro report deadline; allows mixed BNO08x reports and I2C jitter
constexpr uint32_t HEALTH_WATCHDOG_LOOP_MS       = 250;  // main loop must tick < this
constexpr uint32_t LAUNCH_CONFIRM_MS             = 400;
constexpr float    LAUNCH_ACCEL_MPS2             = 8.0f;  // sustained vertical accel
constexpr float    LAUNCH_VS_MPS                 = 5.0f;  // rising vertical speed
constexpr float    LAUNCH_ALT_DELTA_M            = 10.0f; // altitude gained
constexpr float    APOGEE_VS_THRESHOLD_MPS       = 1.5f;  // |vs| below this near apogee
constexpr float    APOGEE_ALT_FLAT_M             = 2.0f;  // max climb across window

// ---------------------------------------------------------------------------
// Estimator / filters
// ---------------------------------------------------------------------------
constexpr float    ALT_FILTER_TAU_S      = 0.3f;
constexpr float    VS_FILTER_GAIN_ALT    = 0.2f;  // complementary estimator gains
constexpr float    VS_FILTER_GAIN_VS     = 0.10f;
constexpr uint32_t BARO_CAL_SAMPLES      = 50;    // launch-ground pressure average
constexpr uint8_t  ALT_MA_WINDOW         = 8;     // moving-average window (samples)
constexpr float    COURSE_SPEED_THRESHOLD_MPS = 3.0f; // min GPS speed to trust course

// ---------------------------------------------------------------------------
// Guidance (parafoil steering)
// ---------------------------------------------------------------------------
constexpr float    GUIDANCE_KP                 = 0.02f; // per degree of heading error
constexpr float    GUIDANCE_DEADBAND_DEG       = 5.0f;
// Conservative pre-calibration ceiling. Replace only with the measured safe
// limit from an inert recovery-module calibration; 80% was an unsafe placeholder.
constexpr float    MAX_STEERING_COMMAND        = 0.35f; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr float    CMD_RATE_LIMIT_PER_S        = 0.5f;  // command fraction change / s
constexpr uint32_t REVERSAL_GUARD_MS          = 800;   // min time between turn reversals
constexpr float    FINAL_APPROACH_KP           = 0.010f;
constexpr float    FINAL_APPROACH_MAX_COMMAND  = 0.50f;
constexpr float    FINAL_APPROACH_ALTITUDE_M   = 30.0f;
constexpr float    FINAL_APPROACH_RADIUS_M     = 40.0f; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr float    FINAL_APPROACH_MAX_STEERING = 0.50f; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr bool     FLARE_ENABLED              = false;
constexpr float    FLARE_ALTITUDE_M            = 8.0f;
constexpr float    FLARE_BRAKE_AMOUNT          = 0.30f; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr uint32_t FLARE_RAMP_TIME_MS          = 1500; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr float    FLARE_MAX_BRAKE             = 0.6f;  // hard cap even when flare enabled
constexpr float    FLARE_STALL_RELEASE_DPS     = 60.0f; // release if rotation rate spikes
constexpr float    MAX_GUIDANCE_RADIUS_M       = 3000.0f; // geofence
constexpr float    MIN_GUIDANCE_ALTITUDE_M     = 12.0f;   // below this -> no aggressive nav
constexpr float    TARGET_ACCEPT_RADIUS_M      = 10.0f;

// ---------------------------------------------------------------------------
// Landing detection
// ---------------------------------------------------------------------------
constexpr float    LANDED_ALT_CHANGE_M   = 1.0f;
constexpr float    LANDED_VS_MPS         = 1.0f;
constexpr float    LANDED_GND_SPEED_MPS  = 2.0f;
constexpr float    LANDED_ACCEL_JITTER_G = 0.15f;

// ---------------------------------------------------------------------------
// Data-quality rejection
// ---------------------------------------------------------------------------
constexpr float    GPS_MAX_SPEED_MPS     = 120.0f;  // physical envelope for a parafoil
constexpr float    GPS_MAX_JUMP_M        = 50.0f;   // per second position jump
constexpr uint8_t  GPS_RECOVERY_FIX_COUNT = 3;
constexpr float    GPS_MIN_GROUND_SPEED_FOR_COURSE = 3.0f;
constexpr float    BARO_MAX_ALT_STEP_M   = 30.0f;   // per second, physically impossible here
constexpr float    QUAT_MIN_NORM         = 0.85f;
constexpr float    QUAT_MAX_NORM         = 1.15f;

// ---------------------------------------------------------------------------
// Servo calibration (PLACEHOLDER values — replace with measured bench data,
// spec §28 / docs/GROUND_TEST_CHECKLIST.md). Never enable full brake until measured.
// ---------------------------------------------------------------------------
constexpr float    SERVO_LEFT_NEUTRAL_US    = 1500.0f;
constexpr float    SERVO_LEFT_MIN_US        = 1000.0f;
constexpr float    SERVO_LEFT_MAX_US        = 2000.0f;
constexpr float    SERVO_LEFT_MAX_BRAKE_US  = 500.0f;  // full calibrated travel from neutral
constexpr bool     SERVO_LEFT_REVERSED      = false;
constexpr float    SERVO_RIGHT_NEUTRAL_US   = 1500.0f;
constexpr float    SERVO_RIGHT_MIN_US       = 1000.0f;
constexpr float    SERVO_RIGHT_MAX_US       = 2000.0f;
constexpr float    SERVO_RIGHT_MAX_BRAKE_US = 500.0f;
constexpr bool     SERVO_RIGHT_REVERSED     = false;
// Faster bench response while retaining a bounded ramp. With the current
// conservative 35% command cap and 500 us calibrated travel this reaches the
// requested position in about 0.22 s. REQUIRES EXPERIMENTAL CALIBRATION.
constexpr float    SERVO_SLEW_RATE_US_PER_S = 800.0f;
constexpr uint32_t MIN_TURN_REVERSAL_TIME_MS = 1200; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr uint32_t SERVO_REFRESH_MS         = 20;     // 50 Hz refresh

// ---------------------------------------------------------------------------
// Update rates (Hz)
// ---------------------------------------------------------------------------
constexpr int      RATE_IMU_HZ       = 50;
constexpr int      RATE_BARO_HZ      = 25;
constexpr int      RATE_ESTIMATOR_HZ = 50;
constexpr int      RATE_GUIDANCE_HZ  = 15;
constexpr int      RATE_HEALTH_HZ    = 8;
constexpr int      RATE_TELEMETRY_HZ = 5; // default; see per-state map below

// Telemetry rate per flight state (Hz)
constexpr int      TLM_RATE_PAD_HZ     = 1;
constexpr int      TLM_RATE_FLIGHT_HZ  = 5;  // ascent / guided
constexpr int      TLM_RATE_LANDED_HZ  = 1;

// ---------------------------------------------------------------------------
// Watchdog / fault
// ---------------------------------------------------------------------------
constexpr bool     ENABLE_HARDWARE_WDT = true;
constexpr uint32_t WDT_TIMEOUT_MS      = 1000;

// ---------------------------------------------------------------------------
// Target configuration
// ---------------------------------------------------------------------------
constexpr const char* TARGET_NAME = "Default Target";
constexpr double TARGET_LAT = 0.0;  // Alias for TARGET_LATITUDE
constexpr double TARGET_LON = 0.0;  // Alias for TARGET_LONGITUDE

// ---------------------------------------------------------------------------
// LoRa sync word and update rates
// ---------------------------------------------------------------------------
constexpr uint8_t LORA_SYNC_WORD = 0x12;
constexpr uint32_t TELEMETRY_RATE_HZ = 5;
constexpr uint32_t LOG_RATE_HZ = 10;
constexpr uint32_t CONFIG_VERSION = 3;
constexpr uint32_t MAIN_LOOP_INTERVAL_MS = 10; // 100 Hz main loop

// ---------------------------------------------------------------------------
// Inert drop-test mode
// ---------------------------------------------------------------------------
// A ground LoRa command can arm a payload-owned recording run before release.
// During early inert tests, the payload keeps servos neutral and uses only
// onboard sensor time for event markers.
constexpr bool     DROP_TEST_MODE_ENABLED = true;
constexpr bool     DROP_TEST_NEUTRAL_LOCK_ENABLED = true;
constexpr float    DROP_TEST_RELEASE_SPEED_MPS = -1.0f;
constexpr float    DROP_TEST_RELEASE_ALT_LOSS_M = 1.5f;
constexpr uint32_t DROP_TEST_RELEASE_CONFIRM_MS = 400;
constexpr float    DROP_TEST_LANDING_VS_MPS = 0.25f;
constexpr float    DROP_TEST_LANDING_ANGULAR_RATE_DPS = 8.0f;
constexpr uint32_t DROP_TEST_LANDING_CONFIRM_MS = 5000;
constexpr uint32_t DROP_TEST_POST_LANDING_RECORD_MS = 10000;
constexpr uint32_t DROP_TEST_MAX_DURATION_MS = 10UL * 60UL * 1000UL;

// ---------------------------------------------------------------------------
// LoRa remote-control supervision
// ---------------------------------------------------------------------------
// A second Heltec LoRa board may act as a computer-side bridge. These packets
// are only accepted as supervised descent commands; powered ascent and failsafe
// remain neutral-only.
constexpr bool     LORA_REMOTE_CONTROL_ENABLED = true;
constexpr uint32_t LORA_REMOTE_POLL_INTERVAL_MS = 80;
constexpr uint32_t LORA_REMOTE_RX_TIMEOUT_MS = 3;
constexpr uint32_t LORA_REMOTE_COMMAND_TIMEOUT_MS = 750;
constexpr float    LORA_REMOTE_MAX_BRAKE_COMMAND = 0.25f; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr bool     LORA_REMOTE_ALLOW_WITHOUT_TARGET = true;
constexpr bool     LORA_BENCH_SERVO_TEST_ENABLED = true;
constexpr float    LORA_BENCH_MAX_SERVO_COMMAND = 0.08f; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr uint32_t LORA_BENCH_SERVO_TIMEOUT_MS = 600;

// ---------------------------------------------------------------------------
// Servo channel aliases
// ---------------------------------------------------------------------------
constexpr int SERVO_LEFT_CHANNEL = PIN_SERVO_LEFT;
constexpr int SERVO_RIGHT_CHANNEL = PIN_SERVO_RIGHT;

// ---------------------------------------------------------------------------
// WiFi AP configuration
// ---------------------------------------------------------------------------
// Payload Wi-Fi is disabled in flight hardware. Configuration and monitoring
// are provided by the separate PHOENIX-GROUND LoRa dashboard.
constexpr bool PAYLOAD_WIFI_ENABLED = false;
constexpr const char* WIFI_AP_SSID = "PHOENIX-RECOVERY";
constexpr const char* WIFI_AP_PASS = "parafoil";
constexpr uint8_t WIFI_AP_CHANNEL = 1;
constexpr uint8_t WIFI_AP_MAX_CONN = 4;
constexpr uint8_t WIFI_BOOT_START_ATTEMPTS = 5;
constexpr uint32_t WIFI_BOOT_RETRY_DELAY_MS = 750;
constexpr uint32_t WIFI_RECOVERY_CHECK_MS = 3000;

// ---------------------------------------------------------------------------
// Dashboard configuration
// ---------------------------------------------------------------------------
constexpr uint32_t DASHBOARD_PUSH_RATE_HZ = 2;
constexpr uint16_t WIFI_HTTP_PORT = 80;

// ---------------------------------------------------------------------------
// Configuration loading
// ---------------------------------------------------------------------------
inline bool loadConfig() {
    // Configuration is compile-time constants
    // Could be extended to load from NVS/Preferences
    return true;
}
} // namespace cfg

// Ground altitude reference (m, above launch ground). 0.0 = auto-capture at
// PAD_SAFE entry. Set a fixed value to override.
#ifndef GROUND_ALTITUDE_REFERENCE
#define GROUND_ALTITUDE_REFERENCE 0.0
#endif
