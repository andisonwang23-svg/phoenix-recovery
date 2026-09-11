# PHOENIX RECOVERY — Firmware Summary

## Overview

This document describes the complete firmware flashed to the Heltec ESP32-S3 LoRa V4 for autonomous parafoil recovery.

**Build Info:**
- Platform: ESP32-S3 (Arduino framework)
- Flash: 8MB / PSRAM: 2MB
- Upload: ESPTOOL @ 460800 baud
- RAM Usage: 49,912 bytes (15.2%)
- Flash Usage: 910,033 bytes (27.2%)

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    MAIN LOOP (100 Hz)                       │
├─────────────────────────────────────────────────────────────┤
│  Sensor Manager    │  State Estimator  │  State Machine    │
│  (IMU, Baro, GPS)  │  (Altitude, VS)   │  (22 states)      │
├─────────────────────────────────────────────────────────────┤
│  Guidance Controller (15 Hz)  │  Servo Controller (50 Hz)  │
│  (Proportional steering)      │  (Slew-limited commands)   │
├─────────────────────────────────────────────────────────────┤
│  Health Monitor    │  Failsafe       │  Watchdog          │
│  (Subsystem check) │  (Auto-action)  │  (1s timeout)      │
├─────────────────────────────────────────────────────────────┤
│  Telemetry (LoRa)  │  WiFi Dashboard │  Flash Logger      │
│  (5 Hz binary)     │  (AP + SSE)     │  (Event log)       │
└─────────────────────────────────────────────────────────────┘
```

---

## Modules

### 1. Sensor Manager (`sensors/sensor_manager.cpp`)

**Sensors:**
- **IMU**: BNO08x (SparkFun library) @ I2C 0x4B, 50 Hz
- **Barometer**: BMP388 (Adafruit) @ I2C 0x76, 25 Hz
- **GPS**: NEO-6M (TinyGPSPlus) @ UART2, 1 Hz

**Processing:**
- Data quality validation (plausibility checks)
- Complementary filter for vertical speed
- Moving average on altitude (8 samples)
- Navigation computation (distance, bearing, heading error)

### 2. State Estimator (`estimation/state_estimator.cpp`)

- Altitude AGL from barometer (ground-calibrated)
- Vertical speed via complementary filter (baro + IMU)
- GPS course filtering (low-pass)

### 3. Flight State Machine (`logic/state_machine.cpp`)

**22 States:**
```
BOOT → SELF_TEST → PAD_SAFE → ASCENT → APOGEE_TRANSITION
→ PARAFOIL_DEPLOYMENT_WAIT → PARAFOIL_STABILIZATION
→ GUIDED_DESCENT → FINAL_APPROACH → FLARE → LANDED
→ FAILSAFE (from any state on critical failure)
```

**Transitions guarded by multiple signals:**
- Launch: 2 of 3 (accel > 8 m/s², VS > 5 m/s, alt gain > 10m)
- Apogee: |VS| < 1.5 m/s
- Landing: |VS| < 1 m/s + ground speed < 2 m/s

### 4. Guidance Controller (`control/guidance.cpp`)

**Modes:**
- `MODE_DISABLED` - No steering
- `MANUAL` - Bench test only
- `HEADING_TO_TARGET` - Proportional to heading error
- `FINAL_APPROACH` - Reduced gain near target
- `FLARE` - Symmetric brake for landing
- `MODE_FAILSAFE` - Neutral/emergency

**Safety:**
- Deadband: 5°
- Max steering: ±35%
- Rate limit: 0.5 /s
- Reversal guard: 800ms
- Geofence: 3000m radius

### 5. Servo Controller (`control/servo_controller.cpp`)

- Independent calibration per servo
- Slew limiting: 400 μs/s
- 50 Hz refresh
- NVS storage for calibration
- Emergency neutral command

### 6. LoRa Telemetry (`comms/lora.cpp`)

- RadioLib v7 SX1262 driver
- 915 MHz, SF7, BW 125 kHz, CR 4/5
- Binary packet v1 (CCITT-16 CRC)
- 5 Hz default (state-dependent)

### 7. WiFi Dashboard (`comms/wifi_manager.cpp`)

- AP mode: `PHOENIX-RECOVERY` / `parafoil`
- AsyncWebServer + SSE live updates
- Endpoints:
  - `/` - Main dashboard
  - `/api/status` - Flight state JSON
  - `/api/sensors` - Sensor data
  - `/api/navigation` - Target/guidance
  - `/api/config` - GET/POST configuration
  - `/config` - Config HTML page
  - `/bench` - Bench test page
  - `/api/servo/test` - Servo commands (bench mode)

### 8. Safety Systems

**Health Monitor** (`safety/health_monitor.cpp`):
- Per-subsystem health scoring
- Staleness detection (200ms timeout)
- Overall health: NOMINAL / WARNING / CRITICAL / FAILSAFE

**Failsafe Controller** (`safety/failsafe.cpp`):
- Triggers: IMU fail, Baro fail, GPS timeout, Servo fail, Watchdog
- Actions: Neutral servos, Disable guidance, Reboot

**Hardware Watchdog** (`safety/watchdog.cpp`):
- ESP32 hardware WDT
- 1s timeout
- Fed in main loop

### 9. Logging (`logging/`)

- **Flash Logger** (LittleFS): Flight events, state changes, telemetry snapshots
- **Ring Buffer** (RAM): High-speed events, survives soft reset

---

## Configuration (`config.h`)

Key tunable parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TARGET_LATITUDE` | 0.0 | **REQUIRED** - Landing zone |
| `TARGET_LONGITUDE` | 0.0 | **REQUIRED** - Landing zone |
| `GUIDANCE_KP` | 0.02 | Steering gain |
| `GUIDANCE_DEADBAND_DEG` | 5.0 | Heading deadband |
| `MAX_STEERING_COMMAND` | 0.35 | Max brake fraction |
| `DEPLOYMENT_WAIT_MS` | 1500 | Post-apogee wait |
| `PARAFOIL_STABILIZATION_MS` | 3000 | Canopy settle time |
| `FLARE_ENABLED` | true | Enable flare landing |
| `FLARE_ALTITUDE_M` | 8.0 | Flare trigger altitude |
| `SERVO_LEFT_NEUTRAL_US` | 1500 | Left neutral pulse |
| `SERVO_RIGHT_NEUTRAL_US` | 1500 | Right neutral pulse |

---

## Pin Map (Heltec V4)

| Function | GPIO |
|----------|------|
| I2C SDA | 47 |
| I2C SCL | 48 |
| GPS RX | 38 |
| GPS TX | 39 |
| GPS PPS | 41 |
| Servo Left | 4 |
| Servo Right | 6 |
| LoRa NSS | 8 |
| LoRa SCK | 9 |
| LoRa MOSI | 10 |
| LoRa MISO | 11 |
| LoRa RST | 12 |
| LoRa BUSY | 13 |
| LoRa DIO1 | 14 |

---

## Unit Tests (Native)

49 tests passing:
- `test_nav_math`: 28 tests (Haversine, bearing, heading error, normalize, deadband, clamp)
- `test_filters`: 21 tests (Moving average, low-pass, vertical speed, rate limiter, reversal guard)

Run: `pio test -e native`

---

## Build Commands

```bash
# Compile for ESP32
pio run -e rocket

# Upload (board must be in bootloader mode)
pio run -e rocket --target upload

# Run unit tests
pio test -e native

# Bench test mode
pio run -e rocket --build-flags="-DBENCH_TEST_MODE=1"

# Simulation mode
pio run -e rocket --build-flags="-DSIMULATION_MODE=1"
```

---

## Verification

**Flashing successful:** ✅
**WiFi AP active:** ✅ (PHOENIX-RECOVERY)
**Dashboard endpoints:** ✅ (all 5 endpoints responding)
**Firmware state:** SELF_TEST (awaiting I2C sensors)

**To complete setup:**
1. Connect BNO08x IMU to I2C (GPIO47/48)
2. Connect BMP388 barometer to I2C
3. Connect NEO-6M GPS to UART2 (GPIO38/39)
4. Set target coordinates in `/config` page
5. System will progress: SELF_TEST → PAD_SAFE → ready for flight