# PHOENIX RECOVERY — Implementation Summary

## Overview

This document summarizes the complete autonomous parafoil recovery firmware implementation for the Heltec ESP32-S3 LoRa V4 board.

## Architecture

The firmware is organized into modular subsystems:

```
firmware/
├── src/
│   ├── config.h                 # Central configuration
│   ├── vehicle_state.h          # Central state struct
│   ├── main.cpp                 # Entry point & control loop
│   ├── logic/                   # Hardware-free testable logic
│   │   ├── nav_math.h          # Distance, bearing, heading math
│   │   ├── filters.h           # Moving average, LPF, rate limiter
│   │   ├── state_machine.*     # 12-state flight state machine
│   │   ├── guidance_logic.*    # Steering computation
│   │   ├── detection.h         # Flight phase detection
│   │   └── data_quality.h      # Sensor validation
│   ├── sensors/                 # Hardware drivers
│   │   ├── imu.*               # BNO08x IMU
│   │   ├── barometer.*         # BMP388 barometer
│   │   ├── gps.*               # NEO-6M GPS
│   │   └── sensor_manager.*    # Combined sensor interface
│   ├── estimation/
│   │   └── state_estimator.*   # Altitude & vertical speed
│   ├── control/
│   │   ├── servo_controller.*  # Dual servo output
│   │   └── guidance.*          # Mode selection & servo mapping
│   ├── comms/
│   │   ├── lora.*              # SX1262 telemetry
│   │   └── wifi_manager.*      # AP + Web Dashboard
│   ├── telemetry/
│   │   └── telemetry_packet.*  # Binary v1 packet with CRC
│   ├── safety/
│   │   ├── health_monitor.*    # Sensor & system health tracking
│   │   ├── failsafe.*          # Deterministic fault response
│   │   └── watchdog.*          # Hardware watchdog timer
│   └── logging/
│       ├── flash_logger.*      # Persistent flight logs
│       └── ring_buffer.*       # High-speed RAM telemetry buffer
└── tests/
    ├── test_nav_math.cpp       # Navigation math unit tests
    └── test_filters.cpp        # Filter unit tests
```

## Key Features

### 1. 12-State Flight State Machine
- **States**: UNKNOWN → PRE_LAUNCH → ARMED → ASCENT → APOGEE_DETECT → STABILIZING → DESCENT_GUIDED → APPROACH → FLARE → TOUCHDOWN → RECOVERY → POST_FLIGHT
- **Guarded Transitions**: Multi-signal confirmation for all state changes
- **Safety-First**: Servos neutral until guided states

### 2. Autonomous Guidance
- **Proportional Steering**: Heading error → servo command mapping
- **Deadband**: Prevents hunting near target heading
- **Slew Limiting**: Prevents abrupt servo commands
- **Reversal Guard**: Minimum time between direction changes
- **Approach Logic**: Distance-based switch from proportional to flare
- **Flare Maneuver**: Full brake at target proximity

### 3. Sensor Fusion
- **Complementary Filter**: Baro + IMU altitude blending
- **Vertical Speed Estimator**: Low-pass filtered rate of change
- **Data Quality**: GPS/Baro/IMU validation and staleness detection

### 4. Safety Systems
- **Health Monitor**: Real-time subsystem health tracking
- **Failsafe Controller**: Deterministic fault response (neutral servos, flare, spiral, reboot)
- **Hardware Watchdog**: 5-second timeout with auto-reboot
- **Multi-Signal Guards**: No single sensor failure triggers premature action

### 5. Telemetry & Logging
- **Binary Telemetry Packets**: CRC-16 CCITT protected, 1 Hz LoRa
- **WiFi Dashboard**: Live SSE updates, config, bench test controls
- **Flash Logger**: Persistent flight logs (SPIFFS)
- **RAM Ring Buffer**: High-speed telemetry buffer for post-flight analysis

### 6. WiFi Dashboard
- **Live Telemetry**: SSE updates at configurable rate
- **Configuration**: Target coordinates, guidance tuning, servo calibration
- **Bench Test**: Servo test buttons, sensor validation
- **No External Dependencies**: All HTML/CSS/JS embedded

## Building

### Rocket Firmware (ESP32-S3)
```bash
cd phoenix-recovery
pio run -e rocket
```

### Unit Tests (Host Native)
```bash
pio test -e native
```

### Flash to Board
```bash
pio run -e rocket --target upload
```

## Configuration

Edit `firmware/src/config.h` to customize:
- Target coordinates
- LoRa frequency and parameters
- Servo channels and calibration
- Sensor enable/disable
- WiFi AP credentials

## Development Workflow

1. **Logic Development**: Edit files in `src/logic/` - these are hardware-free and testable on host
2. **Unit Tests**: Add tests in `tests/` and run with `pio test -e native`
3. **Integration**: Wire logic into `main.cpp` control loop
4. **Hardware Test**: Flash to ESP32 and use WiFi dashboard for validation
5. **Flight Test**: Enable `SIMULATION_MODE=1` for bench testing without hardware

## Safety Notes

- **Servos Neutral Until Guided**: Servos stay at neutral until state machine reaches DESCENT_GUIDED
- **Failsafe Always Active**: Any critical sensor failure triggers failsafe action
- **Watchdog Required**: Hardware watchdog prevents firmware lockup
- **Test Before Flight**: Always validate on bench with WiFi dashboard before flight

## Next Steps

1. **Ground Station**: Build receiver firmware + Python monitor
2. **Field Testing**: Validate sensor readings and guidance in real conditions
3. **Tune Parameters**: Adjust guidance gains based on flight tests
4. **Add Features**: Return-to-home mode, multiple target support, etc.