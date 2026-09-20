# PHOENIX RECOVERY — Ground Testing Procedures

## Overview

Ground testing verifies all subsystems work correctly before flight. The system includes a dedicated **Bench Test Mode** that disables launch detection and allows safe servo testing.

---

## Bench Test Mode

Enable by setting `BENCH_TEST_MODE=1` in `platformio.ini` or via build flag:

```bash
pio run -e rocket --build-flags="-DBENCH_TEST_MODE=1"
```

### What Changes in Bench Mode

| Normal Mode | Bench Mode |
|-------------|------------|
| Launch detection active | Launch detection **DISABLED** |
| Auto guidance enabled | Manual servo control only |
| Full flight sequence | Stays in PAD_SAFE |
| WiFi may be off | WiFi AP always available |

---

## Pre-Flight Checklist

### 1. Power-On Self-Test

On boot, the system runs automatic checks:

```
[INIT] IMU... OK
[INIT] Barometer... OK
[INIT] GPS... OK (waiting for fix)
[INIT] LoRa... OK
[INIT] Servos... OK
[INIT] Guidance... OK
```

All critical sensors (IMU, Baro) must pass. GPS and LoRa are non-critical for autonomous flight.

### 2. Ground-station Dashboard Access

1. Power on both the payload and the separate ground LoRa board.
2. Connect to WiFi: `PHOENIX-GROUND` (password: `phoenixground`)
3. Open browser: `http://192.168.8.1/`

The payload Wi-Fi radio is disabled. Payload status and commands cross the
LoRa link; the computer connects only to the ground board.

### 3. Configuration

Use the ground dashboard target controls to set the target coordinates over
LoRa. Other control constants remain compile-time firmware configuration.

| Parameter | Where it is set | Required |
|-----------|-------------|----------|
| Target Latitude | Ground dashboard over LoRa | **YES** |
| Target Longitude | Ground dashboard over LoRa | **YES** |
| Kp / deadband | Payload firmware configuration | No |
| Servo calibration | Payload NVS calibration | **YES** |
| Deployment / stabilization timing | Payload firmware configuration | **YES** |
| Flare enable | Payload firmware; must remain `false` until validated | **YES** |

---

## Servo Testing

### Legacy payload dashboard (development builds only)

The following interface is unavailable in the normal flight build because
payload Wi-Fi is disabled. Do not enable it for flight merely to perform a
bench test; use a dedicated controlled development build instead.

1. Navigate to `http://192.168.4.1/bench`
2. Use control buttons:

| Button | Action | Expected Result |
|--------|--------|-----------------|
| Neutral | Both servos to center | 1500μs pulse |
| Left 10% | Left servo pulls 10% | Left brake engages slightly |
| Left 25% | Left servo pulls 25% | Left brake engages more |
| Right 10% | Right servo pulls 10% | Right brake engages slightly |
| Right 25% | Right servo pulls 25% | Right brake engages more |
| Brake 10% | Both servos pull 10% | Symmetric brake (flare test) |

### Safety Limits

- Servo commands clamped to ±35% of max travel (configurable)
- Slew rate limiting prevents abrupt movements
- Emergency neutral button always available

### What to Verify

- [ ] Left servo moves in correct direction (pull = brake)
- [ ] Right servo moves in correct direction (pull = brake)
- [ ] Neutral position is centered
- [ ] No binding or mechanical interference
- [ ] Servos respond smoothly (no jitter)

---

## Sensor Testing

### Via WiFi Dashboard

Navigate to `http://192.168.4.1/bench` and click sensor test buttons.

### IMU Test

- Verify quaternion values are valid (norm ≈ 1.0)
- Check accelerometer reads ~9.8 m/s² when level
- Gyro should read near zero when stationary

### Barometer Test

- Altitude should be stable (±1m when stationary)
- Pressure should be reasonable (900-1100 hPa at sea level)

### GPS Test

- Wait for fix (requires outdoor or window view)
- 4+ satellites for 3D fix
- HDOP < 2.0 for good accuracy

### LoRa Test

- Transmit test packet
- Check RSSI on ground station (if available)

---

## Flight Simulation

For testing guidance logic without actual sensors, enable simulation mode:

```bash
pio run -e rocket --build-flags="-DSIMULATION_MODE=1"
```

This provides synthetic sensor data through the real pipeline, allowing full flight sequence testing on the bench.

---

## Serial Monitor

For detailed diagnostics, connect via USB serial at 115200 baud:

```bash
pio device monitor -b 115200
```

### Serial Commands

| Command | Action |
|---------|--------|
| `status` | Print current flight state and sensor data |
| `arm` | Arm the system (enable launch detection) |
| `disarm` | Disarm the system |
| `servo L <us>` | Set left servo to specific pulse width |
| `servo R <us>` | Set right servo to specific pulse width |
| `servo neutral` | Set both servos to neutral |
| `cal` | Calibrate ground pressure (baro zero) |
| `reset` | Reset to BOOT state |
| `help` | List available commands |

---

## Data Recording

### Flash Logging

When enabled (`FLASH_EVENT_LOG_ENABLED=1`), the system logs:

- Flight state changes
- Sensor data snapshots
- Failsafe events
- Landing detection

Logs stored in LittleFS flash filesystem, accessible via WiFi dashboard.

### RAM Ring Buffer

High-speed telemetry stored in RAM ring buffer for post-flight analysis. Survives soft resets but not power cycles.

---

## Common Issues

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| IMU fails self-test | I2C wiring wrong | Check SDA/SCL pins (GPIO47/48) |
| Baro fails self-test | Wrong I2C address | Verify BMP388 ADDR pin (0x76) |
| GPS no fix | Indoor/obstructed | Move near window or outdoors |
| Servos don't move | Wrong GPIO pins | Check servo wiring (GPIO4, GPIO6) |
| WiFi won't connect | Wrong password | Use: `parafoil` |
| Guidance not active | Target not set | Set target coordinates in config |

---

## Flight Readiness Checklist

Before flight, verify:

- [ ] All sensors pass self-test
- [ ] GPS has 3D fix with HDOP < 2.0
- [ ] Target coordinates set correctly
- [ ] Servos respond correctly in both directions
- [ ] Servo neutral position verified
- [ ] Launch detection triggers with test motion
- [ ] Failsafe triggers when sensor fails
- [ ] Battery fully charged
- [ ] All wiring secure
- [ ] No mechanical binding in brake lines
