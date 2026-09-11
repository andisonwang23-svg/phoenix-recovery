# PHOENIX RECOVERY — Autonomous Flight Operation

## Overview

The Phoenix Recovery system is designed for **fully autonomous parafoil recovery**. Once powered on and armed, the system requires **no WiFi, LoRa, or ground station interaction** during flight. All flight decisions are made onboard using sensor fusion and state machine logic.

---

## Autonomous Flight Sequence

```
BOOT → SELF_TEST → PAD_SAFE → ASCENT → APOGEE → DEPLOY → STABILIZE → GUIDED DESCENT → LAND
```

| Phase | What Happens Automatically |
|-------|---------------------------|
| **Launch Detection** | IMU + baro detect sustained acceleration (>8 m/s²) OR rising altitude (>5m) OR vertical speed (>5 m/s). Requires 2 of 3 signals. |
| **Apogee Detection** | Vertical speed drops below 1.5 m/s threshold |
| **Parafoil Deployment** | Timer-based deployment sequence |
| **Stabilization** | Waits for angular rates to settle + vertical speed jitter to reduce |
| **Guided Descent** | GPS computes bearing to target, proportional steering commands to servos |
| **Final Approach** | Below 30m: switches to final approach guidance mode |
| **Flare** | Below 8m (if enabled): symmetric brake for softer landing |
| **Landing Detection** | Vertical speed < 1 m/s + ground speed < 2 m/s + GPS valid |

---

## Safety Features (Always Running)

- **Failsafe** triggers on: IMU failure, baro failure, GPS timeout (5s in guided states), excessive rotation rate
- **Reversal guard** prevents rapid left-right oscillation (800ms minimum between reversals)
- **Rate limiting** on servo commands prevents abrupt movements
- **Geofence** - guidance disabled outside 3000m radius from launch
- **Hardware watchdog** - 1 second timeout, auto-reboots if main loop hangs

---

## WiFi AP (Ground Only)

WiFi is only for:
- Pre-flight configuration (set target GPS coordinates, tune guidance)
- Bench testing servos
- Post-flight data download

**During flight, WiFi is OFF** (configurable, but recommended off to save power).
