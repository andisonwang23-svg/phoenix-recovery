// ============================================================================
// PHOENIX RECOVERY — guidance & control configuration
// Steering law, final approach, flare, and servo geometry.
//
// ⚠ SERVO NEUTRAL / TRAVEL, GUIDANCE GAINS, AND FLARE PARAMETERS ARE
//   PLACEHOLDERS.  They REQUIRE EXPERIMENTAL CALIBRATION on the real parafoil
//   before first flight.  Safe conservative values are used until then.
// ============================================================================
#pragma once

#include <cstdint>

namespace cfg {

// ---------------------------------------------------------------------------
// Proportional steering law
// ---------------------------------------------------------------------------
constexpr float GUIDANCE_KP               = 0.02f;    // REQUIRES EXPERIMENTAL CALIBRATION
constexpr float GUIDANCE_DEADBAND_DEG     = 5.0f;     // ignore heading error below this
constexpr float MAX_STEERING_COMMAND      = 0.80f;    // ±, calibrated fraction of brake travel
constexpr float CMD_RATE_LIMIT_PER_S      = 0.5f;     // max |Δcommand| per second
constexpr uint32_t REVERSAL_GUARD_MS      = 800;      // block turn-direction flip this long

// ---------------------------------------------------------------------------
// Final approach (onto the target)
// ---------------------------------------------------------------------------
constexpr float FINAL_APPROACH_KP           = 0.010f;  // REQUIRES EXPERIMENTAL CALIBRATION
constexpr float FINAL_APPROACH_MAX_COMMAND  = 0.50f;
constexpr float FINAL_APPROACH_ALTITUDE_M   = 30.0f;   // switch to final approach below this

// ---------------------------------------------------------------------------
// Flare (parking-brake pull just before touchdown). DISABLED BY DEFAULT.
// ---------------------------------------------------------------------------
constexpr bool  FLARE_ENABLED              = false;    // spec §55 — must stay false until
                                                       //   tested on the real canopy
constexpr float FLARE_ALTITUDE_M           = 8.0f;
constexpr float FLARE_MAX_BRAKE            = 0.6f;     // fraction of brake travel
constexpr float FLARE_STALL_RELEASE_DPS    = 60.0f;    // vertical-speed slew that
                                                       //   releases the flare

// ---------------------------------------------------------------------------
// Guidance envelope / target acceptance
// ---------------------------------------------------------------------------
constexpr float MAX_GUIDANCE_RADIUS_M      = 3000.0f;  // reject targets beyond this
constexpr float MIN_GUIDANCE_ALTITUDE_M    = 12.0f;    // stop steering below this
constexpr float TARGET_ACCEPT_RADIUS_M     = 10.0f;    // "landed on target" radius

// ---------------------------------------------------------------------------
// Default landing target (NVS-persisted, overridable via web UI).
// (0.0, 0.0) is a clearly-invalid placeholder — must be set before flight.
// ---------------------------------------------------------------------------
constexpr double TARGET_LATITUDE  = 0.0;   // <-- REQUIRED before any guided flight
constexpr double TARGET_LONGITUDE = 0.0;   // <-- REQUIRED before any guided flight

// ---------------------------------------------------------------------------
// Servo geometry.  POWER FROM 5 V UBEC — ESP32 provides signal only.
//
// "Neutral" = straight glide, "brake" = pull the trailing edge down to turn.
// L/R polarity depends on how the brake lines are rigged; each servo has an
// independent REVERSED flag so a rig can be accommodated without rewiring.
// ---------------------------------------------------------------------------
constexpr float   SERVO_LEFT_NEUTRAL_US   = 1500.0f; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr float   SERVO_LEFT_MIN_US       = 1000.0f;
constexpr float   SERVO_LEFT_MAX_US       = 2000.0f;
constexpr float   SERVO_LEFT_MAX_BRAKE_US = 500.0f;  // pulse travel (from neutral)
constexpr bool    SERVO_LEFT_REVERSED     = false;

constexpr float   SERVO_RIGHT_NEUTRAL_US  = 1500.0f; // REQUIRES EXPERIMENTAL CALIBRATION
constexpr float   SERVO_RIGHT_MIN_US      = 1000.0f;
constexpr float   SERVO_RIGHT_MAX_US      = 2000.0f;
constexpr float   SERVO_RIGHT_MAX_BRAKE_US = 500.0f; // pulse travel (from neutral)
constexpr bool    SERVO_RIGHT_REVERSED    = false;

// Slew limiting / refresh
constexpr uint32_t SERVO_SLEW_RATE_US_PER_S = 800;   // REQUIRES EXPERIMENTAL CALIBRATION
constexpr uint32_t SERVO_REFRESH_MS         = 20;    // ≥50 Hz refresh (ESP32Servo)

} // namespace cfg
