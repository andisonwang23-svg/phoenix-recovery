// ============================================================================
// PHOENIX RECOVERY — Main Entry Point.
// ============================================================================
// Autonomous parafoil recovery system for Heltec ESP32-S3 LoRa V4.
// Boots, self-tests, calibrates, detects flight phases, and guides
// parafoil to GPS target. Configuration/monitoring is provided by the separate
// LoRa ground station; the payload Wi-Fi radio is disabled by default.
// ============================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>
#include "config.h"
#include "vehicle_state.h"

// Subsystems
#include "sensors/sensor_manager.h"
#include "estimation/state_estimator.h"
#include "control/servo_controller.h"
#include "logic/flight_coordinator.h"
#include "logic/drop_test_controller.h"
#include "logic/bench_servo_gate.h"
#include "logic/lora_airtime.h"
#include "logic/tilt_stabilizer.h"
#include "comms/lora.h"
#include "comms/wifi_manager.h"
#include "telemetry/telemetry_packet.h"
#include "safety/health_monitor.h"
#include "safety/watchdog.h"
#include "logging/flash_logger.h"
#include "logging/ring_buffer.h"

// At the configured SF7/BW125/CR4/5, the 124-byte v7 packet takes about
// 205 ms on air. A higher rate would eliminate the command receive window.
static_assert(sizeof(telemetry::TelemetryPacketV1) == 124,
              "Recalculate the LoRa airtime budget after changing telemetry");
static_assert(cfg::TELEMETRY_RATE_HZ <= 2,
              "LoRa telemetry schedule leaves no reliable ground-command receive window");

// ============================================================================
// Global State
// ============================================================================
static phoenix::VehicleState vehicle;
static sensors::SensorManager sensor_mgr;
static estimation::StateEstimator estimator;
static control::ServoController servos;
static logic::FlightCoordinator coordinator;
static logic::DropTestController drop_test_controller;
static logic::TiltStabilizer tilt_stabilizer;
static comms::LoRaTelemetry lora;
static comms::WiFiManager wifi;
static phoenix::safety::HealthMonitor health;
static phoenix::safety::HardwareWatchdog watchdog;
static phoenix::logging::FlashLogger flash_log;
static phoenix::logging::RingBuffer ring_buf;

// Timing
static uint32_t last_loop_ms = 0;
static uint32_t last_telemetry_ms = 0;
static uint32_t last_health_ms = 0;
static uint32_t last_log_ms = 0;
static uint32_t last_status_ms = 0;

// ============================================================================
// Forward Declarations
// ============================================================================
void setupSerial();
void printBanner();
bool initializeSensors();
bool initializeSubsystems();
void sendTelemetry();
void updateHealth();
void logFlightData();
void printStatus();
void updateLoRaRemoteControl();
void updateDropTestRecording();
bool armDropTest(uint32_t now);
void abortDropTest(uint32_t now, const char* reason);
uint32_t telemetryIntervalMs();
bool saveRemoteTarget(double latitude, double longitude);
void loadPersistedTarget();
bool remoteControlAllowedForState(logic::FlightState state);
bool benchServoActuationAllowed(uint32_t now);

static uint32_t last_lora_remote_poll_ms = 0;
static bool lora_remote_runtime_enabled = cfg::LORA_REMOTE_CONTROL_ENABLED;
static bool lora_remote_manual_active = false;
static uint32_t lora_remote_command_expires_ms = 0;
static float lora_remote_servo1_command = 0.0f;
static float lora_remote_servo2_command = 0.0f;
static bool lora_bench_servo_active = false;
static uint32_t lora_bench_servo_expires_ms = 0;
static bool lora_tilt_stabilizer_requested = false;
static uint32_t lora_tilt_stabilizer_expires_ms = 0;
static bool lora_tilt_stabilizer_restart_locked = false;
static uint16_t next_drop_test_id = 1;

static logic::TiltStabilizerConfig tiltStabilizerConfig() {
    logic::TiltStabilizerConfig c;
    c.kp_per_deg = cfg::TILT_STABILIZER_KP_PER_DEG;
    c.kd_per_dps = cfg::TILT_STABILIZER_KD_PER_DPS;
    c.roll_deadband_deg = cfg::TILT_STABILIZER_DEADBAND_DEG;
    c.roll_rate_deadband_dps = cfg::TILT_STABILIZER_RATE_DEADBAND_DPS;
    c.max_brake_command = cfg::TILT_STABILIZER_MAX_BRAKE_COMMAND;
    c.command_rate_limit_per_s = cfg::TILT_STABILIZER_COMMAND_RATE_PER_S;
    c.reversal_guard_ms = cfg::TILT_STABILIZER_REVERSAL_GUARD_MS;
    c.max_duration_ms = cfg::TILT_STABILIZER_MAX_DURATION_MS;
    c.max_vertical_speed_mps = cfg::TILT_STABILIZER_MAX_VERTICAL_SPEED_MPS;
    c.max_vertical_accel_mps2 = cfg::TILT_STABILIZER_MAX_VERTICAL_ACCEL_MPS2;
    c.max_roll_error_deg = cfg::TILT_STABILIZER_MAX_ROLL_ERROR_DEG;
    c.max_abs_pitch_deg = cfg::TILT_STABILIZER_MAX_ABS_PITCH_DEG;
    c.max_roll_rate_dps = cfg::TILT_STABILIZER_MAX_ROLL_RATE_DPS;
    c.correction_sign = cfg::TILT_STABILIZER_CORRECTION_SIGN;
    return c;
}

static logic::DropTestConfig dropTestConfig() {
    logic::DropTestConfig c;
    c.release_speed_mps = cfg::DROP_TEST_RELEASE_SPEED_MPS;
    c.release_altitude_loss_m = cfg::DROP_TEST_RELEASE_ALT_LOSS_M;
    c.release_confirm_ms = cfg::DROP_TEST_RELEASE_CONFIRM_MS;
    c.canopy_observation_delay_ms = cfg::DROP_TEST_CANOPY_OBSERVATION_DELAY_MS;
    c.canopy_deceleration_delta_mps = cfg::DROP_TEST_CANOPY_DECEL_DELTA_MPS;
    c.canopy_accel_signature_mps2 = cfg::DROP_TEST_CANOPY_ACCEL_SIGNATURE_MPS2;
    c.stable_descent_window_ms = cfg::DROP_TEST_STABLE_DESCENT_WINDOW_MS;
    c.stable_descent_max_vs_range_mps = cfg::DROP_TEST_STABLE_DESCENT_MAX_VS_RANGE_MPS;
    c.stable_descent_max_angular_rate_dps = cfg::DROP_TEST_STABLE_DESCENT_MAX_ANGULAR_RATE_DPS;
    c.stable_descent_min_down_speed_mps = cfg::DROP_TEST_STABLE_DESCENT_MIN_DOWN_SPEED_MPS;
    c.landing_max_vertical_speed_mps = cfg::DROP_TEST_LANDING_VS_MPS;
    c.landing_max_angular_rate_dps = cfg::DROP_TEST_LANDING_ANGULAR_RATE_DPS;
    c.landing_max_ground_speed_mps = cfg::DROP_TEST_LANDING_GROUND_SPEED_MPS;
    c.landing_max_altitude_span_m = cfg::DROP_TEST_LANDING_ALTITUDE_SPAN_M;
    c.landing_confirm_ms = cfg::DROP_TEST_LANDING_CONFIRM_MS;
    c.post_landing_record_ms = cfg::DROP_TEST_POST_LANDING_RECORD_MS;
    c.max_duration_ms = cfg::DROP_TEST_MAX_DURATION_MS;
    return c;
}

static logic::CoordinatorConfig coordinatorConfig() {
    logic::CoordinatorConfig c;
    c.state_machine.launch_confirm_ms = cfg::LAUNCH_CONFIRM_MS;
    c.state_machine.apogee_confirmation_time_ms = cfg::APOGEE_CONFIRMATION_TIME_MS;
    c.state_machine.deployment_wait_ms = cfg::DEPLOYMENT_WAIT_MS;
    c.state_machine.min_stabilization_time_ms = cfg::MIN_STABILIZATION_TIME_MS;
    c.state_machine.max_stabilization_time_ms = cfg::MAX_STABILIZATION_TIME_MS;
    c.state_machine.max_stabilization_gyro_rate_dps = cfg::MAX_STABILIZATION_GYRO_RATE;
    c.state_machine.min_descent_speed_mps = cfg::MIN_DESCENT_SPEED;
    c.state_machine.final_approach_altitude_m = cfg::FINAL_APPROACH_ALTITUDE_M;
    c.state_machine.final_approach_radius_m = cfg::FINAL_APPROACH_RADIUS_M;
    c.state_machine.gps_loss_timeout_ms = cfg::GPS_LOSS_TIMEOUT_MS;
    c.state_machine.landed_confirm_ms = cfg::LANDED_CONFIRM_MS;
    c.guidance.kp = cfg::GUIDANCE_KP;
    c.guidance.deadband_deg = cfg::GUIDANCE_DEADBAND_DEG;
    c.guidance.max_steering = cfg::MAX_STEERING_COMMAND;
    c.guidance.cmd_rate_limit_per_s = cfg::CMD_RATE_LIMIT_PER_S;
    c.guidance.reversal_guard_ms = cfg::MIN_TURN_REVERSAL_TIME_MS;
    c.guidance.final_approach_kp = cfg::FINAL_APPROACH_KP;
    c.guidance.final_approach_max_command = cfg::FINAL_APPROACH_MAX_COMMAND;
    c.guidance.final_approach_altitude_m = cfg::FINAL_APPROACH_ALTITUDE_M;
    c.guidance.flare_enabled = cfg::FLARE_ENABLED;
    c.guidance.flare_altitude_m = cfg::FLARE_ALTITUDE_M;
    c.guidance.flare_max_brake = cfg::FLARE_MAX_BRAKE;
    c.guidance.min_guidance_altitude_m = cfg::MIN_GUIDANCE_ALTITUDE_M;
    c.guidance.target_accept_radius_m = cfg::TARGET_ACCEPT_RADIUS_M;
    c.guidance.min_course_speed_mps = cfg::GPS_MIN_GROUND_SPEED_FOR_COURSE;
    c.flare_enabled = cfg::FLARE_ENABLED;
    return c;
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    setupSerial();
    printBanner();
    vehicle.reset_reason = static_cast<uint8_t>(esp_reset_reason());

    Serial.println("[BOOT] Starting PHOENIX RECOVERY...");

    // Initialize configuration
    if (!cfg::loadConfig()) {
        Serial.println("[BOOT] WARNING: Using default config");
    }

    // Target persistence must not depend on the payload web dashboard.
    loadPersistedTarget();

    if (cfg::PAYLOAD_WIFI_ENABLED) {
        if (!wifi.begin(&vehicle, &servos)) {
            Serial.println("[BOOT] WARNING: WiFi init failed; automatic retry enabled");
        }
    } else {
        WiFi.persistent(false);
        WiFi.mode(WIFI_OFF);
        vehicle.wifi_client_count = 0;
        Serial.println("[BOOT] Payload WiFi disabled; use PHOENIX-GROUND over LoRa");
    }

    // Initialize watchdog first
    phoenix::safety::WatchdogConfig wdt_cfg;
    wdt_cfg.enabled = true;
    wdt_cfg.timeout_ms = 5000;
    if (!watchdog.begin(wdt_cfg)) {
        Serial.println("[BOOT] WARNING: Watchdog init failed");
    }

    // Initialize health monitor
    phoenix::safety::HealthConfig health_cfg;
    if (!health.begin(health_cfg)) {
        Serial.println("[BOOT] WARNING: Health monitor init failed");
    }

    coordinator = logic::FlightCoordinator(coordinatorConfig());
    drop_test_controller = logic::DropTestController(dropTestConfig());
    tilt_stabilizer = logic::TiltStabilizer(tiltStabilizerConfig());
    vehicle.configuration_version = cfg::CONFIG_VERSION;

    // Initialize logging
    phoenix::logging::FlashLoggerConfig log_cfg;
    log_cfg.enabled = true;
    log_cfg.max_entries = 10000;
    if (!flash_log.begin(log_cfg)) {
        Serial.println("[BOOT] WARNING: Flash logger init failed");
    }

    phoenix::logging::RingBufferConfig ring_cfg;
    ring_cfg.enabled = true;
    ring_cfg.max_entries = 500;
    if (!ring_buf.begin(ring_cfg)) {
        Serial.println("[BOOT] WARNING: Ring buffer init failed");
    }

    // Initialize sensors
    if (!initializeSensors()) {
        Serial.println("[BOOT] ERROR: Sensor initialization failed");
        vehicle.failure_code = logic::FailCode::FAIL_SENSOR_DATA;
    }

    // Initialize vehicle state
    vehicle.flight_state = logic::FlightState::BOOT;
    vehicle.guidance_mode = logic::GuidanceMode::MODE_DISABLED;
    vehicle.timestamp_ms = millis();

    // Initialize subsystems
    if (!initializeSubsystems()) {
        Serial.println("[BOOT] WARNING: Some subsystems failed to initialize");
    }

    // Start logging at boot so self-test, launch detection, and every later
    // transition share one explainable timeline.
    if (!flash_log.isLogging()) flash_log.startFlightLog();
    if (flash_log.isLogging()) flash_log.logEvent("PHOENIX_RECOVERY_BOOT");

    last_loop_ms = millis();
    Serial.println("[BOOT] PHOENIX RECOVERY ready");
    Serial.printf("[BOOT] Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println("[BOOT] ============================================");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    uint32_t now = millis();
    uint32_t dt = now - last_loop_ms;

    // Feed watchdog
    watchdog.feed();

    // Update vehicle timestamp
    vehicle.timestamp_ms = now;

    // ---- Sensor Read (every loop) ----
    sensor_mgr.update(vehicle);
    updateLoRaRemoteControl();

    // Edge-triggered sensor and actuator events; communications are
    // intentionally absent because their loss never changes flight control.
    static bool previous_gps_valid = false;
    static bool previous_imu_valid = true;
    static bool previous_baro_valid = true;
    static bool servo_saturated_logged = false;
    if (previous_gps_valid && !vehicle.gps_valid) { ring_buf.pushEvent("GPS_LOST"); flash_log.logEvent("GPS_LOST"); }
    if (!previous_gps_valid && vehicle.gps_valid) { ring_buf.pushEvent("GPS_RECOVERED"); flash_log.logEvent("GPS_RECOVERED"); }
    if (previous_imu_valid && !vehicle.imu_valid) { ring_buf.pushEvent("IMU_FAILURE"); flash_log.logEvent("IMU_FAILURE"); }
    if (previous_baro_valid && !vehicle.barometer_valid) { ring_buf.pushEvent("BAROMETER_FAILURE"); flash_log.logEvent("BAROMETER_FAILURE"); }
    previous_gps_valid = vehicle.gps_valid;
    previous_imu_valid = vehicle.imu_valid;
    previous_baro_valid = vehicle.barometer_valid;

    // ---- State Estimation ----
    float dt_s = dt / 1000.0f;
    estimator.update(vehicle, dt_s);
    if (vehicle.altitude_agl_m > vehicle.max_altitude_agl_m) {
        vehicle.max_altitude_agl_m = vehicle.altitude_agl_m;
    }

    updateDropTestRecording();

    // ---- Flight coordinator: sole flight-state and brake-request owner ----
    logic::CoordinatorInput coordinator_in;
    coordinator_in.now_ms = now;
    coordinator_in.dt_s = dt_s;
    coordinator_in.latitude = vehicle.latitude;
    coordinator_in.longitude = vehicle.longitude;
    coordinator_in.target_latitude = vehicle.target_latitude;
    coordinator_in.target_longitude = vehicle.target_longitude;
    coordinator_in.altitude_agl_m = vehicle.altitude_agl_m;
    coordinator_in.vertical_speed_mps = vehicle.vertical_speed_mps;
    coordinator_in.vertical_accel_mps2 = vehicle.vertical_accel_mps2;
    coordinator_in.ground_speed_mps = vehicle.ground_speed_mps;
    coordinator_in.gps_course_deg = vehicle.gps_course_deg;
    coordinator_in.angular_rate_dps = vehicle.angular_rate_dps;
    coordinator_in.gps_valid = vehicle.gps_valid;
    coordinator_in.imu_valid = vehicle.imu_valid;
    coordinator_in.barometer_valid = vehicle.barometer_valid;
    coordinator_in.servo_valid = servos.isHealthy();
    coordinator_in.remote_manual_active = lora_remote_manual_active;
    coordinator_in.remote_servo1_brake = lora_remote_servo1_command;
    coordinator_in.remote_servo2_brake = lora_remote_servo2_command;
    logic::CoordinatorOutput command = coordinator.step(coordinator_in);

    vehicle.flight_state = command.state;
    vehicle.guidance_mode = command.mode;
    vehicle.failure_code = command.failure;
    vehicle.target_valid = command.target_valid;
    vehicle.guidance_ready = command.guidance_ready;
    vehicle.gps_fallback_active = command.gps_fallback_active;
    vehicle.imu_fallback_active = command.imu_fallback_active;
    vehicle.barometer_fallback_active = command.barometer_fallback_active;
    vehicle.degraded_guidance = command.degraded_guidance;
    vehicle.launch_readiness_ok = command.launch_readiness_ok;
    vehicle.preflight_launch_warning = command.preflight_launch_warning;
    vehicle.lora_remote_enabled = lora_remote_runtime_enabled;
    vehicle.lora_remote_command_allowed = command.remote_command_allowed;
    vehicle.lora_remote_manual_active = command.remote_manual_active;
    vehicle.target_bearing_deg = command.target_bearing_deg;
    vehicle.heading_error_deg = command.heading_error_deg;
    vehicle.distance_to_target_m = command.distance_to_target_m;
    vehicle.requested_left_servo_command = command.requested_left_brake;
    vehicle.requested_right_servo_command = command.requested_right_brake;
    if (vehicle.drop_test_neutral_lock) {
        vehicle.requested_left_servo_command = 0.0f;
        vehicle.requested_right_servo_command = 0.0f;
    }
    const logic::TransitionTimestamps& times = coordinator.timestamps();
    vehicle.state_entry_ms = times.state_entry_ms;
    vehicle.launch_ms = times.launch_ms;
    vehicle.apogee_ms = times.apogee_ms;
    vehicle.deployment_wait_start_ms = times.deployment_wait_ms;
    vehicle.stabilization_start_ms = times.stabilization_ms;
    vehicle.guidance_start_ms = times.guidance_ms;
    vehicle.final_approach_start_ms = times.final_approach_ms;
    vehicle.flare_start_ms = times.flare_ms;
    vehicle.landed_ms = times.landed_ms;
    vehicle.failsafe_ms = times.failsafe_ms;

    if (command.state_changed) {
        const char* event = logic::flightStateName(command.state);
        switch (command.state) {
            case logic::FlightState::APOGEE_CONFIRMED: event = "APOGEE_CONFIRMED"; break;
            case logic::FlightState::DEPLOYMENT_WAIT: event = "DEPLOYMENT_WAIT_STARTED"; break;
            case logic::FlightState::PARAFOIL_STABILIZATION: event = "STABILIZATION_STARTED"; break;
            case logic::FlightState::GUIDED_DESCENT: event = "STABILIZATION_COMPLETE"; break;
            case logic::FlightState::FINAL_APPROACH: event = "FINAL_APPROACH_STARTED"; break;
            case logic::FlightState::FLARE: event = "FLARE_STARTED"; break;
            case logic::FlightState::LANDED: event = "LANDED"; break;
            case logic::FlightState::FAILSAFE_DESCENT: event = "FAILSAFE_ENTERED"; break;
            default: break;
        }
        ring_buf.pushEvent(event);
        flash_log.logEvent(event);
        if (command.state == logic::FlightState::GUIDED_DESCENT) {
            ring_buf.pushEvent("GUIDANCE_STARTED");
            flash_log.logEvent("GUIDANCE_STARTED");
        }
    }
    static bool previous_gps_fallback = false;
    static bool previous_imu_fallback = false;
    static bool previous_baro_fallback = false;
    if (!previous_gps_fallback && command.gps_fallback_active) { ring_buf.pushEvent("GPS_FALLBACK_ACTIVE"); flash_log.logEvent("GPS_FALLBACK_ACTIVE"); }
    if (previous_gps_fallback && !command.gps_fallback_active) { ring_buf.pushEvent("GPS_FALLBACK_CLEARED"); flash_log.logEvent("GPS_FALLBACK_CLEARED"); }
    if (!previous_imu_fallback && command.imu_fallback_active) { ring_buf.pushEvent("IMU_FALLBACK_ACTIVE"); flash_log.logEvent("IMU_FALLBACK_ACTIVE"); }
    if (previous_imu_fallback && !command.imu_fallback_active) { ring_buf.pushEvent("IMU_FALLBACK_CLEARED"); flash_log.logEvent("IMU_FALLBACK_CLEARED"); }
    if (!previous_baro_fallback && command.barometer_fallback_active) { ring_buf.pushEvent("BAROMETER_FALLBACK_ACTIVE"); flash_log.logEvent("BAROMETER_FALLBACK_ACTIVE"); }
    if (previous_baro_fallback && !command.barometer_fallback_active) { ring_buf.pushEvent("BAROMETER_FALLBACK_CLEARED"); flash_log.logEvent("BAROMETER_FALLBACK_CLEARED"); }
    previous_gps_fallback = command.gps_fallback_active;
    previous_imu_fallback = command.imu_fallback_active;
    previous_baro_fallback = command.barometer_fallback_active;
    static bool previous_preflight_launch_warning = false;
    if (!previous_preflight_launch_warning && command.preflight_launch_warning) {
        ring_buf.pushEvent("LAUNCH_ATTEMPT_NOT_READY");
        flash_log.logEvent("LAUNCH_ATTEMPT_NOT_READY");
    }
    previous_preflight_launch_warning = command.preflight_launch_warning;

    // ---- Explicit IMU/barometer ground roll-stabilization test ----
    // This controller is deliberately outside the flight coordinator and is
    // accepted only through the same prelaunch service gate as an unloaded
    // bench command. It cannot run after launch, while armed, or during a drop
    // recording. GPS is not consulted.
    const bool tilt_preflight_state =
        command.state == logic::FlightState::SELF_TEST ||
        command.state == logic::FlightState::PAD_SAFE;
    logic::TiltStabilizerInput tilt_in;
    tilt_in.now_ms = now;
    tilt_in.dt_s = dt_s;
    tilt_in.command_fresh = lora_tilt_stabilizer_requested &&
        static_cast<int32_t>(lora_tilt_stabilizer_expires_ms - now) > 0;
    tilt_in.preflight_allowed = tilt_preflight_state && benchServoActuationAllowed(now);
    tilt_in.imu_valid = vehicle.imu_valid && vehicle.imu_age_ms <= cfg::SENSOR_TIMEOUT_MS;
    tilt_in.barometer_valid = vehicle.barometer_valid &&
                             vehicle.baro_age_ms <= cfg::SENSOR_TIMEOUT_MS;
    tilt_in.servo_valid = servos.isHealthy();
    tilt_in.roll_deg = vehicle.roll_deg;
    tilt_in.pitch_deg = vehicle.pitch_deg;
    tilt_in.roll_rate_dps = vehicle.gyro_x_dps;
    tilt_in.vertical_speed_mps = vehicle.vertical_speed_mps;
    tilt_in.vertical_accel_mps2 = vehicle.vertical_accel_mps2;
    const logic::TiltStabilizerOutput tilt_output = tilt_stabilizer.update(tilt_in);
    if (tilt_output.status == logic::TiltStabilizerStatus::DURATION_EXPIRED) {
        lora_tilt_stabilizer_requested = false;
        lora_tilt_stabilizer_restart_locked = true;
    }
    vehicle.tilt_stabilizer_requested = lora_tilt_stabilizer_requested;
    vehicle.tilt_stabilizer_active = tilt_output.active;
    vehicle.tilt_stabilizer_neutral = tilt_output.immediate_neutral ||
                                      fabsf(tilt_output.signed_command) < 0.0005f;
    vehicle.tilt_stabilizer_status = tilt_output.status;
    vehicle.tilt_reference_roll_deg = tilt_output.reference_roll_deg;
    vehicle.tilt_roll_error_deg = tilt_output.roll_error_deg;
    vehicle.tilt_control_command = tilt_output.signed_command;

    static bool previous_tilt_active = false;
    if (!previous_tilt_active && tilt_output.active) {
        ring_buf.pushEvent("TILT_STABILIZER_ACTIVE");
        flash_log.logEvent("TILT_STABILIZER_ACTIVE");
    } else if (previous_tilt_active && !tilt_output.active) {
        ring_buf.pushEvent("TILT_STABILIZER_NEUTRAL");
        flash_log.logEvent("TILT_STABILIZER_NEUTRAL");
    }
    previous_tilt_active = tilt_output.active;

    health.update(vehicle);
    const bool ground_servo_test_active =
        cfg::PAYLOAD_WIFI_ENABLED && wifi.isServoTestActive();

    const bool bench_allowed_now = benchServoActuationAllowed(now);
    if (lora_bench_servo_active && !bench_allowed_now) {
        lora_bench_servo_active = false;
        lora_remote_servo1_command = 0.0f;
        lora_remote_servo2_command = 0.0f;
        servos.emergencyNeutral();
        ring_buf.pushEvent("LORA_BENCH_SERVO_INTERLOCK");
        flash_log.logEvent("LORA_BENCH_SERVO_INTERLOCK");
    }

    // An explicit, time-limited, prelaunch bench command may operate even when
    // sensors placed the flight coordinator in failsafe. It is gated by launch,
    // arming, recording, servo health, and boot-time window—not sensor health.
    if (lora_bench_servo_active && bench_allowed_now) {
        servos.setBenchServoCommands(lora_remote_servo1_command,
                                     lora_remote_servo2_command);
        servos.update();
    } else if (command.failsafe_active || command.state == logic::FlightState::LANDED ||
        vehicle.drop_test_neutral_lock) {
        servos.emergencyNeutral();
    } else if (tilt_output.active) {
        servos.setBrakeCommands(tilt_output.left_brake, tilt_output.right_brake);
        servos.update();
    } else if (lora_tilt_stabilizer_requested) {
        // A requested test with a blocked sensor/motion gate must never fall
        // through to another actuator owner.
        servos.emergencyNeutral();
    } else if (ground_servo_test_active &&
               (command.state == logic::FlightState::PAD_SAFE ||
                command.state == logic::FlightState::SELF_TEST)) {
        servos.update();
    } else if (command.remote_manual_active) {
        servos.setServoCommands(command.requested_left_brake, command.requested_right_brake);
        servos.update();
    } else {
        servos.setBrakeCommands(command.requested_left_brake, command.requested_right_brake);
        servos.update();
    }
    // Publish the controller's actual outputs for telemetry and the dashboard.
    vehicle.lora_bench_servo_active = lora_bench_servo_active;
    vehicle.servo_healthy = servos.isHealthy();
    vehicle.left_servo_command = servos.getLeftCommand();
    vehicle.right_servo_command = servos.getRightCommand();
    vehicle.left_servo_us = servos.getLeftUs();
    vehicle.right_servo_us = servos.getRightUs();
    vehicle.left_servo_angle_deg = servos.getLeftAngleDeg();
    vehicle.right_servo_angle_deg = servos.getRightAngleDeg();
    vehicle.left_servo_turn_deg = servos.getLeftTurnDeg();
    vehicle.right_servo_turn_deg = servos.getRightTurnDeg();
    vehicle.servo_last_update_ms = now;
    const bool saturated = command.servo_saturated;
    if (saturated && !servo_saturated_logged) { ring_buf.pushEvent("SERVO_SATURATION"); flash_log.logEvent("SERVO_SATURATION"); }
    servo_saturated_logged = saturated;
    if (command.close_log && flash_log.isLogging()) {
        // Preserve the neutralized actuator outputs and final GPS fix before
        // sealing the log header.
        flash_log.logTelemetry(vehicle);
        flash_log.endFlightLog();
    }

    // ---- Telemetry (configurable rate) ----
    if (now - last_telemetry_ms >= telemetryIntervalMs()) {
        sendTelemetry();
        last_telemetry_ms = now;
    }

    // ---- Health Update (1 Hz) ----
    if (now - last_health_ms >= 1000) {
        updateHealth();
        last_health_ms = now;
    }

    // ---- Logging (configurable) ----
    if (now - last_log_ms >= (1000 / cfg::LOG_RATE_HZ)) {
        logFlightData();
        last_log_ms = now;
    }

    // ---- Optional payload WiFi dashboard ----
    if (cfg::PAYLOAD_WIFI_ENABLED) wifi.update();

    // Periodic hardware diagnostics are intentionally concise enough to leave
    // enabled in bench builds. They verify that initialized devices continue
    // producing fresh data instead of merely acknowledging on the bus.
    if (now - last_status_ms >= 2000) {
        printStatus();
        last_status_ms = now;
    }

    // ---- Loop Timing ----
    vehicle.loop_time_us = (micros() - (now * 1000)); // Approximate
    vehicle.loop_rate_hz = 1000.0f / max(dt, (uint32_t)1);

    last_loop_ms = now;

    // Yield to the ESP32 idle tasks and, when enabled, its WiFi/TCP tasks.
    delay(1);
}

bool remoteControlAllowedForState(logic::FlightState state) {
    return state == logic::FlightState::GUIDED_DESCENT ||
           state == logic::FlightState::FINAL_APPROACH;
}

bool benchServoActuationAllowed(uint32_t now) {
    (void)now;
    logic::BenchServoGateInput gate;
    gate.flight_state = vehicle.flight_state;
    gate.launch_ms = vehicle.launch_ms;
    gate.drop_test_recording = vehicle.drop_test_recording;
    gate.armed = vehicle.armed;
    gate.servo_healthy = servos.isHealthy();
    return logic::benchServoTestAllowed(gate);
}

static void publishDropTestOutput(const logic::DropTestOutput& out) {
    vehicle.drop_test_state = out.state;
    vehicle.drop_test_recording = out.recording;
    vehicle.drop_test_neutral_lock = out.neutral_lock;
    vehicle.drop_test_armed_ms = out.armed_ms;
    vehicle.drop_test_release_onset_ms = out.release_onset_ms;
    vehicle.drop_test_release_confirm_ms = out.release_confirm_ms;
    vehicle.drop_test_canopy_signature_ms = out.canopy_signature_ms;
    vehicle.drop_test_stable_descent_ms = out.stable_descent_ms;
    vehicle.drop_test_landing_candidate_ms = out.landing_candidate_ms;
    vehicle.drop_test_landing_confirm_ms = out.landing_confirm_ms;
    vehicle.drop_test_log_closed_ms = out.log_closed_ms;
    // The recorder arm is a live interlock, not a permanent latch. Keeping it
    // synchronized here makes abort/completion release later bench testing.
    vehicle.armed = out.recording;
    vehicle.armed_ms = out.recording ? out.armed_ms : 0;

    struct EventName { uint16_t bit; const char* name; };
    static const EventName names[] = {
        {logic::DROP_EVENT_RELEASE_ONSET, "DROP_TEST_RELEASE_ONSET"},
        {logic::DROP_EVENT_RELEASE_REJECTED, "DROP_TEST_RELEASE_CANDIDATE_REJECTED"},
        {logic::DROP_EVENT_RELEASE_CONFIRMED, "DROP_TEST_RELEASE_CONFIRMED"},
        {logic::DROP_EVENT_CANOPY_SIGNATURE, "DROP_TEST_CANOPY_SIGNATURE_SUSPECTED"},
        {logic::DROP_EVENT_STABLE_DESCENT, "DROP_TEST_STABLE_DESCENT_OBSERVED"},
        {logic::DROP_EVENT_LANDING_CANDIDATE, "DROP_TEST_LANDING_CANDIDATE"},
        {logic::DROP_EVENT_LANDING_REJECTED, "DROP_TEST_LANDING_CANDIDATE_REJECTED"},
        {logic::DROP_EVENT_LANDING_CONFIRMED, "DROP_TEST_LANDING_CONFIRMED"},
        {logic::DROP_EVENT_TIMEOUT, "DROP_TEST_TIMEOUT"},
        {logic::DROP_EVENT_ABORTED, "DROP_TEST_ABORTED"},
        {logic::DROP_EVENT_COMPLETE, "DROP_TEST_COMPLETE"}
    };
    for (const auto& item : names) {
        if ((out.events & item.bit) != 0U) {
            ring_buf.pushEvent(item.name);
            flash_log.logEvent(item.name);
        }
    }

    if (out.close_log && flash_log.isLogging()) {
        flash_log.logTelemetry(vehicle);
        flash_log.endFlightLog();
    }
}

bool armDropTest(uint32_t now) {
    if (!cfg::DROP_TEST_MODE_ENABLED) return false;
    const bool terminal =
        vehicle.drop_test_state == phoenix::DropTestState::IDLE ||
        vehicle.drop_test_state == phoenix::DropTestState::TEST_COMPLETE ||
        vehicle.drop_test_state == phoenix::DropTestState::TEST_ABORTED;
    const bool ready = terminal &&
        vehicle.imu_valid &&
        vehicle.barometer_valid &&
        servos.isHealthy() &&
        (!vehicle.battery_voltage_valid || vehicle.battery_voltage_v >= cfg::DROP_TEST_MIN_BATTERY_V) &&
        vehicle.failure_code == logic::FailCode::FAIL_NONE;
    if (!ready) {
        vehicle.drop_test_rejected_count++;
        ring_buf.pushEvent("DROP_TEST_ARM_REJECTED");
        flash_log.logEvent("DROP_TEST_ARM_REJECTED");
        return false;
    }

    if (flash_log.isLogging()) {
        flash_log.logEvent("DROP_TEST_PREARM_LOG_CLOSED");
        flash_log.endFlightLog();
    }
    if (!flash_log.startFlightLog()) {
        vehicle.drop_test_rejected_count++;
        ring_buf.pushEvent("DROP_TEST_LOG_OPEN_FAILED");
        return false;
    }
    vehicle.drop_test_id = next_drop_test_id++;
    if (next_drop_test_id == 0) next_drop_test_id = 1;
    const logic::DropTestOutput armed =
        drop_test_controller.arm(now, vehicle.altitude_agl_m);
    publishDropTestOutput(armed);
    // The new controller is intentionally stricter: an inert recording run is
    // always neutral-locked, regardless of radio or dashboard state.
    vehicle.drop_test_neutral_lock = true;
    servos.emergencyNeutral();
    ring_buf.pushEvent("DROP_TEST_ARMED");
    flash_log.logEvent("DROP_TEST_ARMED");
    flash_log.logTelemetry(vehicle);
    return true;
}

void abortDropTest(uint32_t now, const char* reason) {
    lora_remote_manual_active = false;
    lora_bench_servo_active = false;
    lora_remote_servo1_command = 0.0f;
    lora_remote_servo2_command = 0.0f;
    servos.emergencyNeutral();

    if (vehicle.drop_test_recording) {
        // Record the operator/system-specific reason while the log is still
        // open; publishDropTestOutput then records the generic abort marker,
        // final neutral snapshot, and seals the file.
        ring_buf.pushEvent(reason);
        flash_log.logEvent(reason);
        const logic::DropTestOutput aborted = drop_test_controller.abort(now);
        publishDropTestOutput(aborted);
    }
}

void updateDropTestRecording() {
    if (!vehicle.drop_test_recording) return;
    logic::DropTestInput input;
    input.now_ms = vehicle.timestamp_ms;
    input.altitude_agl_m = vehicle.altitude_agl_m;
    input.vertical_speed_mps = vehicle.vertical_speed_mps;
    input.vertical_accel_mps2 = vehicle.vertical_accel_mps2;
    input.angular_rate_dps = vehicle.angular_rate_dps;
    input.ground_speed_mps = vehicle.ground_speed_mps;
    input.barometer_valid = vehicle.barometer_valid;
    input.imu_valid = vehicle.imu_valid;
    input.gps_valid = vehicle.gps_valid;
    publishDropTestOutput(drop_test_controller.step(input));
}

bool saveRemoteTarget(double latitude, double longitude) {
    if (!isfinite(latitude) || !isfinite(longitude)) return false;
    if (latitude < -90.0 || latitude > 90.0) return false;
    if (longitude < -180.0 || longitude > 180.0) return false;
    if (fabs(latitude) < 1e-9 && fabs(longitude) < 1e-9) return false;

    Preferences targetPrefs;
    if (!targetPrefs.begin("target_cfg", false)) return false;
    targetPrefs.putDouble("latitude", latitude);
    targetPrefs.putDouble("longitude", longitude);
    targetPrefs.end();
    return true;
}

void loadPersistedTarget() {
    Preferences targetPrefs;
    if (targetPrefs.begin("target_cfg", true)) {
        vehicle.target_latitude =
            targetPrefs.getDouble("latitude", cfg::TARGET_LATITUDE);
        vehicle.target_longitude =
            targetPrefs.getDouble("longitude", cfg::TARGET_LONGITUDE);
        targetPrefs.end();
    } else {
        vehicle.target_latitude = cfg::TARGET_LATITUDE;
        vehicle.target_longitude = cfg::TARGET_LONGITUDE;
    }
}

void updateLoRaRemoteControl() {
    const uint32_t now = millis();
    vehicle.lora_remote_enabled = lora_remote_runtime_enabled;
    vehicle.lora_remote_link_active =
        vehicle.lora_remote_last_rx_ms != 0 &&
        now - vehicle.lora_remote_last_rx_ms <= cfg::LORA_REMOTE_COMMAND_TIMEOUT_MS;
    vehicle.lora_remote_command_age_ms =
        vehicle.lora_remote_last_rx_ms == 0 ? UINT32_MAX : now - vehicle.lora_remote_last_rx_ms;

    if (lora_remote_manual_active &&
        static_cast<int32_t>(now - lora_remote_command_expires_ms) >= 0) {
        lora_remote_manual_active = false;
        lora_remote_servo1_command = 0.0f;
        lora_remote_servo2_command = 0.0f;
        ring_buf.pushEvent("LORA_REMOTE_EXPIRED");
        flash_log.logEvent("LORA_REMOTE_EXPIRED");
    }
    if (lora_bench_servo_active &&
        static_cast<int32_t>(now - lora_bench_servo_expires_ms) >= 0) {
        lora_bench_servo_active = false;
        lora_remote_servo1_command = 0.0f;
        lora_remote_servo2_command = 0.0f;
        servos.emergencyNeutral();
        ring_buf.pushEvent("LORA_BENCH_SERVO_NEUTRAL");
        flash_log.logEvent("LORA_BENCH_SERVO_NEUTRAL");
    }
    if (lora_tilt_stabilizer_requested &&
        static_cast<int32_t>(now - lora_tilt_stabilizer_expires_ms) >= 0) {
        lora_tilt_stabilizer_requested = false;
        tilt_stabilizer.disarm(logic::TiltStabilizerStatus::COMMAND_TIMEOUT);
        servos.emergencyNeutral();
        ring_buf.pushEvent("TILT_STABILIZER_COMMAND_TIMEOUT");
        flash_log.logEvent("TILT_STABILIZER_COMMAND_TIMEOUT");
    }

    if (!cfg::LORA_REMOTE_CONTROL_ENABLED || !lora_remote_runtime_enabled || !lora.isHealthy()) {
        vehicle.lora_remote_manual_active = false;
        vehicle.lora_remote_servo1_command = 0.0f;
        vehicle.lora_remote_servo2_command = 0.0f;
        return;
    }
    if (now - last_lora_remote_poll_ms < cfg::LORA_REMOTE_POLL_INTERVAL_MS) return;
    last_lora_remote_poll_ms = now;

    comms::LoRaRemoteCommand remote;
    if (!lora.pollRemoteCommand(remote)) return;

    vehicle.lora_remote_last_rx_ms = now;
    vehicle.lora_remote_sequence = remote.sequence;
    vehicle.lora_rssi = static_cast<int8_t>(remote.rssi);
    vehicle.lora_snr = remote.snr;
    vehicle.drop_test_last_ground_command_ms = now;

    switch (remote.type) {
        case comms::LoRaRemoteCommandType::PING:
            vehicle.lora_remote_accepted_count++;
            ring_buf.pushEvent("LORA_REMOTE_PING");
            break;
        case comms::LoRaRemoteCommandType::SET_TARGET:
            if (vehicle.drop_test_recording) {
                vehicle.lora_remote_rejected_count++;
                ring_buf.pushEvent("LORA_TARGET_REJECTED_DROP_ACTIVE");
                flash_log.logEvent("LORA_TARGET_REJECTED_DROP_ACTIVE");
                break;
            }
            if (saveRemoteTarget(remote.target_latitude, remote.target_longitude)) {
                vehicle.target_latitude = remote.target_latitude;
                vehicle.target_longitude = remote.target_longitude;
                vehicle.lora_remote_accepted_count++;
                ring_buf.pushEvent("LORA_REMOTE_TARGET_SET");
                flash_log.logEvent("LORA_REMOTE_TARGET_SET");
            } else {
                vehicle.lora_remote_rejected_count++;
                ring_buf.pushEvent("LORA_REMOTE_TARGET_REJECTED");
            }
            break;
        case comms::LoRaRemoteCommandType::NEUTRAL:
            lora_remote_manual_active = false;
            lora_bench_servo_active = false;
            lora_tilt_stabilizer_requested = false;
            lora_tilt_stabilizer_restart_locked = false;
            tilt_stabilizer.disarm();
            lora_remote_servo1_command = 0.0f;
            lora_remote_servo2_command = 0.0f;
            vehicle.lora_remote_accepted_count++;
            ring_buf.pushEvent("LORA_REMOTE_NEUTRAL");
            flash_log.logEvent("LORA_REMOTE_NEUTRAL");
            break;
        case comms::LoRaRemoteCommandType::BENCH_SERVO: {
            const bool bench_safe = cfg::LORA_BENCH_SERVO_TEST_ENABLED &&
                benchServoActuationAllowed(now);
            if (!bench_safe) {
                lora_bench_servo_active = false;
                lora_remote_servo1_command = 0.0f;
                lora_remote_servo2_command = 0.0f;
                vehicle.lora_remote_rejected_count++;
                ring_buf.pushEvent("LORA_BENCH_SERVO_REJECTED");
                break;
            }
            lora_remote_servo1_command = constrain(remote.servo1_command,
                                                    -cfg::LORA_BENCH_MAX_SERVO_COMMAND,
                                                    cfg::LORA_BENCH_MAX_SERVO_COMMAND);
            lora_remote_servo2_command = constrain(remote.servo2_command,
                                                    -cfg::LORA_BENCH_MAX_SERVO_COMMAND,
                                                    cfg::LORA_BENCH_MAX_SERVO_COMMAND);
            lora_remote_manual_active = false;
            lora_tilt_stabilizer_requested = false;
            lora_tilt_stabilizer_restart_locked = false;
            tilt_stabilizer.disarm();
            lora_bench_servo_active = true;
            lora_bench_servo_expires_ms = now + cfg::LORA_BENCH_SERVO_TIMEOUT_MS;
            vehicle.lora_remote_accepted_count++;
            ring_buf.pushEvent("LORA_BENCH_SERVO_ACTIVE");
            flash_log.logEvent("LORA_BENCH_SERVO_ACTIVE");
            break;
        }
        case comms::LoRaRemoteCommandType::ARM_DROP_TEST:
            lora_tilt_stabilizer_requested = false;
            tilt_stabilizer.disarm();
            if (armDropTest(now)) {
                vehicle.lora_remote_accepted_count++;
            } else {
                vehicle.lora_remote_rejected_count++;
            }
            break;
        case comms::LoRaRemoteCommandType::ABORT_DROP_TEST:
            abortDropTest(now, "DROP_TEST_ABORTED_BY_GROUND");
            vehicle.lora_remote_accepted_count++;
            ring_buf.pushEvent("LORA_ABORT_NEUTRAL");
            flash_log.logEvent("LORA_ABORT_NEUTRAL");
            break;
        case comms::LoRaRemoteCommandType::REQUEST_LOG_INDEX:
            vehicle.lora_remote_accepted_count++;
            ring_buf.pushEvent("LORA_LOG_INDEX_REQUEST");
            flash_log.logEvent("LORA_LOG_INDEX_REQUEST");
            break;
        case comms::LoRaRemoteCommandType::CANCEL_LOG_TRANSFER:
            vehicle.lora_remote_accepted_count++;
            ring_buf.pushEvent("LORA_LOG_TRANSFER_CANCEL");
            flash_log.logEvent("LORA_LOG_TRANSFER_CANCEL");
            break;
        case comms::LoRaRemoteCommandType::MANUAL_BRAKE: {
            const bool safe_state = remoteControlAllowedForState(vehicle.flight_state) &&
                !vehicle.drop_test_recording &&
                vehicle.failure_code == logic::FailCode::FAIL_NONE &&
                servos.isHealthy() &&
                (vehicle.imu_valid || vehicle.barometer_valid);
            if (!safe_state) {
                lora_remote_manual_active = false;
                lora_remote_servo1_command = 0.0f;
                lora_remote_servo2_command = 0.0f;
                vehicle.lora_remote_rejected_count++;
                ring_buf.pushEvent("LORA_REMOTE_MANUAL_REJECTED");
                flash_log.logEvent("LORA_REMOTE_MANUAL_REJECTED");
                break;
            }
            lora_remote_servo1_command = constrain(remote.servo1_command,
                                                   -cfg::LORA_REMOTE_MAX_BRAKE_COMMAND,
                                                   cfg::LORA_REMOTE_MAX_BRAKE_COMMAND);
            lora_remote_servo2_command = constrain(remote.servo2_command,
                                                   -cfg::LORA_REMOTE_MAX_BRAKE_COMMAND,
                                                   cfg::LORA_REMOTE_MAX_BRAKE_COMMAND);
            lora_remote_manual_active = true;
            lora_tilt_stabilizer_requested = false;
            tilt_stabilizer.disarm();
            lora_remote_command_expires_ms = now + cfg::LORA_REMOTE_COMMAND_TIMEOUT_MS;
            vehicle.lora_remote_accepted_count++;
            ring_buf.pushEvent("LORA_REMOTE_MANUAL_ACTIVE");
            break;
        }
        case comms::LoRaRemoteCommandType::START_TILT_STABILIZER: {
            const bool preflight_state =
                vehicle.flight_state == logic::FlightState::SELF_TEST ||
                vehicle.flight_state == logic::FlightState::PAD_SAFE;
            const bool safe = cfg::TILT_STABILIZER_TEST_ENABLED &&
                !lora_tilt_stabilizer_restart_locked &&
                preflight_state && benchServoActuationAllowed(now) &&
                vehicle.imu_valid && vehicle.barometer_valid &&
                vehicle.imu_age_ms <= cfg::SENSOR_TIMEOUT_MS &&
                vehicle.baro_age_ms <= cfg::SENSOR_TIMEOUT_MS;
            if (!safe) {
                lora_tilt_stabilizer_requested = false;
                tilt_stabilizer.disarm(logic::TiltStabilizerStatus::INTERLOCK);
                servos.emergencyNeutral();
                vehicle.lora_remote_rejected_count++;
                ring_buf.pushEvent("TILT_STABILIZER_REJECTED");
                break;
            }
            lora_remote_manual_active = false;
            lora_bench_servo_active = false;
            if (!tilt_stabilizer.isArmed()) {
                tilt_stabilizer.arm(now, vehicle.roll_deg);
                ring_buf.pushEvent("TILT_STABILIZER_ARMED");
                flash_log.logEvent("TILT_STABILIZER_ARMED");
            }
            lora_tilt_stabilizer_requested = true;
            lora_tilt_stabilizer_expires_ms =
                now + cfg::TILT_STABILIZER_COMMAND_TIMEOUT_MS;
            vehicle.lora_remote_accepted_count++;
            break;
        }
        case comms::LoRaRemoteCommandType::STOP_TILT_STABILIZER:
            lora_tilt_stabilizer_requested = false;
            lora_tilt_stabilizer_restart_locked = false;
            tilt_stabilizer.disarm();
            servos.emergencyNeutral();
            vehicle.lora_remote_accepted_count++;
            ring_buf.pushEvent("TILT_STABILIZER_STOPPED");
            flash_log.logEvent("TILT_STABILIZER_STOPPED");
            break;
        case comms::LoRaRemoteCommandType::DISABLE_REMOTE:
            lora_remote_runtime_enabled = false;
            lora_remote_manual_active = false;
            lora_bench_servo_active = false;
            lora_tilt_stabilizer_requested = false;
            lora_tilt_stabilizer_restart_locked = false;
            tilt_stabilizer.disarm();
            lora_remote_servo1_command = 0.0f;
            lora_remote_servo2_command = 0.0f;
            vehicle.lora_remote_accepted_count++;
            ring_buf.pushEvent("LORA_REMOTE_DISABLED");
            flash_log.logEvent("LORA_REMOTE_DISABLED");
            break;
        default:
            vehicle.lora_remote_rejected_count++;
            ring_buf.pushEvent("LORA_REMOTE_UNKNOWN");
            break;
    }

    vehicle.lora_remote_manual_active = lora_remote_manual_active;
    vehicle.lora_remote_servo1_command = lora_remote_servo1_command;
    vehicle.lora_remote_servo2_command = lora_remote_servo2_command;
}

// ============================================================================
// Initialization Functions
// ============================================================================
void setupSerial() {
    Serial.begin(115200);
    // Battery-only boot has no USB host, so never wait for Serial here.
    // The Wi-Fi dashboard must come up just because the ESP32 has power.
    delay(50);
    Serial.println();
}

void printBanner() {
    Serial.println("============================================");
    Serial.println("  PHOENIX RECOVERY - Parafoil Avionics");
    Serial.println("  Autonomous GPS-Guided Landing System");
    Serial.println("============================================");
    Serial.printf("  Build: %s %s\n", __DATE__, __TIME__);
    Serial.printf("  Target: %s\n", cfg::TARGET_NAME);
    Serial.printf("  LoRa: %.1f MHz, SF%d\n", cfg::LORA_FREQ_MHZ, cfg::LORA_SF);
    Serial.println("============================================");
    Serial.println();
}

bool initializeSensors() {
    Serial.println("[INIT] Initializing sensors...");

    sensors::SensorConfig sensor_cfg;
    sensor_cfg.imu_enabled = true;
    sensor_cfg.baro_enabled = true;
    sensor_cfg.gps_enabled = true;

    if (!sensor_mgr.begin(sensor_cfg)) {
        Serial.println("[INIT] FAILED: Sensor manager");
        return false;
    }

    Serial.print("[INIT] Calibrating BMP388 ground pressure... ");
    if (sensor_mgr.calibrateGroundPressure()) Serial.println("OK");
    else Serial.println("SKIPPED (barometer unavailable)");

    Serial.println("[INIT] Sensors OK");
    return true;
}

bool initializeSubsystems() {
    bool all_ok = true;

    // Servos
    Serial.print("[INIT] Servos... ");
    control::ServoConfig servo_cfg;
    servo_cfg.left_channel = cfg::SERVO_LEFT_CHANNEL;
    servo_cfg.right_channel = cfg::SERVO_RIGHT_CHANNEL;
    if (servos.begin(servo_cfg)) {
        Serial.println("OK");
    } else {
        Serial.println("FAILED");
        all_ok = false;
    }

    // LoRa
    Serial.print("[INIT] LoRa... ");
    if (lora.begin()) {
        Serial.println("OK");
    } else {
        Serial.println("FAILED (non-critical)");
    }

    return all_ok;
}

// ============================================================================
// Control Functions
// ============================================================================
void sendTelemetry() {
    telemetry::TelemetryPacketV1 packet;
    telemetry::encodeTelemetry(vehicle, packet, vehicle.telemetry_sequence);

    // LoRa is telemetry only, including during failsafe.
    if (lora.isHealthy()) {
        if (lora.send(packet)) {
            vehicle.telemetry_ack_count++;
        }
    }

    vehicle.telemetry_sequence++;
}

uint32_t telemetryIntervalMs() {
    uint32_t rate_hz = cfg::TELEMETRY_RATE_HZ;
    if (vehicle.drop_test_recording) {
        rate_hz = cfg::DROP_TEST_TELEMETRY_RATE_HZ;
    } else if (vehicle.drop_test_id != 0 &&
               (vehicle.drop_test_state == phoenix::DropTestState::IDLE ||
                vehicle.drop_test_state == phoenix::DropTestState::TEST_COMPLETE ||
                vehicle.drop_test_state == phoenix::DropTestState::TEST_ABORTED)) {
        rate_hz = cfg::DROP_TEST_IDLE_TELEMETRY_RATE_HZ;
    }
    if (rate_hz == 0) rate_hz = 1;
    return 1000UL / rate_hz;
}

void updateHealth() {
    vehicle.free_heap_bytes = ESP.getFreeHeap();
    if (vehicle.min_free_heap_bytes == 0 || vehicle.free_heap_bytes < vehicle.min_free_heap_bytes) {
        vehicle.min_free_heap_bytes = vehicle.free_heap_bytes;
    }
    // Check if we should start/stop flight logging
    if (vehicle.flight_state == logic::FlightState::PRE_LAUNCH &&
        vehicle.armed) {
        // Starting flight
        if (!flash_log.isLogging()) {
            flash_log.startFlightLog();
            ring_buf.pushEvent("FLIGHT_START");
        }
    }

    // Log health metrics
    char msg[128];
    snprintf(msg, sizeof(msg), "HEALTH: fail=%s Heap=%d",
             logic::failCodeName(vehicle.failure_code), ESP.getFreeHeap());
    ring_buf.pushEvent(msg);
}

void logFlightData() {
    if (!flash_log.isLogging()) return;

    // Log telemetry snapshot
    flash_log.logTelemetry(vehicle);

    // Log state changes
    static logic::FlightState last_state = logic::FlightState::UNKNOWN;
    if (vehicle.flight_state != last_state) {
        flash_log.logFlightState(vehicle);
        last_state = vehicle.flight_state;
    }
}

void printStatus() {
    const auto& imu = sensor_mgr.getIMU()->getData();
    const auto& baro = sensor_mgr.getBarometer()->getData();
    const auto& gps = sensor_mgr.getGPS()->getData();
    const uint32_t now = millis();
    const uint32_t nmea_age = gps.last_nmea_ms > 0 ? now - gps.last_nmea_ms : UINT32_MAX;

    Serial.println("\n--- STATUS ---");
    Serial.printf("State: %d, Mode: %d\n", (uint8_t)vehicle.flight_state, (uint8_t)vehicle.guidance_mode);
    Serial.printf("IMU: %s age=%lums roll=%.1f pitch=%.1f yaw=%.1f\n",
                  imu.valid ? "VALID" : "INVALID", (unsigned long)vehicle.imu_age_ms,
                  imu.roll_deg, imu.pitch_deg, imu.yaw_deg);
    Serial.printf("BARO: %s age=%lums pressure=%.2fhPa temp=%.1fC alt=%.1fm AGL=%.1fm VS=%.1fm/s\n",
                  baro.valid ? "VALID" : "INVALID", (unsigned long)vehicle.baro_age_ms,
                  baro.pressure_hpa, baro.temperature_c, baro.altitude_m,
                  vehicle.altitude_agl_m, vehicle.vertical_speed_mps);
    Serial.printf("BARO DIAG: hw_ok=%lu hw_fail=%lu streak=%u warmup=%lu quality_reject=%lu qstreak=%u recovery=%lu/%lu raw=%.2fhPa/%.1fm last_ok=%lums last_fail=%lums last_reject=%lums\n",
                  (unsigned long)baro.hardware_read_successes,
                  (unsigned long)baro.hardware_read_failures,
                  (unsigned int)baro.consecutive_hardware_failures,
                  (unsigned long)baro.warmup_samples_discarded,
                  (unsigned long)baro.quality_rejections,
                  (unsigned int)baro.consecutive_quality_rejections,
                  (unsigned long)baro.successful_recoveries,
                  (unsigned long)baro.recovery_attempts,
                  baro.raw_pressure_hpa, baro.raw_altitude_m,
                  (unsigned long)baro.last_hardware_success_ms,
                  (unsigned long)baro.last_hardware_failure_ms,
                  (unsigned long)baro.last_quality_rejection_ms);
    Serial.printf("GPS: %s used=%d visible=%d nmea=%s age=%lums uart_bytes=%lu sentences=%lu checksum_fail=%lu recoveries=%lu (%.6f, %.6f)\n",
                  vehicle.gps_valid ? "FIX" : "NO FIX", vehicle.satellite_count,
                  vehicle.satellites_in_view,
                  gps.last_nmea_ms > 0 ? "ACTIVE" : "NONE", (unsigned long)nmea_age,
                  (unsigned long)gps.uart_bytes_received,
                  (unsigned long)gps.valid_nmea_sentences,
                  (unsigned long)gps.failed_nmea_checksums,
                  (unsigned long)gps.recovery_attempts,
                  vehicle.latitude, vehicle.longitude);
    if (cfg::PAYLOAD_WIFI_ENABLED) {
        Serial.printf("WIFI: ACTIVE=%s mode=%d SSID=%s IP=%s clients=%d starts=%lu/%lu last_start=%lums\n",
                      wifi.isRunning() ? "YES" : "NO",
                      static_cast<int>(WiFi.getMode()), cfg::WIFI_AP_SSID,
                      wifi.getAPIP().toString().c_str(), wifi.getClientCount(),
                      (unsigned long)wifi.getSuccessfulStarts(),
                      (unsigned long)wifi.getRestartAttempts(),
                      (unsigned long)wifi.getLastStartMs());
    } else {
        Serial.println("WIFI: DISABLED ON PAYLOAD (use PHOENIX-GROUND)");
    }
    Serial.printf("Target: %.6f, %.6f\n", vehicle.target_latitude, vehicle.target_longitude);
    Serial.printf("Distance: %.1fm, Heading: %.1f deg\n",
                  vehicle.distance_to_target_m, vehicle.heading_error_deg);
    Serial.printf("SERVOS: %s L=%.2f/%.0fus R=%.2f/%.0fus\n",
                  servos.isHealthy() ? "READY" : "FAILED",
                  vehicle.left_servo_command, vehicle.left_servo_us,
                  vehicle.right_servo_command, vehicle.right_servo_us);
    Serial.printf("SERVO ANGLES: Servo1 turn=%.1fdeg angle=%.1fdeg Servo2 turn=%.1fdeg angle=%.1fdeg\n",
                  vehicle.left_servo_turn_deg, vehicle.left_servo_angle_deg,
                  vehicle.right_servo_turn_deg, vehicle.right_servo_angle_deg);
    Serial.printf("LORA: %s packets=%lu\n", lora.isHealthy() ? "READY" : "FAILED",
                  (unsigned long)vehicle.telemetry_ack_count);
    Serial.printf("LORA COMMANDS: seq=%u accepted=%lu rejected=%lu bench=%s armed=%s link=%s\n",
                  vehicle.lora_remote_sequence,
                  (unsigned long)vehicle.lora_remote_accepted_count,
                  (unsigned long)vehicle.lora_remote_rejected_count,
                  lora_bench_servo_active ? "ACTIVE" : "OFF",
                  vehicle.armed ? "YES" : "NO",
                  vehicle.lora_remote_link_active ? "LIVE" : "STALE");
    Serial.printf("Loop: %.1f Hz, Heap: %d bytes\n", vehicle.loop_rate_hz, ESP.getFreeHeap());
    Serial.printf("Failsafe: %s (%s)\n",
                  vehicle.failure_code == logic::FailCode::FAIL_NONE ? "INACTIVE" : "ACTIVE",
                  logic::failCodeName(vehicle.failure_code));
    Serial.printf("Launch readiness: %s%s\n",
                  vehicle.launch_readiness_ok ? "OK" : "NOT READY",
                  vehicle.preflight_launch_warning ? " — LAUNCH ATTEMPT BLOCKED; FAILSAFE ACTIVE" : "");
    Serial.println("---------------\n");
}
