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
#include "logic/servo_retract_test.h"
#include "logic/guided_drop_controller.h"
#include "logic/guided_drop_arm_gate.h"
#include "comms/lora.h"
#include "comms/wifi_manager.h"
#include "telemetry/telemetry_packet.h"
#include "safety/health_monitor.h"
#include "safety/watchdog.h"
#include "logging/flash_logger.h"
#include "logging/ring_buffer.h"

// Wire format v8 adds synchronized configuration, age, power-validity, and
// guided-profile evidence. Keep this exact assertion tied to the airtime test.
static_assert(sizeof(telemetry::TelemetryPacketV1) == 140,
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
static logic::ServoRetractTest servo_retract_test;
static logic::GuidedDropController guided_drop_controller;
static logic::GuidedDropArmGate guided_drop_arm_gate(cfg::GUIDED_DROP_CONFIRM_WINDOW_MS);
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
static uint32_t last_post_landing_wifi_attempt_ms = 0;
static bool boot_reset_fail_neutral = false;

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
bool armDropTest(uint32_t now, bool guided_return = false);
void abortDropTest(uint32_t now, const char* reason);
uint32_t telemetryIntervalMs();
bool saveRemoteTarget(double latitude, double longitude);
void loadPersistedTarget();
bool remoteControlAllowedForState(logic::FlightState state);
bool benchServoActuationAllowed(uint32_t now);
bool guidedPhysicalArmActive();
logic::GuidedDropUnlocks guidedDropUnlocks();
logic::GuidedArmPrerequisites guidedArmPrerequisites();
float guidedAuthorityLimit(logic::GuidedAuthorityLevel level);

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
static bool guided_drop_requested = false;
static bool guided_abort_latched = false;
static logic::GuidedAuthorityLevel guided_authority =
    logic::GuidedAuthorityLevel::NEUTRAL;
static uint16_t next_drop_test_id = 1;

static logic::ServoRetractTestConfig servoRetractTestConfig() {
    logic::ServoRetractTestConfig c;
    c.extension_command = cfg::SERVO_RETRACT_TEST_COMMAND;
    c.extension_hold_ms = cfg::SERVO_RETRACT_TEST_HOLD_MS;
    c.retraction_settle_ms = cfg::SERVO_RETRACT_TEST_SETTLE_MS;
    return c;
}

static logic::GuidedDropConfig guidedDropConfig() {
    logic::GuidedDropConfig c;
    c.enabled = cfg::GUIDED_DROP_TEST_ENABLED &&
                cfg::GUIDED_DROP_CALIBRATION_CONFIRMED;
    c.deployment_neutral_ms = cfg::GUIDED_DROP_DEPLOYMENT_NEUTRAL_MS;
    c.navigation_confirm_ms = cfg::GUIDED_DROP_NAV_CONFIRM_MS;
    c.navigation_loss_latch_ms = cfg::GUIDED_DROP_NAV_LOSS_LATCH_MS;
    c.sensor_max_age_ms = cfg::SENSOR_TIMEOUT_MS;
    c.min_activation_altitude_m = cfg::GUIDED_DROP_MIN_ACTIVATION_ALTITUDE_M;
    c.max_activation_angular_rate_dps = cfg::GUIDED_DROP_MAX_ACTIVATION_GYRO_DPS;
    c.reduced_authority_angular_rate_dps = cfg::GUIDED_DROP_MAX_ACTIVATION_GYRO_DPS;
    c.neutral_angular_rate_dps = cfg::GUIDED_DROP_NEUTRAL_GYRO_DPS;
    c.instability_latch_ms = cfg::GUIDED_DROP_INSTABILITY_LATCH_MS;
    c.final_approach_radius_m = cfg::FINAL_APPROACH_RADIUS_M;
    c.gps_max_position_jump_m = cfg::GPS_MAX_JUMP_M;
    c.gps_recovery_fix_count = cfg::GPS_RECOVERY_FIX_COUNT;
    c.imu_course_hold_enabled = cfg::GUIDED_DROP_IMU_COURSE_HOLD_ENABLED;
    c.imu_course_hold_max_ms = cfg::GUIDED_DROP_IMU_COURSE_HOLD_MAX_MS;
    c.imu_course_hold_max_command = cfg::GUIDED_DROP_IMU_COURSE_HOLD_MAX_COMMAND;
    c.oscillation_sign_change_limit = cfg::GUIDED_DROP_OSCILLATION_SIGN_CHANGES;
    c.oscillation_window_ms = cfg::GUIDED_DROP_OSCILLATION_WINDOW_MS;
    c.guidance.kp = cfg::GUIDANCE_KP;
    c.guidance.deadband_deg = cfg::GUIDANCE_DEADBAND_DEG;
    c.guidance.max_steering = cfg::GUIDED_DROP_MAX_STEERING;
    c.guidance.low_speed_max_command = 0.05f;
    c.guidance.unreachable_max_command = cfg::GUIDED_DROP_MAX_STEERING;
    c.guidance.cmd_rate_limit_per_s = cfg::CMD_RATE_LIMIT_PER_S;
    c.guidance.reversal_guard_ms = cfg::MIN_TURN_REVERSAL_TIME_MS;
    c.guidance.final_approach_kp = cfg::FINAL_APPROACH_KP;
    c.guidance.final_approach_max_command = cfg::GUIDED_DROP_FINAL_MAX_STEERING;
    c.guidance.final_approach_altitude_m = cfg::FINAL_APPROACH_ALTITUDE_M;
    c.guidance.flare_enabled = false;
    c.guidance.min_guidance_altitude_m = cfg::MIN_GUIDANCE_ALTITUDE_M;
    c.guidance.target_accept_radius_m = cfg::TARGET_ACCEPT_RADIUS_M;
    c.guidance.min_course_speed_mps = cfg::GPS_MIN_GROUND_SPEED_FOR_COURSE;
    return c;
}

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
    c.release_imu_accel_mps2 = cfg::DROP_TEST_RELEASE_IMU_ACCEL_MPS2;
    c.release_candidate_timeout_ms = cfg::DROP_TEST_RELEASE_CANDIDATE_TIMEOUT_MS;
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
    if (cfg::PIN_GUIDED_DROP_ARM_SWITCH >= 0) {
        pinMode(cfg::PIN_GUIDED_DROP_ARM_SWITCH,
                cfg::GUIDED_DROP_ARM_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
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
    servo_retract_test = logic::ServoRetractTest(servoRetractTestConfig());
    guided_drop_controller = logic::GuidedDropController(guidedDropConfig());
    vehicle.configuration_version = cfg::CONFIG_VERSION;

    const esp_reset_reason_t reset_reason =
        static_cast<esp_reset_reason_t>(vehicle.reset_reason);
    boot_reset_fail_neutral = reset_reason == ESP_RST_PANIC ||
        reset_reason == ESP_RST_INT_WDT || reset_reason == ESP_RST_TASK_WDT ||
        reset_reason == ESP_RST_WDT || reset_reason == ESP_RST_BROWNOUT ||
        reset_reason == ESP_RST_UNKNOWN;
    if (boot_reset_fail_neutral) {
        coordinator.forceFailsafe(logic::FailCode::FAIL_REBOOT_UNCERTAIN,
                                  millis());
        guided_abort_latched = true;
    }

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

    // Establish calibrated neutral before sensor calibration or radio startup.
    // This minimizes the interval in which a rebooted servo signal could float.
    control::ServoConfig boot_servo_cfg;
    boot_servo_cfg.left_channel = cfg::SERVO_LEFT_CHANNEL;
    boot_servo_cfg.right_channel = cfg::SERVO_RIGHT_CHANNEL;
    if (servos.begin(boot_servo_cfg)) {
        servos.emergencyNeutral();
        Serial.println("[BOOT] Servos attached at calibrated neutral");
    } else {
        Serial.println("[BOOT] ERROR: servo neutral initialization failed");
        vehicle.failure_code = logic::FailCode::FAIL_SERVO;
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
    if (flash_log.isLogging()) {
        flash_log.logEvent("PHOENIX_RECOVERY_BOOT");
        if (boot_reset_fail_neutral) {
            flash_log.logEvent("REBOOT_UNCERTAIN_FAIL_NEUTRAL");
        }
    }

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

    // ---- Automatic unloaded extension/return-to-neutral service test ----
    // This deliberately shares the existing sensor-independent bench gate and
    // the calibrated bench-servo output path. It cannot become active after
    // launch, while armed, or while a drop recording is open.
    logic::ServoRetractTestInput retract_in;
    retract_in.now_ms = now;
    retract_in.preflight_allowed = benchServoActuationAllowed(now);
    retract_in.servo_healthy = servos.isHealthy();
    const logic::ServoRetractTestOutput retract_output =
        servo_retract_test.step(retract_in);
    vehicle.servo_retract_test_status = retract_output.status;
    vehicle.servo_retract_test_active = retract_output.active;
    static logic::ServoRetractTestStatus previous_retract_status =
        logic::ServoRetractTestStatus::IDLE;
    if (retract_output.status != previous_retract_status) {
        const char* event = logic::servoRetractTestStatusName(retract_output.status);
        ring_buf.pushEvent(event);
        flash_log.logEvent(event);
        previous_retract_status = retract_output.status;
    }

    // ---- Calibration-locked return-to-target inert-drop controller ----
    const logic::GuidedArmResult guided_arm_snapshot =
        guided_drop_arm_gate.snapshot(now);
    vehicle.guided_arm_status = guided_arm_snapshot.status;
    vehicle.guided_canopy_detected =
        vehicle.drop_test_canopy_signature_ms != 0;
    vehicle.guided_confirmation_pending = guided_arm_snapshot.confirmation_pending;
    vehicle.guided_physical_arm_active = guidedPhysicalArmActive();
    vehicle.guided_authority = guided_authority;
    vehicle.guided_authority_limit = guidedAuthorityLimit(guided_authority);
    vehicle.guided_abort_latched = guided_abort_latched;
    vehicle.guided_profile_unlock_flags =
        (cfg::GUIDED_DROP_TEST_ENABLED ? 1U : 0U) |
        (cfg::GUIDED_DROP_CALIBRATION_CONFIRMED ? 2U : 0U) |
        (cfg::GUIDED_DROP_20_PERCENT_UNLOCKED ? 4U : 0U) |
        (cfg::GUIDED_DROP_40_PERCENT_UNLOCKED ? 8U : 0U) |
        (cfg::GUIDED_DROP_VALIDATED_FULL_UNLOCKED ? 16U : 0U);
    logic::GuidedDropInput guided_in;
    guided_in.now_ms = now;
    guided_in.dt_s = dt_s;
    guided_in.guided_test_requested = guided_drop_requested;
    guided_in.drop_recording = vehicle.drop_test_recording;
    guided_in.drop_state = vehicle.drop_test_state;
    guided_in.release_confirm_ms = vehicle.drop_test_release_confirm_ms;
    guided_in.stable_descent_observed = vehicle.drop_test_stable_descent_ms != 0;
    guided_in.canopy_event_observed = vehicle.guided_canopy_detected;
    guided_in.abort_latched = guided_abort_latched;
    guided_in.authority = guided_authority;
    guided_in.authority_limit = vehicle.guided_authority_limit;
    guided_in.latitude = vehicle.latitude;
    guided_in.longitude = vehicle.longitude;
    guided_in.target_latitude = vehicle.target_latitude;
    guided_in.target_longitude = vehicle.target_longitude;
    guided_in.altitude_agl_m = vehicle.altitude_agl_m;
    guided_in.ground_speed_mps = vehicle.ground_speed_mps;
    guided_in.gps_course_deg = vehicle.gps_course_deg;
    guided_in.imu_heading_deg = vehicle.yaw_deg;
    guided_in.angular_rate_dps = vehicle.angular_rate_dps;
    guided_in.gps_valid = vehicle.gps_valid;
    guided_in.imu_valid = vehicle.imu_valid;
    guided_in.barometer_valid = vehicle.barometer_valid;
    guided_in.servo_valid = servos.isHealthy();
    guided_in.gps_age_ms = vehicle.gps_age_ms;
    guided_in.imu_age_ms = vehicle.imu_age_ms;
    guided_in.barometer_age_ms = vehicle.baro_age_ms;
    const logic::GuidedDropOutput guided_output =
        guided_drop_controller.step(guided_in);
    vehicle.guided_drop_status = guided_output.status;
    vehicle.guided_drop_requested = guided_drop_requested;
    vehicle.guided_drop_active = guided_output.active;
    vehicle.guided_gps_suspect = guided_output.gps_suspect;
    vehicle.guided_course_hold_active = guided_output.course_hold_active;
    vehicle.guided_oscillation_damping = guided_output.oscillation_damping_active;
    static logic::GuidedDropStatus previous_guided_status =
        logic::GuidedDropStatus::FEATURE_DISABLED;
    if (guided_output.status != previous_guided_status) {
        const char* event = logic::guidedDropStatusName(guided_output.status);
        ring_buf.pushEvent(event);
        flash_log.logEvent(event);
        previous_guided_status = guided_output.status;
    }
    if (guided_drop_requested) {
        vehicle.drop_test_neutral_lock = guided_output.neutral_required;
        vehicle.requested_left_servo_command = guided_output.left_brake;
        vehicle.requested_right_servo_command = guided_output.right_brake;
        vehicle.target_bearing_deg = guided_output.target_bearing_deg;
        vehicle.heading_error_deg = guided_output.heading_error_deg;
        vehicle.distance_to_target_m = guided_output.distance_to_target_m;
    }

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
    if (retract_output.interlocked) {
        servos.emergencyNeutral();
    }

    // An explicit, time-limited, prelaunch bench command may operate even when
    // sensors placed the flight coordinator in failsafe. It is gated by launch,
    // arming, recording, servo health, and boot-time window—not sensor health.
    if (lora_bench_servo_active && bench_allowed_now) {
        servos.setBenchServoCommands(lora_remote_servo1_command,
                                     lora_remote_servo2_command);
        servos.update();
    } else if (retract_output.active && bench_allowed_now) {
        servos.setBenchServoCommands(retract_output.servo1_command,
                                     retract_output.servo2_command);
        servos.update();
    } else if (command.failsafe_active || command.state == logic::FlightState::LANDED ||
        vehicle.drop_test_neutral_lock) {
        servos.emergencyNeutral();
    } else if (guided_drop_requested && guided_output.active) {
        servos.setGuidedDropBrakeCommands(guided_output.left_brake,
                                          guided_output.right_brake,
                                          vehicle.guided_authority_limit);
        servos.update();
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

    // ---- Optional payload WiFi dashboard / post-landing log service ----
    const bool recovery_wifi_allowed =
        cfg::PAYLOAD_POST_LANDING_WIFI_ENABLED &&
        !vehicle.drop_test_recording &&
        (vehicle.flight_state == logic::FlightState::LANDED ||
         vehicle.drop_test_state == phoenix::DropTestState::TEST_COMPLETE);
    if (!cfg::PAYLOAD_WIFI_ENABLED && recovery_wifi_allowed &&
        !wifi.isRunning() &&
        now - last_post_landing_wifi_attempt_ms >= cfg::WIFI_RECOVERY_CHECK_MS) {
        last_post_landing_wifi_attempt_ms = now;
        if (wifi.begin(&vehicle, &servos)) {
            ring_buf.pushEvent("POST_LANDING_WIFI_STARTED");
            Serial.printf("[RECOVERY] Payload log service: http://%s/recovery\n",
                          wifi.getAPIP().toString().c_str());
        }
    }
    if (cfg::PAYLOAD_WIFI_ENABLED || wifi.isRunning()) wifi.update();

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

bool guidedPhysicalArmActive() {
    if (cfg::PIN_GUIDED_DROP_ARM_SWITCH < 0) return false;
    const int level = digitalRead(cfg::PIN_GUIDED_DROP_ARM_SWITCH);
    return cfg::GUIDED_DROP_ARM_ACTIVE_LOW ? level == LOW : level == HIGH;
}

logic::GuidedDropUnlocks guidedDropUnlocks() {
    logic::GuidedDropUnlocks unlocks;
    unlocks.feature_enabled = cfg::GUIDED_DROP_TEST_ENABLED;
    unlocks.calibration_confirmed = cfg::GUIDED_DROP_CALIBRATION_CONFIRMED;
    unlocks.twenty_percent_unlocked = cfg::GUIDED_DROP_20_PERCENT_UNLOCKED;
    unlocks.forty_percent_unlocked = cfg::GUIDED_DROP_40_PERCENT_UNLOCKED;
    unlocks.validated_full_unlocked = cfg::GUIDED_DROP_VALIDATED_FULL_UNLOCKED;
    return unlocks;
}

logic::GuidedArmPrerequisites guidedArmPrerequisites() {
    logic::GuidedArmPrerequisites prerequisites;
    prerequisites.physical_arm_active = guidedPhysicalArmActive();
    prerequisites.target_valid = isfinite(vehicle.target_latitude) &&
        isfinite(vehicle.target_longitude) &&
        vehicle.target_latitude >= -90.0 && vehicle.target_latitude <= 90.0 &&
        vehicle.target_longitude >= -180.0 && vehicle.target_longitude <= 180.0 &&
        !(fabs(vehicle.target_latitude) < 1e-9 &&
          fabs(vehicle.target_longitude) < 1e-9);
    prerequisites.gps_valid = vehicle.gps_valid &&
        vehicle.gps_age_ms <= cfg::SENSOR_TIMEOUT_MS;
    prerequisites.imu_valid = vehicle.imu_valid &&
        vehicle.imu_age_ms <= cfg::SENSOR_TIMEOUT_MS;
    prerequisites.barometer_valid = vehicle.barometer_valid &&
        vehicle.baro_age_ms <= cfg::SENSOR_TIMEOUT_MS;
    prerequisites.servo_valid = servos.isHealthy();
    prerequisites.log_ready = flash_log.isLogging();
    prerequisites.recorder_idle = !vehicle.drop_test_recording;
    return prerequisites;
}

float guidedAuthorityLimit(logic::GuidedAuthorityLevel level) {
    switch (level) {
        case logic::GuidedAuthorityLevel::TWENTY_PERCENT: return 0.20f;
        case logic::GuidedAuthorityLevel::FORTY_PERCENT: return 0.40f;
        case logic::GuidedAuthorityLevel::VALIDATED_FULL:
            return cfg::GUIDED_DROP_MAX_STEERING;
        default: return 0.0f;
    }
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
    if (!out.recording && guided_drop_requested) {
        guided_drop_requested = false;
        guided_drop_controller.reset();
        guided_drop_arm_gate.reset();
        guided_authority = logic::GuidedAuthorityLevel::NEUTRAL;
        vehicle.guided_drop_requested = false;
        vehicle.guided_drop_active = false;
        vehicle.guided_canopy_detected = false;
        vehicle.guided_confirmation_pending = false;
        vehicle.guided_authority = logic::GuidedAuthorityLevel::NEUTRAL;
        vehicle.guided_authority_limit = 0.0f;
    }

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

bool armDropTest(uint32_t now, bool guided_return) {
    if (!cfg::DROP_TEST_MODE_ENABLED) return false;
    const bool target_valid = isfinite(vehicle.target_latitude) &&
        isfinite(vehicle.target_longitude) &&
        vehicle.target_latitude >= -90.0 && vehicle.target_latitude <= 90.0 &&
        vehicle.target_longitude >= -180.0 && vehicle.target_longitude <= 180.0 &&
        !(fabs(vehicle.target_latitude) < 1e-9 &&
          fabs(vehicle.target_longitude) < 1e-9);
    if (guided_return &&
        (!cfg::GUIDED_DROP_TEST_ENABLED ||
         !cfg::GUIDED_DROP_CALIBRATION_CONFIRMED ||
         !target_valid || !vehicle.gps_valid ||
         !guidedPhysicalArmActive() ||
         guided_authority == logic::GuidedAuthorityLevel::NEUTRAL)) {
        vehicle.drop_test_rejected_count++;
        ring_buf.pushEvent("GUIDED_DROP_ARM_REJECTED");
        flash_log.logEvent("GUIDED_DROP_ARM_REJECTED");
        return false;
    }
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
    guided_drop_controller.reset();
    guided_drop_requested = guided_return;
    guided_abort_latched = false;
    vehicle.guided_drop_requested = guided_return;
    // The new controller is intentionally stricter: an inert recording run is
    // always neutral-locked, regardless of radio or dashboard state.
    vehicle.drop_test_neutral_lock = true;
    servos.emergencyNeutral();
    const char* armed_event = guided_return
        ? "GUIDED_DROP_TEST_ARMED" : "DROP_TEST_ARMED";
    ring_buf.pushEvent(armed_event);
    flash_log.logEvent(armed_event);
    if (guided_return) {
        // No visual pre-open confirmation is used. The controller remains
        // neutral until the payload records a canopy load signature and a
        // separate stable-descent window after release.
        flash_log.logEvent("GUIDED_CANOPY_AUTODETECTION_REQUIRED");
    }
    phoenix::logging::ConfigurationSnapshotV1 snapshot{};
    snapshot.schema_version = 1;
    snapshot.guided_authority = static_cast<uint8_t>(guided_authority);
    snapshot.flare_enabled = 0;
    snapshot.servo1_controls_left_brake =
        servos.servo1ControlsLeftBrake() ? 1U : 0U;
    snapshot.configuration_version = cfg::CONFIG_VERSION;
    snapshot.target_latitude = vehicle.target_latitude;
    snapshot.target_longitude = vehicle.target_longitude;
    const control::ServoController::Calibration& servo1 =
        servos.getLeftCalibration();
    const control::ServoController::Calibration& servo2 =
        servos.getRightCalibration();
    snapshot.servo1_neutral_us = servo1.neutral_us;
    snapshot.servo1_min_us = servo1.min_us;
    snapshot.servo1_max_us = servo1.max_us;
    snapshot.servo1_max_brake_us = servo1.max_brake_us;
    snapshot.servo2_neutral_us = servo2.neutral_us;
    snapshot.servo2_min_us = servo2.min_us;
    snapshot.servo2_max_us = servo2.max_us;
    snapshot.servo2_max_brake_us = servo2.max_brake_us;
    flash_log.logConfigurationSnapshot(snapshot);
    flash_log.logTelemetry(vehicle);
    return true;
}

void abortDropTest(uint32_t now, const char* reason) {
    lora_remote_manual_active = false;
    lora_bench_servo_active = false;
    lora_remote_servo1_command = 0.0f;
    lora_remote_servo2_command = 0.0f;
    guided_abort_latched = true;
    vehicle.guided_abort_latched = true;
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
    guided_drop_requested = false;
    guided_drop_controller.reset();
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
            servo_retract_test.stop();
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
            servo_retract_test.stop();
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
        case comms::LoRaRemoteCommandType::ARM_DROP_TEST: {
            servo_retract_test.stop();
            lora_tilt_stabilizer_requested = false;
            tilt_stabilizer.disarm();
            const bool accepted = armDropTest(now, false);
            if (accepted) {
                vehicle.lora_remote_accepted_count++;
            } else {
                vehicle.lora_remote_rejected_count++;
            }
            break;
        }
        case comms::LoRaRemoteCommandType::CONFIRM_CANOPY_READY: {
            servo_retract_test.stop();
            lora_tilt_stabilizer_requested = false;
            tilt_stabilizer.disarm();
            // Retain the packet number for compatibility with older ground
            // firmware, but visual confirmation no longer has control authority.
            vehicle.lora_remote_rejected_count++;
            ring_buf.pushEvent("CANOPY_CONFIRM_COMMAND_DEPRECATED");
            flash_log.logEvent("CANOPY_CONFIRM_COMMAND_DEPRECATED");
            break;
        }
        case comms::LoRaRemoteCommandType::PREPARE_GUIDED_DROP_TEST: {
            servo_retract_test.stop();
            lora_tilt_stabilizer_requested = false;
            tilt_stabilizer.disarm();
            logic::GuidedAuthorityLevel requested =
                logic::GuidedAuthorityLevel::NEUTRAL;
            if (remote.servo1_command >= 0.75f) {
                requested = logic::GuidedAuthorityLevel::VALIDATED_FULL;
            } else if (remote.servo1_command >= 0.30f) {
                requested = logic::GuidedAuthorityLevel::FORTY_PERCENT;
            } else if (remote.servo1_command >= 0.15f) {
                requested = logic::GuidedAuthorityLevel::TWENTY_PERCENT;
            }
            const logic::GuidedArmResult result = guided_drop_arm_gate.prepare(
                now, requested, guidedDropUnlocks(), guidedArmPrerequisites());
            vehicle.guided_arm_status = result.status;
            vehicle.guided_confirmation_pending = result.confirmation_pending;
            if (result.confirmation_pending) {
                guided_authority = result.authority;
                vehicle.guided_authority = result.authority;
                vehicle.lora_remote_accepted_count++;
                ring_buf.pushEvent("GUIDED_DROP_PREPARED");
                flash_log.logEvent("GUIDED_DROP_PREPARED");
            } else {
                vehicle.lora_remote_rejected_count++;
                ring_buf.pushEvent(logic::guidedArmStatusName(result.status));
                flash_log.logEvent(logic::guidedArmStatusName(result.status));
            }
            break;
        }
        case comms::LoRaRemoteCommandType::ARM_GUIDED_DROP_TEST: {
            servo_retract_test.stop();
            lora_tilt_stabilizer_requested = false;
            tilt_stabilizer.disarm();
            // The current dashboard uses this as a single-step arm request.
            // The payload still owns every gate and selects only the lowest
            // calibrated 20% authority stage.
            const logic::GuidedArmResult result = guided_drop_arm_gate.armNow(
                now,
                logic::GuidedAuthorityLevel::TWENTY_PERCENT,
                guidedDropUnlocks(),
                guidedArmPrerequisites());
            vehicle.guided_arm_status = result.status;
            vehicle.guided_confirmation_pending = false;
            if (result.arm_authorized) {
                guided_authority = result.authority;
                vehicle.guided_authority = result.authority;
            }
            const bool armed = result.arm_authorized && armDropTest(now, true);
            if (armed) {
                vehicle.lora_remote_accepted_count++;
            } else {
                // Never leave an ARMED guided gate or non-neutral authority
                // behind if the recorder's independent final gate rejects.
                if (result.arm_authorized) {
                    guided_drop_arm_gate.reset();
                    guided_authority = logic::GuidedAuthorityLevel::NEUTRAL;
                    vehicle.guided_arm_status = logic::GuidedArmStatus::IDLE;
                    vehicle.guided_authority = logic::GuidedAuthorityLevel::NEUTRAL;
                    vehicle.guided_authority_limit = 0.0f;
                }
                vehicle.lora_remote_rejected_count++;
                ring_buf.pushEvent(logic::guidedArmStatusName(result.status));
                flash_log.logEvent(logic::guidedArmStatusName(result.status));
            }
            break;
        }
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
            servo_retract_test.stop();
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
            servo_retract_test.stop();
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
        case comms::LoRaRemoteCommandType::START_SERVO_RETRACT_TEST: {
            const bool safe = cfg::SERVO_RETRACT_TEST_ENABLED &&
                benchServoActuationAllowed(now);
            lora_remote_manual_active = false;
            lora_bench_servo_active = false;
            lora_tilt_stabilizer_requested = false;
            lora_tilt_stabilizer_restart_locked = false;
            tilt_stabilizer.disarm();
            const logic::ServoRetractTestOutput started =
                servo_retract_test.start(now, safe, servos.isHealthy());
            if (started.active) {
                vehicle.lora_remote_accepted_count++;
                ring_buf.pushEvent("SERVO_RETRACT_TEST_STARTED");
                flash_log.logEvent("SERVO_RETRACT_TEST_STARTED");
            } else {
                servos.emergencyNeutral();
                vehicle.lora_remote_rejected_count++;
                ring_buf.pushEvent("SERVO_RETRACT_TEST_REJECTED");
            }
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
            servo_retract_test.stop();
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
    if (servos.isHealthy()) {
        Serial.println("OK (neutral already established)");
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
