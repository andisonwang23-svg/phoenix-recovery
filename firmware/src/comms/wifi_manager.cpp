// ============================================================================
// PHOENIX RECOVERY — WiFi Manager Implementation.
// ============================================================================
#include "wifi_manager.h"
#include <Preferences.h>

namespace comms {

namespace {
float normalizeDegrees180(float degrees) {
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees < -180.0f) degrees += 360.0f;
    return degrees;
}
} // namespace

WiFiManager::WiFiManager() = default;

WiFiManager::~WiFiManager() {
    if (server_) delete server_;
    if (events_) delete events_;
}

bool WiFiManager::begin(phoenix::VehicleState* state, control::ServoController* servos) {
    state_ = state;
    servos_ = servos;
    Preferences targetPrefs;
    if (targetPrefs.begin("target_cfg", true)) {
        state_->target_latitude = targetPrefs.getDouble("latitude", cfg::TARGET_LATITUDE);
        state_->target_longitude = targetPrefs.getDouble("longitude", cfg::TARGET_LONGITUDE);
        targetPrefs.end();
    }
    if (!startAPWithRetries(cfg::WIFI_BOOT_START_ATTEMPTS, cfg::WIFI_BOOT_RETRY_DELAY_MS)) {
        markAPRunning(false);
        return false;
    }
    setupRoutes();
    setupCaptivePortal();
    markAPRunning(true);
    push_interval_ms_ = 1000 / cfg::DASHBOARD_PUSH_RATE_HZ;
    return true;
}

bool WiFiManager::startAPWithRetries(uint8_t attempts, uint32_t retry_delay_ms) {
    if (attempts == 0) attempts = 1;
    for (uint8_t attempt = 1; attempt <= attempts; ++attempt) {
        if (setupAP()) return true;
        Serial.printf("[WiFi] boot AP attempt %u/%u failed\n", attempt, attempts);
        markAPRunning(false);
        if (attempt < attempts) delay(retry_delay_ms);
    }
    return false;
}

bool WiFiManager::setupAP() {
    restart_attempts_++;
    // Reset only the active AP interface. Erasing stored credentials and
    // immediately changing WIFI_MODE_NULL could leave some ESP32-S3 clients
    // unable to see the AP during repeated recovery attempts.
    WiFi.persistent(false);
    // Avoid cycling through WIFI_OFF. Some ESP32-S3 radio builds can report a
    // valid SoftAP afterward while failing to advertise it in scan results.
    WiFi.mode(WIFI_AP);
    delay(250);
    WiFi.setSleep(false);

    const IPAddress ap_ip(192, 168, 4, 1);
    const IPAddress gateway(192, 168, 4, 1);
    const IPAddress subnet(255, 255, 255, 0);
    if (!WiFi.softAPConfig(ap_ip, gateway, subnet)) {
        Serial.println("[WiFi] ERROR: static AP address configuration failed");
        return false;
    }

    if (!WiFi.softAP(cfg::WIFI_AP_SSID, cfg::WIFI_AP_PASS,
                     cfg::WIFI_AP_CHANNEL, false, cfg::WIFI_AP_MAX_CONN)) {
        Serial.println("[WiFi] ERROR: access point failed to start");
        return false;
    }
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    delay(100);
    const IPAddress actual_ip = WiFi.softAPIP();
    if (WiFi.getMode() != WIFI_AP || actual_ip[0] == 0) {
        Serial.printf("[WiFi] ERROR: AP verification failed mode=%d ip=%s\n",
                      static_cast<int>(WiFi.getMode()), actual_ip.toString().c_str());
        WiFi.softAPdisconnect(true);
        return false;
    }
    successful_starts_++;
    last_start_ms_ = millis();
    Serial.printf("[WiFi] AP active: SSID=%s channel=%u\n",
                  cfg::WIFI_AP_SSID, cfg::WIFI_AP_CHANNEL);
    Serial.printf("[WiFi] Dashboard: http://%s/\n",
                  WiFi.softAPIP().toString().c_str());
    return true;
}

bool WiFiManager::apHealthy() const {
    return WiFi.getMode() == WIFI_AP && WiFi.softAPIP()[0] != 0;
}

void WiFiManager::markAPRunning(bool running) {
    ap_started_ = running;
    if (state_) state_->wifi_ap_active = running;
}

void WiFiManager::recoverAPIfNeeded(uint32_t now_ms) {
    if (now_ms - last_ap_check_ms_ < cfg::WIFI_RECOVERY_CHECK_MS) {
        return;
    }
    last_ap_check_ms_ = now_ms;

    if (apHealthy()) {
        degraded_health_checks_ = 0;
        markAPRunning(true);
        return;
    }

    degraded_health_checks_++;
    markAPRunning(false);

    const int clients = WiFi.softAPgetStationNum();
    if (clients > 0) {
        // If a client is still associated, avoid a recovery cycle that would
        // create the exact disconnect behavior we are trying to eliminate.
        Serial.printf("[WiFi] WARNING: AP health degraded (%lu), clients=%d; holding AP\n",
                      (unsigned long)degraded_health_checks_, clients);
        return;
    }

    Serial.printf("[WiFi] AP inactive/unhealthy; automatic restart attempt %lu\n",
                  (unsigned long)(restart_attempts_ + 1));
    if (setupAP()) {
        markAPRunning(true);
        degraded_health_checks_ = 0;
        if (!server_) setupRoutes();
        setupCaptivePortal();
    }
}

void WiFiManager::setupRoutes() {
    server_ = new AsyncWebServer(cfg::WIFI_HTTP_PORT);
    events_ = new AsyncEventSource("/events");

    // CORS for local access
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

    // Main dashboard
    server_->on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "text/html", generateDashboardHTML());
    });

    // API endpoints
    server_->on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "application/json", getStatusJSON());
    });

    server_->on("/api/sensors", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "application/json", getSensorsJSON());
    });

    server_->on("/api/navigation", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "application/json", getNavigationJSON());
    });

    server_->on("/api/config", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "application/json", getConfigJSON());
    });

    server_->on("/api/readiness", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "application/json", getReadinessJSON());
    });

    server_->on("/api/config", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleConfigPost(request);
    });

    server_->on("/api/reboot", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleReboot(request);
    });

    // Config page
    server_->on("/config", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "text/html", generateConfigHTML());
    });

    server_->on("/setup", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "text/html", generateSetupHTML());
    });

    // Captive-network probes used by Android, Apple, Windows, and Firefox.
    // Redirect them to the local dashboard so clients do not abandon the AP
    // merely because it intentionally has no internet connection.
    const char* portal_paths[] = {
        "/generate_204", "/gen_204", "/hotspot-detect.html",
        "/library/test/success.html", "/connecttest.txt", "/ncsi.txt",
        "/canonical.html", "/success.txt"
    };
    for (const char* path : portal_paths) {
        server_->on(path, HTTP_GET, [](AsyncWebServerRequest* request) {
            request->redirect("http://192.168.4.1/");
        });
    }

    // Bench test page
    server_->on("/bench", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "text/html", generateBenchHTML());
    });

    // Bench servo test
    server_->on("/api/servo/test", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleServoTest(request);
    });
    server_->on("/api/servo/response", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleServoResponse(request);
    });

    // SSE events
    server_->addHandler(events_);
    events_->onConnect([this](AsyncEventSourceClient* client) {
        if (client->lastId()) {
            char buf[32];
            snprintf(buf, sizeof(buf), "Last event ID: %u\n", client->lastId());
            client->write(buf, strlen(buf));
        }
        client->send("connected", NULL, millis(), 1000);
    });

    server_->onNotFound([](AsyncWebServerRequest* request) {
        request->redirect("http://192.168.4.1/");
    });

    server_->begin();
}

void WiFiManager::setupCaptivePortal() {
    if (dns_started_) dns_server_.stop();
    dns_server_.setErrorReplyCode(DNSReplyCode::NoError);
    dns_started_ = dns_server_.start(53, "*", WiFi.softAPIP());
    Serial.printf("[WiFi] Captive DNS: %s\n", dns_started_ ? "active" : "failed");
}

void WiFiManager::update() {
    uint32_t now = millis();
    if (dns_started_) dns_server_.processNextRequest();

    // Measure mechanical-system response using the IMU. This is intentionally
    // separate from PWM timing: without servo feedback the firmware can only
    // report when the safely suspended test article actually begins rotating.
    if (response_measurement_active_) {
        const uint32_t elapsed = now - response_command_ms_;
        if (state_) {
            response_samples_++;
            const float rate_change = fabsf(state_->angular_rate_dps - response_baseline_rate_dps_);
            response_yaw_delta_deg_ = normalizeDegrees180(state_->yaw_deg - response_baseline_yaw_deg_);
            response_peak_turn_deg_ = fmaxf(response_peak_turn_deg_, fabsf(response_yaw_delta_deg_));
            const float vertical_accel_change = fabsf(state_->vertical_accel_mps2 - response_baseline_vertical_accel_mps2_);
            const float sink_increase = fmaxf(0.0f, response_baseline_vertical_speed_mps_ - state_->vertical_speed_mps);
            const float ground_speed_loss = fmaxf(0.0f, response_baseline_ground_speed_mps_ - state_->ground_speed_mps);

            response_peak_rate_delta_dps_ = fmaxf(response_peak_rate_delta_dps_, rate_change);
            response_peak_vertical_accel_delta_mps2_ =
                fmaxf(response_peak_vertical_accel_delta_mps2_, vertical_accel_change);
            response_max_sink_increase_mps_ = fmaxf(response_max_sink_increase_mps_, sink_increase);
            response_max_ground_speed_loss_mps_ = fmaxf(response_max_ground_speed_loss_mps_, ground_speed_loss);

            // Unitless, conservative brake-effect score. It is intentionally
            // not labeled as aerodynamic drag force because there is no load
            // cell, airspeed probe, or calibrated canopy model in this vehicle.
            response_drag_proxy_ =
                response_max_ground_speed_loss_mps_ +
                response_max_sink_increase_mps_ +
                0.10f * response_peak_vertical_accel_delta_mps2_ +
                0.02f * response_peak_rate_delta_dps_;

            if (!response_measurement_detected_ && state_->imu_valid && elapsed >= 20 &&
                state_->angular_rate_dps >= 5.0f && rate_change >= 8.0f) {
                response_delay_ms_ = elapsed;
                response_measurement_detected_ = true;
            }
        }
        if (elapsed >= 1400) {
            response_measurement_active_ = false;
        }
    }

    if (servo_test_expires_ms_ != 0 &&
        static_cast<int32_t>(now - servo_test_expires_ms_) >= 0) {
        servo_test_expires_ms_ = 0;
        if (servos_) servos_->emergencyNeutral();
    }
    recoverAPIfNeeded(now);
    if (!ap_started_) return;

    state_->wifi_client_count = static_cast<uint8_t>(WiFi.softAPgetStationNum());

    if (now - last_push_ms_ >= push_interval_ms_) {
        pushEvents();
        last_push_ms_ = now;
    }
}

bool WiFiManager::isServoTestActive() const {
    return servo_test_expires_ms_ != 0 &&
           static_cast<int32_t>(servo_test_expires_ms_ - millis()) > 0;
}

void WiFiManager::pushEvents() {
    if (!events_ || events_->count() == 0) return;

    // Send live telemetry via SSE
    String json = getStatusJSON();
    events_->send(json.c_str(), "telemetry", millis());
}

String WiFiManager::getStatusJSON() {
    char buf[3072];
    const char* state_name = logic::flightStateName(state_->flight_state);
    const char* guidance_name = logic::guidanceModeName(state_->guidance_mode);
    const char* fail_name = logic::failCodeName(state_->failure_code);

    snprintf(buf, sizeof(buf),
        "{"
        "\"flight_state\":\"%s\","
        "\"guidance_mode\":\"%s\","
        "\"failsafe\":\"%s\","
        "\"uptime_ms\":%u,"
        "\"loop_rate_hz\":%.1f,"
        "\"free_heap_kb\":%u,"
        "\"wifi_clients\":%d,"
        "\"lora_rssi\":%d,"
        "\"lora_snr\":%.1f,"
        "\"max_altitude_agl\":%.1f,"
        "\"left_servo_command\":%.3f,"
        "\"right_servo_command\":%.3f,"
        "\"left_servo_us\":%.0f,"
        "\"right_servo_us\":%.0f,"
        "\"servo_1_angle_deg\":%.1f,"
        "\"servo_2_angle_deg\":%.1f,"
        "\"servo_1_turn_deg\":%.1f,"
        "\"servo_2_turn_deg\":%.1f,"
        "\"gps_fallback\":%s,"
        "\"imu_fallback\":%s,"
        "\"barometer_fallback\":%s,"
        "\"degraded_guidance\":%s,"
        "\"launch_readiness_ok\":%s,"
        "\"preflight_launch_warning\":%s,"
        "\"sensors\":{"
            "\"imu\":{\"valid\":%s,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"gyro_x\":%.1f,\"gyro_y\":%.1f,\"gyro_z\":%.1f,\"angular_rate\":%.1f,\"vertical_accel\":%.2f,\"age_ms\":%u},"
            "\"baro\":{\"valid\":%s,\"pressure_hpa\":%.2f,\"temperature_c\":%.1f,\"altitude_raw\":%.1f,\"altitude_agl\":%.1f,\"vs\":%.2f,\"age_ms\":%u},"
            "\"gps\":{\"valid\":%s,\"fix\":%s,\"uart_active\":%s,\"nmea_age_ms\":%u,\"lat\":%.7f,\"lon\":%.7f,\"altitude\":%.1f,\"sats\":%d,\"hdop\":%.1f,\"speed\":%.1f,\"course\":%.1f,\"age_ms\":%u}"
        "},"
        "\"navigation\":{"
            "\"target_lat\":%.7f,"
            "\"target_lon\":%.7f,"
            "\"distance_m\":%.1f,"
            "\"bearing_deg\":%.1f,"
            "\"course_deg\":%.1f,"
            "\"heading_error_deg\":%.1f,"
            "\"ground_speed_mps\":%.1f"
        "}"
        "}",
        state_name, guidance_name, fail_name,
        state_->timestamp_ms,
        state_->loop_rate_hz,
        state_->free_heap_bytes / 1024,
        WiFi.softAPgetStationNum(),
        state_->lora_rssi,
        state_->lora_snr,
        state_->max_altitude_agl_m,
        state_->left_servo_command,
        state_->right_servo_command,
        state_->left_servo_us,
        state_->right_servo_us,
        state_->left_servo_angle_deg,
        state_->right_servo_angle_deg,
        state_->left_servo_turn_deg,
        state_->right_servo_turn_deg,
        state_->gps_fallback_active ? "true" : "false",
        state_->imu_fallback_active ? "true" : "false",
        state_->barometer_fallback_active ? "true" : "false",
        state_->degraded_guidance ? "true" : "false",
        state_->launch_readiness_ok ? "true" : "false",
        state_->preflight_launch_warning ? "true" : "false",
        state_->imu_valid ? "true" : "false", state_->roll_deg, state_->pitch_deg, state_->yaw_deg,
        state_->gyro_x_dps, state_->gyro_y_dps, state_->gyro_z_dps, state_->angular_rate_dps, state_->vertical_accel_mps2, state_->imu_age_ms,
        state_->barometer_valid ? "true" : "false", state_->barometric_pressure_hpa,
        state_->barometric_temperature_c, state_->barometric_altitude_m,
        state_->altitude_agl_m, state_->vertical_speed_mps, state_->baro_age_ms,
        state_->gps_valid ? "true" : "false", state_->gps_valid ? "true" : "false",
        state_->gps_nmea_active ? "true" : "false", state_->gps_nmea_age_ms,
        state_->latitude, state_->longitude, state_->gps_altitude_m, state_->satellite_count, state_->hdop,
        state_->ground_speed_mps, state_->gps_course_deg, state_->gps_age_ms,
        state_->target_latitude, state_->target_longitude,
        state_->distance_to_target_m, state_->target_bearing_deg,
        state_->gps_course_deg, state_->heading_error_deg,
        state_->ground_speed_mps
    );
    return String(buf);
}

String WiFiManager::getSensorsJSON() {
    char buf[1280];
    snprintf(buf, sizeof(buf),
        "{"
        "\"imu\":{\"valid\":%s,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"gyro_x\":%.1f,\"gyro_y\":%.1f,\"gyro_z\":%.1f,\"angular_rate\":%.1f,\"vertical_accel\":%.2f,\"age_ms\":%u},"
        "\"baro\":{\"valid\":%s,\"pressure_hpa\":%.2f,\"temperature_c\":%.1f,\"altitude_raw\":%.1f,\"altitude_agl\":%.1f,\"vs\":%.2f,\"age_ms\":%u},"
        "\"gps\":{\"valid\":%s,\"fix\":%s,\"uart_active\":%s,\"nmea_age_ms\":%u,\"lat\":%.7f,\"lon\":%.7f,\"altitude\":%.1f,\"sats\":%d,\"hdop\":%.1f,\"speed\":%.1f,\"course\":%.1f,\"age_ms\":%u}"
        "}",
        state_->imu_valid ? "true" : "false", state_->roll_deg, state_->pitch_deg, state_->yaw_deg,
        state_->gyro_x_dps, state_->gyro_y_dps, state_->gyro_z_dps, state_->angular_rate_dps, state_->vertical_accel_mps2, state_->imu_age_ms,
        state_->barometer_valid ? "true" : "false", state_->barometric_pressure_hpa,
        state_->barometric_temperature_c, state_->barometric_altitude_m,
        state_->altitude_agl_m, state_->vertical_speed_mps, state_->baro_age_ms,
        state_->gps_valid ? "true" : "false", state_->gps_valid ? "true" : "false",
        state_->gps_nmea_active ? "true" : "false", state_->gps_nmea_age_ms,
        state_->latitude, state_->longitude, state_->gps_altitude_m, state_->satellite_count, state_->hdop,
        state_->ground_speed_mps, state_->gps_course_deg, state_->gps_age_ms
    );
    return String(buf);
}

String WiFiManager::getNavigationJSON() {
    char buf[1024];
    snprintf(buf, sizeof(buf),
        "{"
        "\"target_lat\":%.7f,"
        "\"target_lon\":%.7f,"
        "\"distance_m\":%.1f,"
        "\"bearing_deg\":%.1f,"
        "\"course_deg\":%.1f,"
        "\"heading_error_deg\":%.1f,"
        "\"ground_speed_mps\":%.1f"
        "}",
        state_->target_latitude, state_->target_longitude,
        state_->distance_to_target_m, state_->target_bearing_deg,
        state_->gps_course_deg, state_->heading_error_deg,
        state_->ground_speed_mps
    );
    return String(buf);
}

String WiFiManager::getConfigJSON() {
    const auto left = servos_ ? servos_->getLeftCalibration() : control::ServoController::Calibration{};
    const auto right = servos_ ? servos_->getRightCalibration() : control::ServoController::Calibration{};
    char buf[2048];
    snprintf(buf, sizeof(buf),
        "{"
        "\"target_lat\":%.7f,"
        "\"target_lon\":%.7f,"
        "\"guidance_kp\":%.4f,"
        "\"guidance_deadband\":%.1f,"
        "\"max_steering\":%.2f,"
        "\"servo_1_neutral\":%.0f,"
        "\"servo_2_neutral\":%.0f,"
        "\"servo_1_reversed\":%s,\"servo_2_reversed\":%s,"
        "\"servo_1_brake_us\":%.0f,\"servo_2_brake_us\":%.0f,"
        "\"servo_1_controls_left_brake\":%s,"
        "\"deployment_wait_ms\":%u,"
        "\"stabilization_ms\":%u,"
        "\"flare_enabled\":%s"
        "}",
        state_->target_latitude, state_->target_longitude,
        cfg::GUIDANCE_KP, cfg::GUIDANCE_DEADBAND_DEG, cfg::MAX_STEERING_COMMAND,
        left.neutral_us, right.neutral_us,
        left.reversed ? "true" : "false", right.reversed ? "true" : "false",
        left.max_brake_us, right.max_brake_us,
        servos_ && servos_->servo1ControlsLeftBrake() ? "true" : "false",
        cfg::DEPLOYMENT_WAIT_MS, cfg::PARAFOIL_STABILIZATION_MS,
        cfg::FLARE_ENABLED ? "true" : "false"
    );
    return String(buf);
}

String WiFiManager::getReadinessJSON() {
    const bool target = state_ && state_->target_latitude >= -90.0 && state_->target_latitude <= 90.0 &&
                        state_->target_longitude >= -180.0 && state_->target_longitude <= 180.0 &&
                        !(state_->target_latitude == 0.0 && state_->target_longitude == 0.0);
    const bool servos = servos_ && servos_->isHealthy();
    const bool preflight = state_ && (state_->flight_state == logic::FlightState::BOOT ||
                           state_->flight_state == logic::FlightState::SELF_TEST ||
                           state_->flight_state == logic::FlightState::PRE_LAUNCH ||
                           state_->flight_state == logic::FlightState::PAD_SAFE);
    const bool ready = state_ && preflight && target && servos && state_->gps_valid &&
                       state_->imu_valid && state_->barometer_valid && state_->failure_code == logic::FailCode::FAIL_NONE;
    char b[512];
    snprintf(b,sizeof(b),"{\"ready\":%s,\"preflight\":%s,\"target\":%s,\"servos\":%s,\"gps\":%s,\"imu\":%s,\"barometer\":%s,\"failsafe\":\"%s\",\"satellites\":%d,\"launch_readiness_ok\":%s,\"preflight_launch_warning\":%s}",
        ready?"true":"false",preflight?"true":"false",target?"true":"false",servos?"true":"false",
        state_&&state_->gps_valid?"true":"false",state_&&state_->imu_valid?"true":"false",
        state_&&state_->barometer_valid?"true":"false",state_?logic::failCodeName(state_->failure_code):"UNKNOWN",
        state_?state_->satellite_count:0,
        state_&&state_->launch_readiness_ok?"true":"false",
        state_&&state_->preflight_launch_warning?"true":"false");
    return String(b);
}

void WiFiManager::handleConfigPost(AsyncWebServerRequest* request) {
    if (state_->flight_state != logic::FlightState::PAD_SAFE &&
        state_->flight_state != logic::FlightState::BOOT &&
        state_->flight_state != logic::FlightState::SELF_TEST) {
        request->send(403, "application/json", "{\"error\":\"Config locked after launch\"}");
        return;
    }

    if (!request->hasParam("target_lat") || !request->hasParam("target_lon")) {
        request->send(400, "application/json", "{\"error\":\"Missing target coordinates\"}");
        return;
    }

    const double latitude = request->getParam("target_lat")->value().toDouble();
    const double longitude = request->getParam("target_lon")->value().toDouble();
    if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0 ||
        (latitude == 0.0 && longitude == 0.0)) {
        request->send(400, "application/json", "{\"error\":\"Invalid target coordinates\"}");
        return;
    }

    Preferences targetPrefs;
    if (!targetPrefs.begin("target_cfg", false)) {
        request->send(500, "application/json", "{\"error\":\"Could not open target storage\"}");
        return;
    }
    targetPrefs.putDouble("latitude", latitude);
    targetPrefs.putDouble("longitude", longitude);
    targetPrefs.end();
    state_->target_latitude = latitude;
    state_->target_longitude = longitude;
    if (servos_ && request->hasParam("servo_1_neutral") && request->hasParam("servo_2_neutral") &&
        request->hasParam("servo_1_brake") && request->hasParam("servo_2_brake")) {
        auto left = servos_->getLeftCalibration();
        auto right = servos_->getRightCalibration();
        left.neutral_us = constrain(request->getParam("servo_1_neutral")->value().toFloat(), 1100.0f, 1900.0f);
        right.neutral_us = constrain(request->getParam("servo_2_neutral")->value().toFloat(), 1100.0f, 1900.0f);
        left.max_brake_us = constrain(request->getParam("servo_1_brake")->value().toFloat(), 50.0f, 500.0f);
        right.max_brake_us = constrain(request->getParam("servo_2_brake")->value().toFloat(), 50.0f, 500.0f);
        left.reversed = request->hasParam("servo_1_reversed") && request->getParam("servo_1_reversed")->value() == "true";
        right.reversed = request->hasParam("servo_2_reversed") && request->getParam("servo_2_reversed")->value() == "true";
        servos_->setServo1ControlsLeftBrake(!request->hasParam("servo_1_side") ||
            request->getParam("servo_1_side")->value() != "right");
        servos_->setLeftCalibration(left); servos_->setRightCalibration(right);
        servos_->saveCalibration(); servos_->emergencyNeutral();
    }
    request->send(200, "application/json", "{\"status\":\"coordinates saved\"}");
}

void WiFiManager::handleReboot(AsyncWebServerRequest* request) {
    request->send(200, "application/json", "{\"status\":\"rebooting\"}");
    delay(100);
    ESP.restart();
}

void WiFiManager::handleServoTest(AsyncWebServerRequest* request) {
    if (!state_ || !servos_ || !servos_->isHealthy()) {
        request->send(503, "application/json", "{\"error\":\"Servo controller unavailable\"}");
        return;
    }

    const logic::FlightState flightState = state_->flight_state;
    const bool onGround = flightState == logic::FlightState::BOOT ||
                          flightState == logic::FlightState::SELF_TEST ||
                          flightState == logic::FlightState::PRE_LAUNCH ||
                          flightState == logic::FlightState::PAD_SAFE;
    if (!onGround) {
        request->send(403, "application/json", "{\"error\":\"Ground test locked outside pre-launch states\"}");
        return;
    }

    if (!request->hasParam("command")) {
        request->send(400, "application/json", "{\"error\":\"Missing command\"}");
        return;
    }

    const String command = request->getParam("command")->value();
    if (command == "neutral") {
        servo_test_expires_ms_ = 0;
        response_measurement_active_ = false;
        servos_->emergencyNeutral();
    } else if (command == "servo1_40") {
        servos_->setServoCommands(0.40f, 0.0f);
    } else if (command == "servo1_80") {
        servos_->setServoCommands(0.80f, 0.0f);
    } else if (command == "servo2_40") {
        servos_->setServoCommands(0.0f, 0.40f);
    } else if (command == "servo2_80") {
        servos_->setServoCommands(0.0f, 0.80f);
    } else if (command == "brake40") {
        servos_->setBrakeCommands(0.40f, 0.40f);
    } else {
        request->send(400, "application/json", "{\"error\":\"Invalid command\"}");
        return;
    }

    if (command != "neutral") {
        const uint32_t now = millis();
        servo_test_expires_ms_ = now + 1500;
        response_command_ = command;
        response_command_ms_ = now;
        response_delay_ms_ = 0;
        response_measurement_detected_ = false;
        response_measurement_active_ = true;
        response_baseline_rate_dps_ = state_->angular_rate_dps;
        response_baseline_yaw_deg_ = state_->yaw_deg;
        response_yaw_delta_deg_ = 0.0f;
        response_peak_turn_deg_ = 0.0f;
        response_baseline_vertical_speed_mps_ = state_->vertical_speed_mps;
        response_baseline_ground_speed_mps_ = state_->ground_speed_mps;
        response_baseline_vertical_accel_mps2_ = state_->vertical_accel_mps2;
        response_peak_rate_delta_dps_ = 0.0f;
        response_peak_vertical_accel_delta_mps2_ = 0.0f;
        response_max_sink_increase_mps_ = 0.0f;
        response_max_ground_speed_loss_mps_ = 0.0f;
        response_drag_proxy_ = 0.0f;
        response_samples_ = 0;
    }
    servos_->update();

    char response[256];
    snprintf(response, sizeof(response),
             "{\"status\":\"ok\",\"command\":\"%s\",\"servo_1_gpio\":4,\"servo_2_gpio\":6,"
             "\"servo_1_us\":%.0f,\"servo_2_us\":%.0f,"
             "\"servo_1_turn_deg\":%.1f,\"servo_2_turn_deg\":%.1f}",
             command.c_str(), servos_->getLeftUs(), servos_->getRightUs(),
             servos_->getLeftTurnDeg(), servos_->getRightTurnDeg());
    request->send(200, "application/json", response);
}

void WiFiManager::handleServoResponse(AsyncWebServerRequest* request) {
    char response[1024];
    const bool imu_valid = state_ && state_->imu_valid;
    const char* status = response_measurement_active_ ? "measuring" :
                         response_measurement_detected_ ? "detected" :
                         !imu_valid ? "imu_unavailable" : "not_detected";
    snprintf(response, sizeof(response),
             "{\"status\":\"%s\",\"command\":\"%s\",\"delay_ms\":%lu,"
             "\"rocket_turn_deg\":%.2f,\"peak_rocket_turn_deg\":%.2f,"
             "\"baseline_yaw_deg\":%.2f,\"yaw_deg\":%.2f,"
             "\"angular_rate_dps\":%.2f,\"baseline_rate_dps\":%.2f,"
             "\"peak_rate_delta_dps\":%.2f,"
             "\"baseline_vertical_speed_mps\":%.2f,\"vertical_speed_mps\":%.2f,"
             "\"max_sink_increase_mps\":%.2f,"
             "\"baseline_ground_speed_mps\":%.2f,\"ground_speed_mps\":%.2f,"
             "\"max_ground_speed_loss_mps\":%.2f,"
             "\"baseline_vertical_accel_mps2\":%.2f,\"vertical_accel_mps2\":%.2f,"
             "\"peak_vertical_accel_delta_mps2\":%.2f,"
             "\"drag_proxy\":%.2f,\"samples\":%u,"
             "\"imu_valid\":%s,\"barometer_valid\":%s,\"gps_valid\":%s,"
             "\"note\":\"drag_proxy is unitless; real drag force requires a load cell or airspeed sensor\"}",
             status, response_command_.c_str(), (unsigned long)response_delay_ms_,
             response_yaw_delta_deg_, response_peak_turn_deg_,
             response_baseline_yaw_deg_, state_ ? state_->yaw_deg : 0.0f,
             state_ ? state_->angular_rate_dps : 0.0f, response_baseline_rate_dps_,
             response_peak_rate_delta_dps_,
             response_baseline_vertical_speed_mps_, state_ ? state_->vertical_speed_mps : 0.0f,
             response_max_sink_increase_mps_,
             response_baseline_ground_speed_mps_, state_ ? state_->ground_speed_mps : 0.0f,
             response_max_ground_speed_loss_mps_,
             response_baseline_vertical_accel_mps2_, state_ ? state_->vertical_accel_mps2 : 0.0f,
             response_peak_vertical_accel_delta_mps2_,
             response_drag_proxy_, response_samples_,
             imu_valid ? "true" : "false",
             state_ && state_->barometer_valid ? "true" : "false",
             state_ && state_->gps_valid ? "true" : "false");
    request->send(200, "application/json", response);
}

// ============================================================================
// HTML PAGE GENERATION
// ============================================================================

String WiFiManager::generateDashboardHTML() {
    return R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PHOENIX RECOVERY</title>
<style>
:root { --bg:#0d1117; --card:#161b22; --text:#e6edf3; --muted:#8b949e; --accent:#58a6ff; --ok:#3fb950; --warn:#d29922; --err:#f85149; --border:#30363d; }
* { box-sizing:border-box; margin:0; padding:0; font-family: system-ui, -apple-system, sans-serif; }
body { background:var(--bg); color:var(--text); min-height:100vh; padding:16px; }
h1 { font-size:1.5rem; font-weight:600; margin-bottom:8px; }
.header { display:flex; justify-content:space-between; align-items:center; margin-bottom:16px; padding-bottom:12px; border-bottom:1px solid var(--border); }
.badge { padding:4px 10px; border-radius:4px; font-size:0.75rem; font-weight:600; text-transform:uppercase; }
.badge-ready { background:#1f6feb22; color:#58a6ff; }
.badge-flight { background:#f7816622; color:#f78166; }
.badge-guidance { background:#a371f722; color:#a371f7; }
.badge-landed { background:#3fb95022; color:#3fb950; }
.badge-failsafe { background:#f8514922; color:#f85149; }
.grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(280px,1fr)); gap:12px; }
.card { background:var(--card); border:1px solid var(--border); border-radius:8px; padding:16px; }
.card h2 { font-size:0.85rem; font-weight:600; color:var(--muted); text-transform:uppercase; letter-spacing:0.5px; margin-bottom:12px; border-bottom:1px solid var(--border); padding-bottom:8px; }
.row { display:flex; justify-content:space-between; padding:6px 0; border-bottom:1px solid var(--border); }
.row:last-child { border-bottom:none; }
.label { color:var(--muted); font-size:0.9rem; }
.value { font-weight:600; font-family: ui-monospace, monospace; text-align:right; }
.value.ok { color:var(--ok); }
.value.warn { color:var(--warn); }
.value.err { color:var(--err); }
.value.accent { color:var(--accent); }
.links { margin-top:16px; display:flex; gap:12px; flex-wrap:wrap; }
.links a { color:var(--accent); text-decoration:none; font-size:0.9rem; }
.links a:hover { text-decoration:underline; }
.connection { font-size:.8rem; color:var(--muted); margin:-8px 0 14px; }
.connection.live { color:var(--ok); }
.connection.error { color:var(--err); }
.launch-warning { display:none; margin:0 0 14px; padding:12px; border-radius:8px; border:1px solid var(--err); background:#f8514922; color:var(--err); font-weight:700; }
</style>
</head>
<body>
<div class="header">
<h1>PHOENIX RECOVERY</h1>
<span id="stateBadge" class="badge badge-ready">BOOT</span>
</div>
<div id="connection" class="connection">Connecting to live telemetry…</div>
<div id="launchWarning" class="launch-warning">⚠ Launch attempt detected while the system was not ready. FAILSAFE_DESCENT is active and servos are neutral.</div>

<div class="grid">
<div class="card">
<h2>Flight State</h2>
<div class="row"><span class="label">State</span><span id="fs" class="value accent">BOOT</span></div>
<div class="row"><span class="label">Guidance</span><span id="gm" class="value">DISABLED</span></div>
<div class="row"><span class="label">Failsafe</span><span id="fc" class="value ok">NONE</span></div>
<div class="row"><span class="label">Fallback</span><span id="fallback" class="value ok">NONE</span></div>
<div class="row"><span class="label">Uptime</span><span id="up" class="value">0s</span></div>
<div class="row"><span class="label">Loop Rate</span><span id="lr" class="value">0 Hz</span></div>
</div>

<div class="card">
<h2>Live Sensor Data</h2>
<div class="row"><span class="label">Sensor Summary</span><span id="sensorSummary" class="value warn">WAITING</span></div>
<div class="row"><span class="label">IMU Roll / Pitch / Yaw</span><span id="imuSnapshot" class="value">0.0° / 0.0° / 0.0°</span></div>
<div class="row"><span class="label">Barometer</span><span id="baroSnapshot" class="value">0.00 hPa / 0.0 m</span></div>
<div class="row"><span class="label">GPS</span><span id="gpsSnapshot" class="value">NO FIX / 0 sats</span></div>
<div class="row"><span class="label">Sensor Ages</span><span id="sensorAges" class="value">IMU -- / BARO -- / GPS --</span></div>
</div>

<div class="card">
<h2>Altitude & Vertical</h2>
<div class="row"><span class="label">Altitude AGL</span><span id="alt" class="value">0.0 m</span></div>
<div class="row"><span class="label">Vertical Speed</span><span id="vs" class="value">0.00 m/s</span></div>
<div class="row"><span class="label">Max Altitude</span><span id="maxalt" class="value">0.0 m</span></div>
<div class="row"><span class="label">Raw Baro Altitude</span><span id="rawalt" class="value">0.0 m</span></div>
<div class="row"><span class="label">Pressure</span><span id="pressure" class="value">0.00 hPa</span></div>
<div class="row"><span class="label">Temperature</span><span id="temperature" class="value">0.0 °C</span></div>
<div class="row"><span class="label">Barometer Age</span><span id="bage" class="value">-- ms</span></div>
</div>

<div class="card">
<h2>GPS</h2>
<div class="row"><span class="label">Latitude</span><span id="lat" class="value">0.0000000</span></div>
<div class="row"><span class="label">Longitude</span><span id="lon" class="value">0.0000000</span></div>
<div class="row"><span class="label">Satellites</span><span id="sats" class="value">0</span></div>
<div class="row"><span class="label">HDOP</span><span id="hdop" class="value">0.0</span></div>
<div class="row"><span class="label">Ground Speed</span><span id="gs" class="value">0.0 m/s</span></div>
<div class="row"><span class="label">Course</span><span id="crs" class="value">0.0°</span></div>
<div class="row"><span class="label">Fix Valid</span><span id="gfix" class="value err">NO</span></div>
<div class="row"><span class="label">GPS UART/NMEA</span><span id="guart" class="value err">NO DATA</span></div>
<div class="row"><span class="label">NMEA Age</span><span id="gnmea" class="value">-- ms</span></div>
<div class="row"><span class="label">GPS Altitude</span><span id="galt" class="value">0.0 m</span></div>
<div class="row"><span class="label">GPS Age</span><span id="gage" class="value">-- ms</span></div>
</div>

<div class="card">
<h2>Target & Guidance</h2>
<div class="row"><span class="label">Target Lat</span><span id="tlat" class="value">0.0000000</span></div>
<div class="row"><span class="label">Target Lon</span><span id="tlon" class="value">0.0000000</span></div>
<div class="row"><span class="label">Distance</span><span id="dist" class="value">0 m</span></div>
<div class="row"><span class="label">Bearing</span><span id="brg" class="value">0.0°</span></div>
<div class="row"><span class="label">Heading Error</span><span id="herr" class="value">0.0°</span></div>
</div>

<div class="card">
<h2>Servos</h2>
<div class="row"><span class="label">Servo 1 Command</span><span id="lcmd" class="value">0.000</span></div>
<div class="row"><span class="label">Servo 2 Command</span><span id="rcmd" class="value">0.000</span></div>
<div class="row"><span class="label">Servo 1 μs</span><span id="lus" class="value">1500</span></div>
<div class="row"><span class="label">Servo 2 μs</span><span id="rus" class="value">1500</span></div>
<div class="row"><span class="label">Servo 1 Turn</span><span id="lturn" class="value">0.0°</span></div>
<div class="row"><span class="label">Servo 2 Turn</span><span id="rturn" class="value">0.0°</span></div>
<div class="row"><span class="label">Servo 1 Angle Est.</span><span id="lang" class="value">90.0°</span></div>
<div class="row"><span class="label">Servo 2 Angle Est.</span><span id="rang" class="value">90.0°</span></div>
<div style="margin-top:12px;padding-top:10px;border-top:1px solid var(--border)">
<div class="label" style="margin-bottom:8px">GROUND TEST • Servo 1 GPIO 4 / Servo 2 GPIO 6</div>
<button onclick="groundServo('servo1_40')">Servo 1 40%</button>
<button onclick="groundServo('servo1_80')">Servo 1 80%</button>
<button onclick="groundServo('servo2_40')">Servo 2 40%</button>
<button onclick="groundServo('servo2_80')">Servo 2 80%</button>
<button onclick="groundServo('brake40')">Both 40%</button>
<button onclick="groundServo('neutral')">Neutral</button>
<div id="groundServoStatus" class="connection" style="margin:8px 0 0">Remove servo horns/loads before testing.</div>
</div>
</div>

<div class="card">
<h2>Module Health</h2>
<div class="row"><span class="label">IMU</span><span id="imu" class="value err">INIT</span></div>
<div class="row"><span class="label">Barometer</span><span id="baro" class="value err">INIT</span></div>
<div class="row"><span class="label">LoRa</span><span id="lora" class="value">-- dBm</span></div>
</div>

<div class="card">
<h2>IMU Output</h2>
<div class="row"><span class="label">Roll</span><span id="roll" class="value">0.0°</span></div>
<div class="row"><span class="label">Pitch</span><span id="pitch" class="value">0.0°</span></div>
<div class="row"><span class="label">Yaw</span><span id="yaw" class="value">0.0°</span></div>
<div class="row"><span class="label">Gyro X/Y/Z</span><span id="gyro" class="value">0 / 0 / 0 °/s</span></div>
<div class="row"><span class="label">Angular Rate</span><span id="arate" class="value">0.0 °/s</span></div>
<div class="row"><span class="label">Vertical Accel</span><span id="vaccel" class="value">0.00 m/s²</span></div>
<div class="row"><span class="label">IMU Age</span><span id="iage" class="value">-- ms</span></div>
</div>

<div class="card">
<h2>System</h2>
<div class="row"><span class="label">Free Heap</span><span id="heap" class="value">0 KB</span></div>
<div class="row"><span class="label">WiFi Clients</span><span id="clients" class="value">0</span></div>
</div>
</div>

<div class="links">
<a href="/setup">✅ Preflight Setup</a>
<a href="/config">⚙ Configuration</a>
<a href="/bench">🔧 Bench Test</a>
</div>

<script>
async function groundServo(command) {
    const status = document.getElementById('groundServoStatus');
    status.textContent = 'Sending '+command+'…';
    try {
        const r = await fetch('/api/servo/test?command='+encodeURIComponent(command), {method:'POST'});
        const result = await r.json();
        if (!r.ok || command === 'neutral') {
            status.textContent = r.ok ? 'Neutral command accepted' : result.error;
            status.className = 'connection '+(r.ok ? 'live' : 'error');
            return;
        }
        status.textContent = 'Command accepted; Servo 1 '+result.servo_1_turn_deg.toFixed(1)+'°, Servo 2 '+result.servo_2_turn_deg.toFixed(1)+'°. Measuring rocket turn…';
        status.className = 'connection '+(r.ok ? 'live' : 'error');
        setTimeout(async () => {
            try {
                const m = await fetch('/api/servo/response').then(x => x.json());
                status.textContent =
                    'Servo 1 '+result.servo_1_turn_deg.toFixed(1)+'°, Servo 2 '+result.servo_2_turn_deg.toFixed(1)+'°; '+
                    'rocket/payload turned '+m.rocket_turn_deg.toFixed(1)+'° yaw '+
                    '(peak '+m.peak_rocket_turn_deg.toFixed(1)+'°). Automatic neutral in 1.5 s.';
            } catch (_) {}
        }, 1450);
    } catch (_) {
        status.textContent = 'Servo command failed';
        status.className = 'connection error';
    }
}
// SSE live updates
const evt = new EventSource('/events');
const connection = document.getElementById('connection');
evt.onopen = () => { connection.textContent='LIVE • telemetry connected'; connection.className='connection live'; };
evt.onerror = () => { connection.textContent='Telemetry disconnected — retrying…'; connection.className='connection error'; };
evt.addEventListener('telemetry', e => {
    let d; try { d = JSON.parse(e.data); } catch (_) { connection.textContent='Invalid telemetry payload'; connection.className='connection error'; return; }
    document.getElementById('fs').textContent = d.flight_state;
    document.getElementById('gm').textContent = d.guidance_mode;
    document.getElementById('fc').textContent = d.failsafe;
    const launchWarning = document.getElementById('launchWarning');
    launchWarning.style.display = d.preflight_launch_warning ? 'block' : 'none';
    launchWarning.textContent = '⚠ Launch attempt detected while not ready. Failsafe: '+d.failsafe+'. Servos commanded neutral.';
    const fallbacks = [
        d.gps_fallback ? 'GPS' : '',
        d.imu_fallback ? 'IMU' : '',
        d.barometer_fallback ? 'BARO' : ''
    ].filter(Boolean).join(' + ');
    document.getElementById('fallback').textContent = fallbacks || 'NONE';
    document.getElementById('fallback').className = 'value '+(fallbacks ? 'warn' : 'ok');
    document.getElementById('up').textContent = (d.uptime_ms/1000).toFixed(1)+'s';
    document.getElementById('lr').textContent = d.loop_rate_hz.toFixed(1)+' Hz';
    document.getElementById('heap').textContent = d.free_heap_kb+' KB';
    document.getElementById('clients').textContent = d.wifi_clients;
    document.getElementById('lora').textContent = d.lora_rssi+' dBm / '+d.lora_snr+' dB';
    document.getElementById('maxalt').textContent = d.max_altitude_agl.toFixed(1)+' m';
    document.getElementById('lcmd').textContent = d.left_servo_command.toFixed(3);
    document.getElementById('rcmd').textContent = d.right_servo_command.toFixed(3);
    document.getElementById('lus').textContent = d.left_servo_us.toFixed(0);
    document.getElementById('rus').textContent = d.right_servo_us.toFixed(0);
    document.getElementById('lturn').textContent = d.servo_1_turn_deg.toFixed(1)+'°';
    document.getElementById('rturn').textContent = d.servo_2_turn_deg.toFixed(1)+'°';
    document.getElementById('lang').textContent = d.servo_1_angle_deg.toFixed(1)+'°';
    document.getElementById('rang').textContent = d.servo_2_angle_deg.toFixed(1)+'°';

    const badge = document.getElementById('stateBadge');
    badge.textContent = d.flight_state;
    badge.className = 'badge ' +
        (d.flight_state==='PAD_SAFE'?'badge-ready':
         d.flight_state==='ASCENT'||d.flight_state==='APOGEE_CONFIRMED'?'badge-flight':
         d.flight_state==='GUIDED_DESCENT'||d.flight_state==='FINAL_APPROACH'?'badge-guidance':
         d.flight_state==='LANDED'?'badge-landed':
         d.flight_state==='FAILSAFE_DESCENT'?'badge-failsafe':'');

    if (d.sensors) updateSensors(d.sensors);
    if (d.navigation) updateNav(d.navigation);
});

// Poll for sensor/navigation data (less frequent)
async function poll() {
    try {
        const [s, n] = await Promise.all([
            fetch('/api/sensors').then(r=>r.json()),
            fetch('/api/navigation').then(r=>r.json())
        ]);
        updateSensors(s);
        updateNav(n);
    } catch(e) { connection.textContent='Module API error: '+e.message; connection.className='connection error'; }
    setTimeout(poll, 1000);
}
poll();

function updateSensors(d) {
    const imuOk = !!d.imu.valid;
    const baroOk = !!d.baro.valid;
    const gpsOk = !!d.gps.valid;
    const okCount = [imuOk, baroOk, gpsOk].filter(Boolean).length;
    const summary = okCount === 3 ? 'ALL OK' : okCount + '/3 OK';
    document.getElementById('sensorSummary').textContent = summary;
    document.getElementById('sensorSummary').className = 'value '+(okCount === 3 ? 'ok' : okCount > 0 ? 'warn' : 'err');
    document.getElementById('imuSnapshot').textContent =
        (imuOk ? 'OK ' : 'ERROR ') + d.imu.roll.toFixed(1)+'° / '+d.imu.pitch.toFixed(1)+'° / '+d.imu.yaw.toFixed(1)+'°';
    document.getElementById('imuSnapshot').className = 'value '+(imuOk?'ok':'err');
    document.getElementById('baroSnapshot').textContent =
        (baroOk ? 'OK ' : 'ERROR ') + d.baro.pressure_hpa.toFixed(2)+' hPa / '+d.baro.altitude_agl.toFixed(1)+' m AGL';
    document.getElementById('baroSnapshot').className = 'value '+(baroOk?'ok':'err');
    document.getElementById('gpsSnapshot').textContent =
        (d.gps.fix ? 'FIX ' : (d.gps.uart_active ? 'NMEA, NO FIX ' : 'NO UART DATA ')) +
        d.gps.sats+' sats / '+d.gps.lat.toFixed(7)+', '+d.gps.lon.toFixed(7);
    document.getElementById('gpsSnapshot').className = 'value '+(gpsOk?'ok':(d.gps.uart_active?'warn':'err'));
    document.getElementById('sensorAges').textContent =
        'IMU '+d.imu.age_ms+' ms / BARO '+d.baro.age_ms+' ms / GPS '+formatAge(d.gps.age_ms);

    document.getElementById('alt').textContent = d.baro.altitude_agl.toFixed(1)+' m';
    document.getElementById('vs').textContent = d.baro.vs.toFixed(2)+' m/s';
    document.getElementById('rawalt').textContent = d.baro.altitude_raw.toFixed(1)+' m';
    document.getElementById('pressure').textContent = d.baro.pressure_hpa.toFixed(2)+' hPa';
    document.getElementById('temperature').textContent = d.baro.temperature_c.toFixed(1)+' °C';
    document.getElementById('bage').textContent = d.baro.age_ms+' ms';
    document.getElementById('lat').textContent = d.gps.lat.toFixed(7);
    document.getElementById('lon').textContent = d.gps.lon.toFixed(7);
    document.getElementById('sats').textContent = d.gps.sats;
    document.getElementById('hdop').textContent = d.gps.hdop.toFixed(1);
    document.getElementById('gs').textContent = d.gps.speed.toFixed(1)+' m/s';
    document.getElementById('crs').textContent = d.gps.course.toFixed(1)+'°';
    document.getElementById('gfix').textContent = d.gps.fix?'YES':'NO';
    document.getElementById('gfix').className = 'value '+(d.gps.fix?'ok':'err');
    document.getElementById('guart').textContent = d.gps.uart_active?'RECEIVING':'NO DATA';
    document.getElementById('guart').className = 'value '+(d.gps.uart_active?'ok':'err');
    document.getElementById('gnmea').textContent = formatAge(d.gps.nmea_age_ms);
    document.getElementById('galt').textContent = d.gps.altitude.toFixed(1)+' m';
    document.getElementById('gage').textContent = formatAge(d.gps.age_ms);
    document.getElementById('imu').textContent = d.imu.valid?'OK':'ERROR';
    document.getElementById('imu').className = 'value '+(d.imu.valid?'ok':'err');
    document.getElementById('baro').textContent = d.baro.valid?'OK':'ERROR';
    document.getElementById('baro').className = 'value '+(d.baro.valid?'ok':'err');
    document.getElementById('roll').textContent = d.imu.roll.toFixed(1)+'°';
    document.getElementById('pitch').textContent = d.imu.pitch.toFixed(1)+'°';
    document.getElementById('yaw').textContent = d.imu.yaw.toFixed(1)+'°';
    document.getElementById('gyro').textContent = d.imu.gyro_x.toFixed(1)+' / '+d.imu.gyro_y.toFixed(1)+' / '+d.imu.gyro_z.toFixed(1)+' °/s';
    document.getElementById('arate').textContent = d.imu.angular_rate.toFixed(1)+' °/s';
    document.getElementById('vaccel').textContent = d.imu.vertical_accel.toFixed(2)+' m/s²';
    document.getElementById('iage').textContent = d.imu.age_ms+' ms';
}
function formatAge(ms) {
    return ms >= 4294960000 ? 'never' : ms+' ms';
}
function updateNav(d) {
    document.getElementById('tlat').textContent = d.target_lat.toFixed(7);
    document.getElementById('tlon').textContent = d.target_lon.toFixed(7);
    document.getElementById('dist').textContent = d.distance_m.toFixed(1)+' m';
    document.getElementById('brg').textContent = d.bearing_deg.toFixed(1)+'°';
    document.getElementById('herr').textContent = d.heading_error_deg.toFixed(1)+'°';
    document.getElementById('herr').className = 'value '+(Math.abs(d.heading_error_deg)>5?'warn':'ok');
}
</script>
</body>
</html>
)HTML";
}

String WiFiManager::generateConfigHTML() {
    return R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PHOENIX RECOVERY — Configuration</title>
<style>
:root { --bg:#0d1117; --card:#161b22; --text:#e6edf3; --muted:#8b949e; --accent:#58a6ff; --border:#30363d; --input:#0d1117; }
* { box-sizing:border-box; margin:0; padding:0; font-family: system-ui, -apple-system, sans-serif; }
body { background:var(--bg); color:var(--text); min-height:100vh; padding:16px; }
h1 { font-size:1.5rem; margin-bottom:8px; }
.card { background:var(--card); border:1px solid var(--border); border-radius:8px; padding:16px; margin-bottom:16px; }
.card h2 { font-size:0.85rem; color:var(--muted); text-transform:uppercase; margin-bottom:12px; }
.field { display:flex; flex-direction:column; gap:4px; margin-bottom:12px; }
.field label { font-size:0.85rem; color:var(--muted); }
.field input { background:var(--input); border:1px solid var(--border); border-radius:4px; padding:8px 12px; color:var(--text); font-family:ui-monospace,monospace; }
.field input:focus { outline:none; border-color:var(--accent); }
.btn { background:var(--accent); color:#fff; border:none; border-radius:4px; padding:10px 16px; font-weight:600; cursor:pointer; }
.btn:hover { filter:brightness(1.1); }
.btn-danger { background:#f85149; }
.btn-warn { background:#d29922; }
.row { display:flex; gap:12px; flex-wrap:wrap; }
.row .field { flex:1; min-width:200px; }
</style>
</head>
<body>
<h1>⚙ Configuration</h1>

<div class="card">
<h2>Landing Target</h2>
<div class="row">
<div class="field"><label>Target Latitude</label><input type="number" step="0.0000001" id="tlat" placeholder="Required"></div>
<div class="field"><label>Target Longitude</label><input type="number" step="0.0000001" id="tlon" placeholder="Required"></div>
</div>
<button class="btn" onclick="useComputerLocation()">📍 Use This Device's Location</button>
<div id="locationStatus" style="color:var(--muted);margin-top:8px">Coordinates default to this device, then rocket GPS if browser location is unavailable.</div>
</div>

<div class="card">
<h2>Guidance Tuning</h2>
<div class="row">
<div class="field"><label>Kp (heading)</label><input type="number" step="0.001" id="kp"></div>
<div class="field"><label>Deadband (deg)</label><input type="number" step="0.1" id="deadband"></div>
<div class="field"><label>Max Steering</label><input type="number" step="0.01" id="maxsteer"></div>
</div>
</div>

<div class="card">
<h2>Servo Calibration</h2>
<div class="row">
<div class="field"><label>Servo 1 Neutral (μs)</label><input type="number" id="lneut"></div>
<div class="field"><label>Servo 2 Neutral (μs)</label><input type="number" id="rneut"></div>
</div>
<div class="row">
<div class="field"><label>Servo 1 Maximum Travel (μs, 50–500)</label><input type="number" min="50" max="500" id="lbrake"></div>
<div class="field"><label>Servo 2 Maximum Travel (μs, 50–500)</label><input type="number" min="50" max="500" id="rbrake"></div>
</div>
<div class="row">
<div class="field"><label>Servo 1 Reversed</label><input type="checkbox" id="lrev"></div>
<div class="field"><label>Servo 2 Reversed</label><input type="checkbox" id="rrev"></div>
</div>
<div class="field"><label>Servo 1 brake-line side</label><select id="s1side"><option value="left">Left brake line</option><option value="right">Right brake line</option></select></div>
</div>

<div class="card">
<h2>Flight Timing</h2>
<div class="row">
<div class="field"><label>Deployment Wait (ms)</label><input type="number" id="dwait"></div>
<div class="field"><label>Stabilization (ms)</label><input type="number" id="stab"></div>
</div>
</div>

<div class="card">
<h2>Flare (Experimental)</h2>
<div class="field"><label><input type="checkbox" id="flare"> Enable Flare (DISABLED BY DEFAULT)</label></div>
</div>

<div style="display:flex; gap:12px; flex-wrap:wrap;">
<button class="btn" onclick="saveConfig()">💾 Save to NVS</button>
<button class="btn btn-warn" onclick="restoreDefaults()">↩ Restore Defaults</button>
<button class="btn btn-danger" onclick="reboot()">🔄 Reboot</button>
<a href="/" style="align-self:center; color:var(--accent); text-decoration:none;">← Back to Dashboard</a>
</div>

<script>
async function loadConfig() {
    const r = await fetch('/api/config');
    const c = await r.json();
    document.getElementById('tlat').value = c.target_lat || '';
    document.getElementById('tlon').value = c.target_lon || '';
    document.getElementById('kp').value = c.guidance_kp || '';
    document.getElementById('deadband').value = c.guidance_deadband || '';
    document.getElementById('maxsteer').value = c.max_steering || '';
    document.getElementById('lneut').value = c.servo_1_neutral || '';
    document.getElementById('rneut').value = c.servo_2_neutral || '';
    document.getElementById('lbrake').value = c.servo_1_brake_us || '';
    document.getElementById('rbrake').value = c.servo_2_brake_us || '';
    document.getElementById('lrev').checked = c.servo_1_reversed === true;
    document.getElementById('rrev').checked = c.servo_2_reversed === true;
    document.getElementById('s1side').value = c.servo_1_controls_left_brake === true ? 'left' : 'right';
    document.getElementById('dwait').value = c.deployment_wait_ms || '';
    document.getElementById('stab').value = c.stabilization_ms || '';
    document.getElementById('flare').checked = c.flare_enabled === 'true';
    if (!c.target_lat && !c.target_lon) useComputerLocation();
}
loadConfig();

async function useComputerLocation() {
    const status = document.getElementById('locationStatus');
    status.textContent = 'Finding this device’s location…';
    const apply = (lat, lon, source) => {
        document.getElementById('tlat').value = Number(lat).toFixed(7);
        document.getElementById('tlon').value = Number(lon).toFixed(7);
        status.textContent = source + ' coordinates loaded. Press Save to make them the landing target.';
    };
    const gpsFallback = async () => {
        try {
            const s = await fetch('/api/sensors').then(r => r.json());
            if (s.gps.fix && (s.gps.lat || s.gps.lon)) return apply(s.gps.lat, s.gps.lon, 'Rocket GPS');
        } catch (_) {}
        status.textContent = 'Location unavailable. Enable location permission or wait for rocket GPS lock.';
    };
    if (!navigator.geolocation) return gpsFallback();
    navigator.geolocation.getCurrentPosition(
        p => apply(p.coords.latitude, p.coords.longitude, 'Computer/phone'),
        gpsFallback,
        {enableHighAccuracy:true, timeout:8000, maximumAge:30000}
    );
}

async function saveConfig() {
    const data = {
        target_lat: parseFloat(document.getElementById('tlat').value),
        target_lon: parseFloat(document.getElementById('tlon').value),
        guidance_kp: parseFloat(document.getElementById('kp').value),
        guidance_deadband: parseFloat(document.getElementById('deadband').value),
        max_steering: parseFloat(document.getElementById('maxsteer').value),
        servo_left_neutral: parseFloat(document.getElementById('lneut').value),
        servo_right_neutral: parseFloat(document.getElementById('rneut').value),
        servo_left_brake: parseFloat(document.getElementById('lbrake').value),
        servo_right_brake: parseFloat(document.getElementById('rbrake').value),
        servo_left_reversed: document.getElementById('lrev').checked,
        servo_right_reversed: document.getElementById('rrev').checked,
        deployment_wait_ms: parseInt(document.getElementById('dwait').value),
        stabilization_ms: parseInt(document.getElementById('stab').value),
        flare_enabled: document.getElementById('flare').checked
    };
    const query = new URLSearchParams({target_lat:data.target_lat,target_lon:data.target_lon,
      servo_1_neutral:data.servo_left_neutral,servo_2_neutral:data.servo_right_neutral,
      servo_1_brake:data.servo_left_brake,servo_2_brake:data.servo_right_brake,
      servo_1_reversed:data.servo_left_reversed,servo_2_reversed:data.servo_right_reversed,
      servo_1_side:document.getElementById('s1side').value});
    const r = await fetch('/api/config?'+query.toString(), {method:'POST'});
    const j = await r.json();
    alert(j.status || j.error);
}
async function restoreDefaults() { if(confirm('Restore all defaults?')) alert('Not implemented'); }
async function reboot() { if(confirm('Reboot?')) { await fetch('/api/reboot',{method:'POST'}); setTimeout(()=>location.reload(),2000); } }
</script>
</body>
</html>
)HTML";
}

String WiFiManager::generateSetupHTML() {
    return R"HTML(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PHOENIX RECOVERY Setup</title><style>:root{--bg:#0d1117;--card:#161b22;--text:#e6edf3;--muted:#8b949e;--ok:#3fb950;--bad:#f85149;--accent:#58a6ff}*{box-sizing:border-box}body{font-family:system-ui;background:var(--bg);color:var(--text);margin:0;padding:18px;max-width:850px}h1{margin-top:0}.card{background:var(--card);padding:16px;border-radius:10px;margin:12px 0}.item{display:flex;justify-content:space-between;padding:9px;border-bottom:1px solid #30363d}.ok{color:var(--ok)}.bad{color:var(--bad)}a,button{color:white;background:var(--accent);padding:11px 14px;border:0;border-radius:6px;text-decoration:none;display:inline-block;margin:5px}.banner{font-size:1.2rem;font-weight:700}</style></head><body>
<h1>PHOENIX RECOVERY • Preflight Setup</h1><div class="card"><div id="banner" class="banner">Checking…</div><p id="note"></p></div>
<div class="card"><h2>Readiness</h2><div id="checks"></div></div>
<div class="card"><h2>Finish setup</h2><a href="/config">1. Target & Servo Calibration</a><a href="/bench">2. Safe Servo Tests</a><a href="/">3. Live Telemetry</a><button onclick="refresh()">Recheck</button></div>
<div class="card"><strong>Flight lock:</strong> Configuration and servo tests are rejected after launch. Wi-Fi and LoRa are monitoring tools only and are not required during descent.</div>
<script>const labels={preflight:'Pre-launch state',target:'Landing target saved',servos:'Servo controller',gps:'L76K GNSS fix',imu:'BNO085 IMU',barometer:'BMP388 barometer'};async function refresh(){try{const d=await fetch('/api/readiness').then(r=>r.json());banner.textContent=d.preflight_launch_warning?'🚨 FAILSAFE ACTIVE — LAUNCH ATTEMPT NOT READY':(d.ready?'✅ READY FOR CONTROLLED PREFLIGHT TEST':'⚠ NOT READY');banner.className='banner '+(d.ready&&!d.preflight_launch_warning?'ok':'bad');note.textContent=d.preflight_launch_warning?'The system detected launch-like motion before all required checks passed. Servos are commanded neutral; inspect the failed item below before any further testing.':(d.ready?'All electronic readiness gates pass. This does not validate parafoil aerodynamics.':'Complete every failed item below. GNSS normally requires a clear view of the sky.');checks.innerHTML=Object.keys(labels).map(k=>`<div class="item"><span>${labels[k]}</span><b class="${d[k]?'ok':'bad'}">${d[k]?'PASS':'WAIT'}</b></div>`).join('')+`<div class="item"><span>Launch readiness</span><b class="${d.launch_readiness_ok?'ok':'bad'}">${d.launch_readiness_ok?'OK':'NOT READY'}</b></div><div class="item"><span>Failsafe</span><b class="${d.failsafe==='NONE'?'ok':'bad'}">${d.failsafe}</b></div><div class="item"><span>Satellites</span><b>${d.satellites}</b></div>`}catch(e){banner.textContent='Dashboard connection lost';banner.className='banner bad'}}refresh();setInterval(refresh,2000)</script></body></html>)HTML";
}

String WiFiManager::generateBenchHTML() {
    return R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PHOENIX RECOVERY — Bench Test</title>
<style>
:root { --bg:#0d1117; --card:#161b22; --text:#e6edf3; --muted:#8b949e; --accent:#58a6ff; --ok:#3fb950; --err:#f85149; --border:#30363d; }
* { box-sizing:border-box; margin:0; padding:0; font-family: system-ui, -apple-system, sans-serif; }
body { background:var(--bg); color:var(--text); min-height:100vh; padding:16px; }
h1 { font-size:1.5rem; margin-bottom:8px; }
.warning { background:#f8514922; border:1px solid #f85149; color:#f85149; padding:12px; border-radius:8px; margin-bottom:16px; }
.card { background:var(--card); border:1px solid var(--border); border-radius:8px; padding:16px; margin-bottom:16px; }
.card h2 { font-size:0.85rem; color:var(--muted); text-transform:uppercase; margin-bottom:12px; }
.btn { background:var(--accent); color:#fff; border:none; border-radius:4px; padding:12px 20px; font-weight:600; cursor:pointer; margin:4px; min-width:160px; }
.btn:hover { filter:brightness(1.1); }
.btn:disabled { opacity:0.5; cursor:not-allowed; }
.btn.ok { background:#3fb950; }
.btn.warn { background:#d29922; }
.btn.err { background:#f85149; }
.status { font-family:ui-monospace,monospace; padding:8px; background:var(--bg); border-radius:4px; margin-top:8px; white-space:pre; }
.grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(180px,1fr)); gap:8px; }
</style>
</head>
<body>
<h1>🔧 Bench Test Mode</h1>
<div class="warning">⚠️ <strong>BENCH TEST MODE ACTIVE</strong> — Launch detection disabled. Servo movement allowed. HORN MUST BE REMOVED.</div>

<div class="card">
<h2>Servo Control (Safe Limits)</h2>
<div class="grid">
<button class="btn ok" onclick="servoCmd('neutral')">⏹ Neutral</button>
<button class="btn" onclick="servoCmd('servo1_40')">Servo 1 40%</button>
<button class="btn" onclick="servoCmd('servo1_80')">Servo 1 80%</button>
<button class="btn" onclick="servoCmd('servo2_40')">Servo 2 40%</button>
<button class="btn" onclick="servoCmd('servo2_80')">Servo 2 80%</button>
<button class="btn warn" onclick="servoCmd('brake40')">⏬ Brake 40%</button>
</div>
<div id="servoStatus" class="status">Ready</div>
<p style="color:var(--muted);margin-top:8px;font-size:.9rem">
Drag proxy is not real drag force. It is a brake-effect measurement from IMU/barometer/GPS changes after the servo moves.
Real drag in newtons requires a load cell, airspeed sensor, or calibrated wind-tunnel-style setup.
</p>
</div>

<div class="card">
<h2>Sensor Tests</h2>
<div class="grid">
<button class="btn" onclick="testSensor('imu')">📐 IMU Test</button>
<button class="btn" onclick="testSensor('baro')">📊 Barometer Test</button>
<button class="btn" onclick="testSensor('gps')">🛰 GPS Test</button>
<button class="btn" onclick="testSensor('lora')">📡 LoRa Test</button>
</div>
<div id="sensorStatus" class="status">Select a test</div>
</div>

<div class="card">
<h2>System</h2>
<button class="btn" onclick="getStatus()">📋 Full Status</button>
<button class="btn err" onclick="reboot()">🔄 Reboot</button>
<div id="sysStatus" class="status"></div>
</div>

<a href="/" style="color:var(--accent); text-decoration:none;">← Back to Dashboard</a>

<script>
async function servoCmd(cmd) {
    document.getElementById('servoStatus').textContent = 'Sending '+cmd+'...';
    const r = await fetch('/api/servo/test?command='+encodeURIComponent(cmd), {method:'POST'});
    const j = await r.json();
    if (!r.ok || cmd === 'neutral') {
        document.getElementById('servoStatus').textContent = JSON.stringify(j, null, 2);
        return;
    }
    document.getElementById('servoStatus').textContent = 'Command sent. Measuring IMU response…';
    const started = Date.now();
    const pollResponse = async () => {
        const result = await fetch('/api/servo/response').then(x => x.json());
        if (result.status === 'detected' || result.status === 'not_detected') {
            document.getElementById('servoStatus').textContent =
                'Command: '+result.command+'\n'+
                'Status: '+result.status+'\n'+
                'IMU response delay: '+(result.delay_ms || 0)+' ms\n'+
                'Rocket/payload turn: '+result.rocket_turn_deg.toFixed(1)+'° yaw\n'+
                'Peak rocket/payload turn: '+result.peak_rocket_turn_deg.toFixed(1)+'° yaw\n'+
                'Yaw start → now: '+result.baseline_yaw_deg.toFixed(1)+'° → '+result.yaw_deg.toFixed(1)+'°\n'+
                'Peak rotation-rate change: '+result.peak_rate_delta_dps.toFixed(1)+' °/s\n'+
                'Max ground-speed loss: '+result.max_ground_speed_loss_mps.toFixed(2)+' m/s '+(result.gps_valid?'':'(GPS unavailable/limited)')+'\n'+
                'Max sink-rate increase: '+result.max_sink_increase_mps.toFixed(2)+' m/s '+(result.barometer_valid?'':'(barometer unavailable/limited)')+'\n'+
                'Peak vertical-accel change: '+result.peak_vertical_accel_delta_mps2.toFixed(2)+' m/s² '+(result.imu_valid?'':'(IMU unavailable/limited)')+'\n'+
                'Brake drag proxy: '+result.drag_proxy.toFixed(2)+' unitless\n'+
                'Samples: '+result.samples+'\n'+
                'Note: '+result.note;
        } else if (result.status === 'measuring' && Date.now()-started < 1700) {
            setTimeout(pollResponse, 50);
        } else if (result.status === 'imu_unavailable') {
            document.getElementById('servoStatus').textContent =
                'IMU unavailable, so rotation/acceleration drag proxy is limited.\n'+
                'If GPS/barometer are working during a moving/drop test, speed/sink metrics may still help.\n'+
                JSON.stringify(result, null, 2);
        } else {
            document.getElementById('servoStatus').textContent =
                'No payload rotation detected within 1.4 s.\n'+
                'Suspend the inert module or run a controlled drop/glide test so drag/brake response can be observed.\n'+
                JSON.stringify(result, null, 2);
        }
    };
    setTimeout(pollResponse, 50);
}
async function testSensor(sensor) {
    document.getElementById('sensorStatus').textContent = 'Testing '+sensor+'...';
    try {
        if (sensor === 'lora') {
            const r = await fetch('/api/status');
            const j = await r.json();
            document.getElementById('sensorStatus').textContent = JSON.stringify({
                rssi_dbm: j.lora_rssi,
                snr_db: j.lora_snr
            }, null, 2);
            return;
        }
        const r = await fetch('/api/sensors');
        const j = await r.json();
        const result = j[sensor];
        document.getElementById('sensorStatus').textContent = result
            ? JSON.stringify(result, null, 2)
            : 'No '+sensor+' result returned by the firmware';
    } catch (e) {
        document.getElementById('sensorStatus').textContent = 'Sensor API error: '+e.message;
    }
}
async function getStatus() {
    const r = await fetch('/api/status');
    const j = await r.json();
    document.getElementById('sysStatus').textContent = JSON.stringify(j, null, 2);
}
async function reboot() { if(confirm('Reboot?')) { await fetch('/api/reboot',{method:'POST'}); setTimeout(()=>location.reload(),2000); } }
</script>
</body>
</html>
)HTML";
}

} // namespace comms
