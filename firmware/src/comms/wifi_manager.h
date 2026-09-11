// ============================================================================
// PHOENIX RECOVERY — WiFi Manager (Access Point + Web Server).
// ============================================================================
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include "config.h"
#include "vehicle_state.h"
#include "control/servo_controller.h"

namespace comms {

class WiFiManager {
public:
    WiFiManager();
    ~WiFiManager();

    // Initialize WiFi AP and web server
    bool begin(phoenix::VehicleState* state, control::ServoController* servos);

    // Update - call periodically to handle clients
    void update();

    // Check if WiFi is running
    bool isRunning() const { return ap_started_; }
    uint32_t getRestartAttempts() const { return restart_attempts_; }
    uint32_t getSuccessfulStarts() const { return successful_starts_; }
    uint32_t getLastStartMs() const { return last_start_ms_; }

    // Get connected client count
    int getClientCount() const { return WiFi.softAPgetStationNum(); }

    // Get AP IP address
    IPAddress getAPIP() const { return WiFi.softAPIP(); }

    // True while a short, server-authorized ground-test command owns the
    // servos. Commands expire automatically and return both servos to neutral.
    bool isServoTestActive() const;

private:
    phoenix::VehicleState* state_ = nullptr;
    control::ServoController* servos_ = nullptr;
    AsyncWebServer* server_ = nullptr;
    AsyncEventSource* events_ = nullptr;
    DNSServer dns_server_;
    bool dns_started_ = false;

    bool ap_started_ = false;
    uint32_t last_push_ms_ = 0;
    uint32_t last_ap_check_ms_ = 0;
    uint32_t push_interval_ms_ = 1000 / cfg::DASHBOARD_PUSH_RATE_HZ;
    uint32_t servo_test_expires_ms_ = 0;
    bool response_measurement_active_ = false;
    bool response_measurement_detected_ = false;
    uint32_t response_command_ms_ = 0;
    uint32_t response_delay_ms_ = 0;
    float response_baseline_rate_dps_ = 0.0f;
    float response_baseline_yaw_deg_ = 0.0f;
    float response_yaw_delta_deg_ = 0.0f;
    float response_peak_turn_deg_ = 0.0f;
    float response_baseline_vertical_speed_mps_ = 0.0f;
    float response_baseline_ground_speed_mps_ = 0.0f;
    float response_baseline_vertical_accel_mps2_ = 0.0f;
    float response_peak_rate_delta_dps_ = 0.0f;
    float response_peak_vertical_accel_delta_mps2_ = 0.0f;
    float response_max_sink_increase_mps_ = 0.0f;
    float response_max_ground_speed_loss_mps_ = 0.0f;
    float response_drag_proxy_ = 0.0f;
    uint16_t response_samples_ = 0;
    String response_command_;
    uint32_t restart_attempts_ = 0;
    uint32_t successful_starts_ = 0;
    uint32_t last_start_ms_ = 0;
    uint32_t degraded_health_checks_ = 0;

    bool setupAP();
    bool startAPWithRetries(uint8_t attempts, uint32_t retry_delay_ms);
    bool apHealthy() const;
    void markAPRunning(bool running);
    void recoverAPIfNeeded(uint32_t now_ms);
    void setupRoutes();
    void setupCaptivePortal();
    void pushEvents();
    void handleConfigGet(AsyncWebServerRequest* request);
    void handleConfigPost(AsyncWebServerRequest* request);
    void handleServoTest(AsyncWebServerRequest* request);
    void handleServoResponse(AsyncWebServerRequest* request);
    void handleReboot(AsyncWebServerRequest* request);
    String generateDashboardHTML();
    String generateConfigHTML();
    String generateBenchHTML();
    String generateSetupHTML();
    String getReadinessJSON();
    String getStatusJSON();
    String getSensorsJSON();
    String getNavigationJSON();
    String getConfigJSON();
};

} // namespace comms
