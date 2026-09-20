// ============================================================================
// PHOENIX RECOVERY — LoRa Ground Control Station.
// ============================================================================
// Flash this to a second Heltec WiFi LoRa 32 V4/V4.3 board. This board stays
// with the computer. The computer connects to this board's Wi-Fi dashboard, and
// the board sends supervised command packets to the rocket over LoRa.
//
// Safety boundary:
//   - The rocket remains the final authority.
//   - The rocket ignores manual brake commands during powered ascent,
//     deployment wait, failsafe, landed, and other unsafe states.
//   - Commands expire quickly if this ground station stops transmitting.
// ============================================================================

#include <Arduino.h>
#include <DNSServer.h>
#include <RadioLib.h>
#include <WebServer.h>
#include <WiFi.h>
#include <cmath>

#include "config.h"
#include "comms/lora_remote_protocol.h"
#include "telemetry/telemetry_packet.h"
#include "logic/state_machine.h"

namespace {

constexpr const char* GROUND_AP_SSID = "PHOENIX-GROUND";
constexpr const char* GROUND_AP_PASSWORD = "phoenixground";
constexpr uint8_t GROUND_AP_CHANNEL = 6;
constexpr uint8_t GROUND_AP_MAX_CLIENTS = 3;
constexpr uint16_t GROUND_HTTP_PORT = 80;
constexpr uint32_t TELEMETRY_STALE_MS = 2500;
constexpr uint32_t COMMAND_REPEAT_INTERVAL_MS = 250;
constexpr uint32_t MANUAL_HOLD_MS = 1500;
constexpr float GROUND_MAX_MANUAL_BRAKE = cfg::LORA_REMOTE_MAX_BRAKE_COMMAND;

static Module lora_module(
    cfg::PIN_LORA_NSS,
    cfg::PIN_LORA_DIO1,
    cfg::PIN_LORA_RST,
    cfg::PIN_LORA_BUSY,
    SPI
);
static SX1262 radio(&lora_module);
static WebServer server(GROUND_HTTP_PORT);
static DNSServer dns;

static bool radio_ok = false;
static bool wifi_ok = false;
static bool dns_ok = false;
static uint16_t sequence_number = 0;
static String input_line;
static String last_command = "none";
static String last_error = "";
static uint32_t sent_count = 0;
static uint32_t failed_count = 0;
static uint32_t last_send_ms = 0;
static uint32_t telemetry_received_count = 0;
static uint32_t telemetry_invalid_count = 0;
static int16_t last_command_rssi = 0;
static float last_command_snr = 0.0f;
static volatile bool lora_packet_received = false;

void IRAM_ATTR onLoRaPacketReceived() {
    lora_packet_received = true;
}

struct TelemetrySnapshot {
    bool valid = false;
    uint32_t rx_ms = 0;
    uint32_t payload_time_ms = 0;
    uint8_t sequence = 0;
    logic::FlightState flight_state = logic::FlightState::BOOT;
    logic::GuidanceMode guidance_mode = logic::GuidanceMode::MODE_DISABLED;
    logic::FailCode failsafe_code = logic::FailCode::FAIL_NONE;
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude_agl_m = 0.0f;
    float vertical_speed_mps = 0.0f;
    float ground_speed_mps = 0.0f;
    float gps_course_deg = 0.0f;
    float target_bearing_deg = 0.0f;
    float heading_error_deg = 0.0f;
    float distance_to_target_m = 0.0f;
    float servo1_cmd = 0.0f;
    float servo2_cmd = 0.0f;
    int16_t servo1_us = 1500;
    int16_t servo2_us = 1500;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float angular_rate_dps = 0.0f;
    uint8_t satellites = 0;
    bool gps_valid = false;
    bool imu_valid = false;
    bool baro_valid = false;
    phoenix::DropTestState drop_test_state = phoenix::DropTestState::IDLE;
    bool drop_test_recording = false;
    bool drop_test_neutral_lock = false;
    uint16_t drop_test_id = 0;
    uint32_t drop_test_armed_ms = 0;
    uint32_t drop_test_release_ms = 0;
    uint32_t drop_test_landing_ms = 0;
    int16_t rssi = 0;
    float snr = 0.0f;
};

static TelemetrySnapshot telemetry;

struct ManualHold {
    bool active = false;
    bool bench_mode = false;
    float servo1 = 0.0f;
    float servo2 = 0.0f;
    uint32_t until_ms = 0;
    uint32_t last_repeat_ms = 0;
};

static ManualHold manual_hold;

uint16_t crc16CcittLocal(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool validTelemetryPacket(const telemetry::TelemetryPacketV1& packet) {
    if (packet.magic[0] != 0x50 || packet.magic[1] != 0x52) return false;
    if (packet.version != 2) return false;
    const uint16_t expected = crc16CcittLocal(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(telemetry::TelemetryPacketV1) - sizeof(packet.crc)
    );
    return packet.crc == expected;
}

int16_t fractionToMilli(float value) {
    value = constrain(value, -1.0f, 1.0f);
    return static_cast<int16_t>(lroundf(value * 1000.0f));
}

String escapeJson(const String& input) {
    String out;
    out.reserve(input.length() + 8);
    for (size_t i = 0; i < input.length(); ++i) {
        const char c = input[i];
        if (c == '"' || c == '\\') out += '\\';
        if (c == '\n' || c == '\r') continue;
        out += c;
    }
    return out;
}

bool sendCommand(comms::LoRaRemoteCommandType type,
                 float servo1 = 0.0f,
                 float servo2 = 0.0f,
                 double target_lat = 0.0,
                 double target_lon = 0.0,
                 bool quiet = false) {
    if (!radio_ok) {
        last_error = "LoRa radio is not ready";
        failed_count++;
        if (!quiet) Serial.println(last_error);
        return false;
    }

    comms::LoRaRemoteCommandPacket packet;
    packet.type = static_cast<uint8_t>(type);
    packet.sequence = ++sequence_number;
    packet.servo1_command_milli = fractionToMilli(servo1);
    packet.servo2_command_milli = fractionToMilli(servo2);
    packet.target_lat_e7 = static_cast<int32_t>(llround(target_lat * 10000000.0));
    packet.target_lon_e7 = static_cast<int32_t>(llround(target_lon * 10000000.0));
    comms::finalizeLoRaRemotePacket(packet);

    lora_packet_received = false;
    const int16_t state = radio.transmit(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(packet)
    );
    radio.startReceive();

    if (state == RADIOLIB_ERR_NONE) {
        last_send_ms = millis();
        sent_count++;
        last_error = "";
        last_command_rssi = radio.getRSSI();
        last_command_snr = radio.getSNR();
        last_command = String(comms::loraRemoteCommandName(type));
        if (type == comms::LoRaRemoteCommandType::MANUAL_BRAKE) {
            last_command += " servo1=" + String(servo1, 2) + " servo2=" + String(servo2, 2);
        } else if (type == comms::LoRaRemoteCommandType::SET_TARGET) {
            last_command += " lat=" + String(target_lat, 7) + " lon=" + String(target_lon, 7);
        }
        if (!quiet) {
            Serial.printf("sent seq=%u type=%s\n",
                          packet.sequence,
                          comms::loraRemoteCommandName(type));
        }
        return true;
    }

    failed_count++;
    last_error = "LoRa send failed: RadioLib code " + String(state);
    if (!quiet) Serial.println(last_error);
    radio.startReceive();
    return false;
}

void pollTelemetry() {
    if (!radio_ok || !lora_packet_received) return;
    lora_packet_received = false;
    const uint32_t now = millis();

    telemetry::TelemetryPacketV1 packet;
    const int16_t state = radio.readData(
        reinterpret_cast<uint8_t*>(&packet),
        sizeof(packet)
    );
    radio.startReceive();
    if (state != RADIOLIB_ERR_NONE || !validTelemetryPacket(packet)) {
        telemetry_invalid_count++;
        return;
    }

    telemetry_received_count++;
    telemetry.valid = true;
    telemetry.rx_ms = now;
    telemetry.payload_time_ms = packet.timestamp_ms;
    telemetry.sequence = packet.sequence;
    telemetry.flight_state = static_cast<logic::FlightState>(packet.flight_state);
    telemetry.guidance_mode = static_cast<logic::GuidanceMode>(packet.guidance_mode);
    telemetry.failsafe_code = static_cast<logic::FailCode>(packet.failsafe_code);
    telemetry.latitude = static_cast<double>(packet.latitude_deg7) / 10000000.0;
    telemetry.longitude = static_cast<double>(packet.longitude_deg7) / 10000000.0;
    telemetry.altitude_agl_m = static_cast<float>(packet.altitude_agl_m) / 10.0f;
    telemetry.vertical_speed_mps = static_cast<float>(packet.vertical_speed_mps) / 100.0f;
    telemetry.ground_speed_mps = static_cast<float>(packet.ground_speed_mps) / 100.0f;
    telemetry.gps_course_deg = static_cast<float>(packet.gps_course_deg) / 100.0f;
    telemetry.target_bearing_deg = static_cast<float>(packet.target_bearing_deg) / 100.0f;
    telemetry.heading_error_deg = static_cast<float>(packet.heading_error_deg) / 100.0f;
    telemetry.distance_to_target_m = static_cast<float>(packet.distance_to_target_m);
    telemetry.servo1_cmd = static_cast<float>(packet.left_servo_cmd) / 10000.0f;
    telemetry.servo2_cmd = static_cast<float>(packet.right_servo_cmd) / 10000.0f;
    telemetry.servo1_us = packet.left_servo_us;
    telemetry.servo2_us = packet.right_servo_us;
    telemetry.roll_deg = static_cast<float>(packet.roll_deg) / 100.0f;
    telemetry.pitch_deg = static_cast<float>(packet.pitch_deg) / 100.0f;
    telemetry.yaw_deg = static_cast<float>(packet.yaw_deg) / 100.0f;
    telemetry.angular_rate_dps = static_cast<float>(packet.angular_rate_dps) / 10.0f;
    telemetry.satellites = packet.satellites;
    telemetry.gps_valid = packet.gps_valid != 0 || packet.gps_fix_valid != 0;
    telemetry.imu_valid = packet.imu_valid != 0;
    telemetry.baro_valid = packet.baro_valid != 0;
    telemetry.drop_test_state = static_cast<phoenix::DropTestState>(packet.drop_test_state);
    telemetry.drop_test_recording = (packet.drop_test_flags & 1U) != 0;
    telemetry.drop_test_neutral_lock = (packet.drop_test_flags & 2U) != 0;
    telemetry.drop_test_id = packet.drop_test_id;
    telemetry.drop_test_armed_ms = packet.drop_test_armed_ms;
    telemetry.drop_test_release_ms = packet.drop_test_release_ms;
    telemetry.drop_test_landing_ms = packet.drop_test_landing_ms;
    telemetry.rssi = radio.getRSSI();
    telemetry.snr = radio.getSNR();
}

void updateManualHold() {
    if (!manual_hold.active) return;
    if (telemetry.drop_test_recording) {
        manual_hold.active = false;
        manual_hold.bench_mode = false;
        last_error = "Manual repeats stopped while drop recording is active";
        return;
    }
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - manual_hold.until_ms) >= 0) {
        manual_hold.active = false;
        sendCommand(comms::LoRaRemoteCommandType::NEUTRAL, 0.0f, 0.0f, 0.0, 0.0, true);
        return;
    }
    if (now - manual_hold.last_repeat_ms >= COMMAND_REPEAT_INTERVAL_MS) {
        manual_hold.last_repeat_ms = now;
        sendCommand(manual_hold.bench_mode ? comms::LoRaRemoteCommandType::BENCH_SERVO
                                           : comms::LoRaRemoteCommandType::MANUAL_BRAKE,
                    manual_hold.servo1,
                    manual_hold.servo2,
                    0.0,
                    0.0,
                    true);
    }
}

void printHelp() {
    Serial.println();
    Serial.println("PHOENIX LoRa ground-control commands:");
    Serial.println("  help");
    Serial.println("  ping");
    Serial.println("  status                      shows payload LoRa telemetry link");
    Serial.println("  armdrop                     arm payload-owned inert drop recording");
    Serial.println("  abortdrop                   abort drop recording and command neutral");
    Serial.println("  bench <1|2>                 small timed PAD_SAFE servo test");
    Serial.println("  neutral");
    Serial.println("  servo <servo1> <servo2>     values -1.00..+1.00; rocket clamps again");
    Serial.println("  target <latitude> <longitude>");
    Serial.println("  disable                     disables rocket LoRa remote until reboot");
    Serial.println();
    Serial.println("Web dashboard:");
    Serial.println("  Wi-Fi: PHOENIX-GROUND");
    Serial.println("  Password: phoenixground");
    Serial.println("  Open: http://192.168.8.1/");
    Serial.println();
    Serial.println("Safety: rocket ignores manual servo commands unless it is in GUIDED_DESCENT or FINAL_APPROACH.");
}

void printLinkStatus() {
    const uint32_t now = millis();
    const uint32_t age = telemetry.rx_ms == 0 ? UINT32_MAX : now - telemetry.rx_ms;
    const bool fresh = telemetry.valid && age <= TELEMETRY_STALE_MS;
    Serial.printf("LoRa radio=%s payload_link=%s valid_packets=%lu invalid_packets=%lu",
                  radio_ok ? "READY" : "FAILED",
                  fresh ? "CONNECTED" : "NO TELEMETRY",
                  (unsigned long)telemetry_received_count,
                  (unsigned long)telemetry_invalid_count);
    if (telemetry.valid) {
        Serial.printf(" age=%lums seq=%u RSSI=%ddBm SNR=%.1fdB state=%s GPS=%s IMU=%s BARO=%s\n",
                      (unsigned long)age, telemetry.sequence, telemetry.rssi, telemetry.snr,
                      logic::flightStateName(telemetry.flight_state),
                      telemetry.gps_valid ? "OK" : "NO_FIX",
                      telemetry.imu_valid ? "OK" : "BAD",
                      telemetry.baro_valid ? "OK" : "BAD");
    } else {
        Serial.println();
    }
}

bool parseTwoFloats(const String& line, const char* command,
                    float& first, float& second) {
    return sscanf(line.c_str(), "%*s %f %f", &first, &second) == 2 &&
           line.startsWith(command);
}

bool parseTarget(const String& line, double& lat, double& lon) {
    return sscanf(line.c_str(), "%*s %lf %lf", &lat, &lon) == 2 &&
           line.startsWith("target");
}

void handleLine(String line) {
    line.trim();
    line.toLowerCase();
    if (line.length() == 0) return;

    if (line == "help" || line == "?") {
        printHelp();
    } else if (line == "status") {
        printLinkStatus();
    } else if (line == "bench 1") {
        if (telemetry.drop_test_recording) {
            Serial.println("bench test locked while drop recording is active");
            return;
        }
        manual_hold.active = true;
        manual_hold.bench_mode = true;
        manual_hold.servo1 = 0.08f;
        manual_hold.servo2 = 0.0f;
        manual_hold.until_ms = millis() + 1000;
        manual_hold.last_repeat_ms = millis();
        sendCommand(comms::LoRaRemoteCommandType::BENCH_SERVO, 0.08f, 0.0f);
    } else if (line == "bench 2") {
        if (telemetry.drop_test_recording) {
            Serial.println("bench test locked while drop recording is active");
            return;
        }
        manual_hold.active = true;
        manual_hold.bench_mode = true;
        manual_hold.servo1 = 0.0f;
        manual_hold.servo2 = 0.08f;
        manual_hold.until_ms = millis() + 1000;
        manual_hold.last_repeat_ms = millis();
        sendCommand(comms::LoRaRemoteCommandType::BENCH_SERVO, 0.0f, 0.08f);
    } else if (line == "ping") {
        sendCommand(comms::LoRaRemoteCommandType::PING);
    } else if (line == "armdrop") {
        sendCommand(comms::LoRaRemoteCommandType::ARM_DROP_TEST);
    } else if (line == "abortdrop") {
        manual_hold.active = false;
        manual_hold.bench_mode = false;
        sendCommand(comms::LoRaRemoteCommandType::ABORT_DROP_TEST);
    } else if (line == "neutral") {
        manual_hold.active = false;
        manual_hold.bench_mode = false;
        sendCommand(comms::LoRaRemoteCommandType::NEUTRAL);
    } else if (line == "disable") {
        manual_hold.active = false;
        manual_hold.bench_mode = false;
        sendCommand(comms::LoRaRemoteCommandType::DISABLE_REMOTE);
    } else if (line.startsWith("servo")) {
        if (telemetry.drop_test_recording) {
            Serial.println("manual steering locked while drop recording is active");
            return;
        }
        float servo1 = 0.0f;
        float servo2 = 0.0f;
        if (!parseTwoFloats(line, "servo", servo1, servo2)) {
            Serial.println("bad servo command. Example: servo 0.20 0.00");
            return;
        }
        servo1 = constrain(servo1, -GROUND_MAX_MANUAL_BRAKE, GROUND_MAX_MANUAL_BRAKE);
        servo2 = constrain(servo2, -GROUND_MAX_MANUAL_BRAKE, GROUND_MAX_MANUAL_BRAKE);
        manual_hold.active = true;
        manual_hold.bench_mode = false;
        manual_hold.servo1 = servo1;
        manual_hold.servo2 = servo2;
        manual_hold.until_ms = millis() + MANUAL_HOLD_MS;
        manual_hold.last_repeat_ms = 0;
        sendCommand(comms::LoRaRemoteCommandType::MANUAL_BRAKE, servo1, servo2);
    } else if (line.startsWith("target")) {
        if (telemetry.drop_test_recording) {
            Serial.println("target update locked while drop recording is active");
            return;
        }
        double lat = 0.0;
        double lon = 0.0;
        if (!parseTarget(line, lat, lon) ||
            lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 ||
            (fabs(lat) < 1e-9 && fabs(lon) < 1e-9)) {
            Serial.println("bad target. Example: target 37.1234567 -122.1234567");
            return;
        }
        sendCommand(comms::LoRaRemoteCommandType::SET_TARGET, 0.0f, 0.0f, lat, lon);
    } else {
        Serial.println("unknown command. Type: help");
    }
}

String statusJson() {
    const uint32_t now = millis();
    const uint32_t telemetry_age = telemetry.rx_ms == 0 ? UINT32_MAX : now - telemetry.rx_ms;
    const bool telemetry_fresh = telemetry.valid && telemetry_age <= TELEMETRY_STALE_MS;
    char buf[4608];
    snprintf(buf, sizeof(buf),
             "{"
             "\"wifi_ok\":%s,"
             "\"radio_ok\":%s,"
             "\"ip\":\"%s\","
             "\"clients\":%d,"
             "\"sent_count\":%lu,"
             "\"failed_count\":%lu,"
             "\"sequence\":%u,"
             "\"last_command\":\"%s\","
             "\"last_error\":\"%s\","
             "\"last_send_age_ms\":%lu,"
             "\"manual_hold_active\":%s,"
             "\"manual_servo1\":%.3f,"
             "\"manual_servo2\":%.3f,"
             "\"manual_max\":%.3f,"
             "\"command_rssi\":%d,"
             "\"command_snr\":%.1f,"
             "\"telemetry\":{"
                "\"valid\":%s,"
                "\"fresh\":%s,"
                "\"age_ms\":%lu,"
                "\"payload_time_ms\":%lu,"
                "\"sequence\":%u,"
                "\"rssi\":%d,"
                "\"snr\":%.1f,"
                "\"flight_state\":\"%s\","
                "\"guidance_mode\":\"%s\","
                "\"failsafe\":\"%s\","
                "\"lat\":%.7f,"
                "\"lon\":%.7f,"
                "\"altitude_agl_m\":%.1f,"
                "\"vertical_speed_mps\":%.2f,"
                "\"ground_speed_mps\":%.2f,"
                "\"gps_course_deg\":%.1f,"
                "\"target_bearing_deg\":%.1f,"
                "\"heading_error_deg\":%.1f,"
                "\"distance_to_target_m\":%.1f,"
                "\"servo1_cmd\":%.3f,"
                "\"servo2_cmd\":%.3f,"
                "\"servo1_us\":%d,"
                "\"servo2_us\":%d,"
                "\"roll_deg\":%.1f,"
                "\"pitch_deg\":%.1f,"
                "\"yaw_deg\":%.1f,"
                "\"angular_rate_dps\":%.1f,"
                "\"satellites\":%u,"
                "\"gps_valid\":%s,"
                "\"imu_valid\":%s,"
                "\"baro_valid\":%s,"
                "\"drop_test_state\":\"%s\","
                "\"drop_test_recording\":%s,"
                "\"drop_test_neutral_lock\":%s,"
                "\"drop_test_id\":%u,"
                "\"drop_test_armed_ms\":%lu,"
                "\"drop_test_release_ms\":%lu,"
                "\"drop_test_landing_ms\":%lu"
             "}"
             "}",
             wifi_ok ? "true" : "false",
             radio_ok ? "true" : "false",
             WiFi.softAPIP().toString().c_str(),
             WiFi.softAPgetStationNum(),
             static_cast<unsigned long>(sent_count),
             static_cast<unsigned long>(failed_count),
             sequence_number,
             escapeJson(last_command).c_str(),
             escapeJson(last_error).c_str(),
             last_send_ms == 0 ? static_cast<unsigned long>(UINT32_MAX) : static_cast<unsigned long>(now - last_send_ms),
             manual_hold.active ? "true" : "false",
             manual_hold.servo1,
             manual_hold.servo2,
             GROUND_MAX_MANUAL_BRAKE,
             last_command_rssi,
             last_command_snr,
             telemetry.valid ? "true" : "false",
             telemetry_fresh ? "true" : "false",
             static_cast<unsigned long>(telemetry_age),
             static_cast<unsigned long>(telemetry.payload_time_ms),
             telemetry.sequence,
             telemetry.rssi,
             telemetry.snr,
             logic::flightStateName(telemetry.flight_state),
             logic::guidanceModeName(telemetry.guidance_mode),
             logic::failCodeName(telemetry.failsafe_code),
             telemetry.latitude,
             telemetry.longitude,
             telemetry.altitude_agl_m,
             telemetry.vertical_speed_mps,
             telemetry.ground_speed_mps,
             telemetry.gps_course_deg,
             telemetry.target_bearing_deg,
             telemetry.heading_error_deg,
             telemetry.distance_to_target_m,
             telemetry.servo1_cmd,
             telemetry.servo2_cmd,
             telemetry.servo1_us,
             telemetry.servo2_us,
             telemetry.roll_deg,
             telemetry.pitch_deg,
             telemetry.yaw_deg,
             telemetry.angular_rate_dps,
             telemetry.satellites,
             telemetry.gps_valid ? "true" : "false",
             telemetry.imu_valid ? "true" : "false",
             telemetry.baro_valid ? "true" : "false",
             phoenix::dropTestStateName(telemetry.drop_test_state),
             telemetry.drop_test_recording ? "true" : "false",
             telemetry.drop_test_neutral_lock ? "true" : "false",
             telemetry.drop_test_id,
             static_cast<unsigned long>(telemetry.drop_test_armed_ms),
             static_cast<unsigned long>(telemetry.drop_test_release_ms),
             static_cast<unsigned long>(telemetry.drop_test_landing_ms));
    return String(buf);
}

void sendJson(int code, const String& body) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(code, "application/json", body);
}

void handleCommandApi() {
    const String type = server.hasArg("type") ? server.arg("type") : "";
    bool ok = false;

    if (type == "ping") {
        ok = sendCommand(comms::LoRaRemoteCommandType::PING);
    } else if (type == "armdrop") {
        manual_hold.active = false;
        manual_hold.bench_mode = false;
        ok = sendCommand(comms::LoRaRemoteCommandType::ARM_DROP_TEST);
    } else if (type == "abortdrop") {
        manual_hold.active = false;
        manual_hold.bench_mode = false;
        ok = sendCommand(comms::LoRaRemoteCommandType::ABORT_DROP_TEST);
    } else if (type == "logindex") {
        ok = sendCommand(comms::LoRaRemoteCommandType::REQUEST_LOG_INDEX);
    } else if (type == "neutral") {
        manual_hold.active = false;
        ok = sendCommand(comms::LoRaRemoteCommandType::NEUTRAL);
    } else if (type == "bench") {
        if (telemetry.drop_test_recording) {
            sendJson(409, "{\"ok\":false,\"error\":\"Bench test locked while drop recording is active\"}");
            return;
        }
        const int servo_number = server.hasArg("servo") ? server.arg("servo").toInt() : 0;
        if (servo_number != 1 && servo_number != 2) {
            sendJson(400, "{\"ok\":false,\"error\":\"Choose Servo 1 or Servo 2\"}");
            return;
        }
        const float amount = cfg::LORA_BENCH_MAX_SERVO_COMMAND;
        manual_hold.active = true;
        manual_hold.bench_mode = true;
        manual_hold.servo1 = servo_number == 1 ? amount : 0.0f;
        manual_hold.servo2 = servo_number == 2 ? amount : 0.0f;
        manual_hold.until_ms = millis() + 1000;
        manual_hold.last_repeat_ms = millis();
        ok = sendCommand(comms::LoRaRemoteCommandType::BENCH_SERVO,
                         manual_hold.servo1, manual_hold.servo2);
    } else if (type == "disable") {
        manual_hold.active = false;
        ok = sendCommand(comms::LoRaRemoteCommandType::DISABLE_REMOTE);
    } else if (type == "servo") {
        if (telemetry.drop_test_recording) {
            sendJson(409, "{\"ok\":false,\"error\":\"Manual steering locked while drop recording is active\"}");
            return;
        }
        float servo1 = server.hasArg("servo1") ? server.arg("servo1").toFloat() : 0.0f;
        float servo2 = server.hasArg("servo2") ? server.arg("servo2").toFloat() : 0.0f;
        servo1 = constrain(servo1, -GROUND_MAX_MANUAL_BRAKE, GROUND_MAX_MANUAL_BRAKE);
        servo2 = constrain(servo2, -GROUND_MAX_MANUAL_BRAKE, GROUND_MAX_MANUAL_BRAKE);
        manual_hold.active = true;
        manual_hold.servo1 = servo1;
        manual_hold.servo2 = servo2;
        manual_hold.until_ms = millis() + MANUAL_HOLD_MS;
        manual_hold.last_repeat_ms = 0;
        ok = sendCommand(comms::LoRaRemoteCommandType::MANUAL_BRAKE, servo1, servo2);
    } else if (type == "target") {
        if (telemetry.drop_test_recording) {
            sendJson(409, "{\"ok\":false,\"error\":\"Target update locked while drop recording is active\"}");
            return;
        }
        const double lat = server.hasArg("lat") ? server.arg("lat").toDouble() : 0.0;
        const double lon = server.hasArg("lon") ? server.arg("lon").toDouble() : 0.0;
        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 ||
            (fabs(lat) < 1e-9 && fabs(lon) < 1e-9)) {
            sendJson(400, "{\"ok\":false,\"error\":\"Invalid target coordinates\"}");
            return;
        }
        ok = sendCommand(comms::LoRaRemoteCommandType::SET_TARGET, 0.0f, 0.0f, lat, lon);
    } else {
        sendJson(400, "{\"ok\":false,\"error\":\"Unknown command type\"}");
        return;
    }

    sendJson(ok ? 200 : 503, String("{\"ok\":") + (ok ? "true" : "false") +
             ",\"status\":" + statusJson() + "}");
}

String htmlPage() {
    return R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#07100f">
<title>PHOENIX Ground Station</title>
<style>
:root{--bg:#07100f;--panel:#0c1817;--panel2:#101f1d;--line:#203a36;--text:#eef8f4;--muted:#91aaa4;--orange:#ff8b3d;--cyan:#36d4bd;--ok:#55dd91;--warn:#ffc857;--bad:#ff6b6b;--shadow:0 18px 50px #0007}
*{box-sizing:border-box}html{background:var(--bg)}body{margin:0;color:var(--text);font:15px/1.45 Inter,ui-sans-serif,system-ui,-apple-system,Segoe UI,sans-serif;background:radial-gradient(circle at 80% -10%,#173c35 0,transparent 32rem),linear-gradient(180deg,#091412,#050a09);min-height:100vh}
button,input{font:inherit}.shell{width:min(1400px,100%);margin:auto;padding:18px}.top{display:flex;align-items:center;justify-content:space-between;gap:18px;padding:4px 0 18px}.brand{display:flex;align-items:center;gap:12px}.mark{width:42px;height:42px;display:grid;place-items:center;border:1px solid #ff8b3d77;background:#ff8b3d16;border-radius:12px;color:var(--orange);font-size:22px}.eyebrow{color:var(--orange);font:800 10px/1 ui-monospace,monospace;letter-spacing:.18em}.brand h1{font-size:20px;line-height:1.1;margin:5px 0 0}.topright{display:flex;align-items:center;gap:10px}.pill{border:1px solid var(--line);border-radius:999px;padding:7px 11px;font:800 11px ui-monospace,monospace;letter-spacing:.06em;background:#0a1514}.dot{display:inline-block;width:7px;height:7px;border-radius:50%;background:currentColor;margin-right:7px;box-shadow:0 0 10px currentColor}.live{color:var(--ok)}.stale{color:var(--warn)}.offline{color:var(--bad)}
.mission{display:grid;grid-template-columns:1.4fr repeat(3,1fr);gap:1px;background:var(--line);border:1px solid var(--line);border-radius:16px;overflow:hidden;box-shadow:var(--shadow);margin-bottom:14px}.mission>div{background:#0b1715;padding:18px}.stateLabel,.label,h2{font:800 10px ui-monospace,monospace;letter-spacing:.13em;text-transform:uppercase;color:var(--muted)}.state{font-size:clamp(24px,4vw,40px);font-weight:900;letter-spacing:-.04em;margin-top:5px}.metric{font:800 clamp(20px,3vw,30px) ui-monospace,monospace;margin-top:7px}.unit{font-size:12px;color:var(--muted);margin-left:4px}
.layout{display:grid;grid-template-columns:1.2fr .95fr .95fr;gap:14px}.card{background:linear-gradient(145deg,#0e1c1a,#091311);border:1px solid var(--line);border-radius:16px;padding:16px;box-shadow:0 12px 35px #0004}.span2{grid-column:span 2}.cardHead{display:flex;justify-content:space-between;align-items:center;margin-bottom:13px}h2{margin:0;color:#b6cbc6}.tiny{color:var(--muted);font:700 11px ui-monospace,monospace}.rows{display:grid;gap:0}.row{display:flex;align-items:center;justify-content:space-between;gap:16px;padding:9px 0;border-top:1px solid #19302c}.row:first-child{border-top:0}.k{color:var(--muted)}.v{font:750 13px ui-monospace,SFMono-Regular,Menlo,monospace;text-align:right}.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.health{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.sensor{padding:12px;border:1px solid var(--line);border-radius:12px;background:#07110f}.sensor b{display:block;font:800 12px ui-monospace,monospace;margin-top:5px}.sensor.good{border-color:#55dd9155}.sensor.fail{border-color:#ff6b6b55}.attitude{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:12px}.att{text-align:center;padding:12px 4px;background:#07110f;border-radius:11px}.att strong{display:block;font:800 18px ui-monospace,monospace;color:var(--cyan)}
.servo{margin:13px 0}.servoTop{display:flex;justify-content:space-between;margin-bottom:7px}.track{height:9px;background:#07110f;border:1px solid var(--line);border-radius:99px;overflow:hidden}.fill{height:100%;width:0;background:linear-gradient(90deg,var(--cyan),var(--orange));transition:width .35s}.pulse{font:700 12px ui-monospace,monospace;color:var(--muted)}
.notice{border-left:3px solid var(--warn);background:#ffc85710;padding:11px 12px;border-radius:4px 10px 10px 4px;color:#f9e6ae;font-size:12px;margin-bottom:13px}.buttonGrid{display:grid;grid-template-columns:1fr 1fr;gap:8px}button{appearance:none;border:1px solid #31514a;background:#142824;color:var(--text);border-radius:11px;padding:11px 12px;font-weight:850;cursor:pointer;transition:.15s}button:hover{transform:translateY(-1px);border-color:#52786f}button:active{transform:translateY(0)}button.primary{background:var(--orange);border-color:var(--orange);color:#1b0c03}button.neutral{background:var(--ok);border-color:var(--ok);color:#04150c}button.danger{color:#ff9a9a;border-color:#743f3f;background:#281414}button:disabled{opacity:.35;cursor:not-allowed;transform:none}.full{width:100%}.field{margin-top:10px}.field label,.slider label{display:flex;justify-content:space-between;color:var(--muted);font-size:12px;margin-bottom:6px}input[type=text],input[type=number]{width:100%;border:1px solid var(--line);background:#07110f;color:var(--text);border-radius:10px;padding:11px 12px;outline:none}input:focus{border-color:var(--cyan)}input[type=range]{width:100%;accent-color:var(--orange)}.slider{margin:12px 0}.status{min-height:20px;margin-top:10px;color:var(--muted);font-size:12px}.commandRow{display:flex;gap:8px;margin-top:10px}.commandRow button{flex:1}.footer{display:flex;justify-content:space-between;gap:12px;color:#637d77;font:700 10px ui-monospace,monospace;padding:14px 3px 2px}
@media(max-width:950px){.mission{grid-template-columns:1fr 1fr}.layout{grid-template-columns:1fr 1fr}.span2{grid-column:span 2}}
@media(max-width:620px){.shell{padding:12px}.top{align-items:flex-start}.topright .pill:first-child{display:none}.mission,.layout{grid-template-columns:1fr}.span2{grid-column:span 1}.mission>div{padding:14px}.health{grid-template-columns:1fr 1fr 1fr}.buttonGrid{grid-template-columns:1fr}.state{font-size:28px}.footer{display:block}.footer span{display:block;margin-top:4px}}
</style>
</head>
<body>
<main class="shell">
  <header class="top">
    <div class="brand"><div class="mark">▲</div><div><div class="eyebrow">Recovery Operations</div><h1>PHOENIX Ground Station</h1></div></div>
    <div class="topright"><span id="agePill" class="pill">NO DATA</span><span id="linkPill" class="pill stale"><i class="dot"></i>STARTING</span></div>
  </header>

  <section class="mission">
    <div><div class="stateLabel">Flight State</div><div id="flight" class="state">WAITING</div><div id="guidance" class="tiny">GUIDANCE DISABLED</div></div>
    <div><div class="label">Altitude AGL</div><div class="metric"><span id="alt">--</span><span class="unit">m</span></div></div>
    <div><div class="label">Distance to Target</div><div class="metric"><span id="distance">--</span><span class="unit">m</span></div></div>
    <div><div class="label">Ground Speed</div><div class="metric"><span id="speed">--</span><span class="unit">m/s</span></div></div>
  </section>

  <section class="layout">
    <article class="card span2">
      <div class="cardHead"><h2>Navigation & Position</h2><span id="gpsBadge" class="tiny">GPS WAITING</span></div>
      <div class="rows">
        <div class="row"><span class="k">Current coordinates</span><span id="coords" class="v">--</span></div>
        <div class="row"><span class="k">Course / target bearing</span><span id="course" class="v">--</span></div>
        <div class="row"><span class="k">Heading error</span><span id="heading" class="v">--</span></div>
        <div class="row"><span class="k">Vertical speed</span><span id="vspeed" class="v">--</span></div>
      </div>
    </article>

    <article class="card">
      <div class="cardHead"><h2>Sensor Health</h2><span id="failsafe" class="tiny">--</span></div>
      <div class="health">
        <div id="gpsSensor" class="sensor"><span class="label">GPS</span><b id="gpsText">--</b></div>
        <div id="imuSensor" class="sensor"><span class="label">IMU</span><b id="imuText">--</b></div>
        <div id="baroSensor" class="sensor"><span class="label">BARO</span><b id="baroText">--</b></div>
      </div>
      <div class="attitude">
        <div class="att"><span class="label">Roll</span><strong id="roll">--</strong></div>
        <div class="att"><span class="label">Pitch</span><strong id="pitch">--</strong></div>
        <div class="att"><span class="label">Yaw</span><strong id="yaw">--</strong></div>
      </div>
    </article>

    <article class="card">
      <div class="cardHead"><h2>Servo Output</h2><span class="tiny">ACTUAL COMMAND</span></div>
      <div class="servo"><div class="servoTop"><b>Servo 1</b><span id="servo1Text" class="pulse">--</span></div><div class="track"><div id="servo1Fill" class="fill"></div></div></div>
      <div class="servo"><div class="servoTop"><b>Servo 2</b><span id="servo2Text" class="pulse">--</span></div><div class="track"><div id="servo2Fill" class="fill"></div></div></div>
      <button class="neutral full" onclick="neutral()">Neutral Both Servos</button>
    </article>

    <article class="card">
      <div class="cardHead"><h2>Bench Test</h2><span class="tiny">PAD_SAFE ONLY</span></div>
      <div class="notice">Use only with an inert, unloaded mechanism. Each test is limited, repeated briefly over LoRa, and automatically returns to neutral.</div>
      <div class="buttonGrid"><button id="bench1" onclick="bench(1)">Test Servo 1 · 8%</button><button id="bench2" onclick="bench(2)">Test Servo 2 · 8%</button></div>
      <div id="benchStatus" class="status">Ready for guarded bench test.</div>
    </article>

    <article class="card">
      <div class="cardHead"><h2>Drop Test Recording</h2><span id="dropGate" class="tiny">PAYLOAD OWNED</span></div>
      <div class="notice">Arm before release. The payload records locally and keeps servos neutral; LoRa timing is used only for supervision.</div>
      <div class="rows">
        <div class="row"><span class="k">Drop state</span><span id="dropState" class="v">--</span></div>
        <div class="row"><span class="k">Test ID</span><span id="dropId" class="v">--</span></div>
        <div class="row"><span class="k">Payload time</span><span id="payloadTime" class="v">--</span></div>
        <div class="row"><span class="k">Event times</span><span id="dropTimes" class="v">--</span></div>
      </div>
      <div class="commandRow"><button class="primary" onclick="armDrop()">Arm Drop Test</button><button class="danger" onclick="abortDrop()">Abort And Neutral</button></div>
      <button class="full" onclick="requestLogIndex()">Request Log Index</button>
      <div id="dropStatus" class="status">Waiting for payload telemetry.</div>
    </article>

    <article class="card">
      <div class="cardHead"><h2>Landing Target</h2><span class="tiny">LORA UPDATE</span></div>
      <div class="field"><label>Latitude</label><input id="lat" type="number" step="0.0000001" placeholder="37.1234567"></div>
      <div class="field"><label>Longitude</label><input id="lon" type="number" step="0.0000001" placeholder="-122.1234567"></div>
      <div class="commandRow"><button id="sendTarget" class="primary" onclick="sendTarget()">Send Target</button><button onclick="ping()">Ping</button></div>
      <div id="targetStatus" class="status">Target changes are validated by the payload.</div>
    </article>

    <article class="card span2">
      <div class="cardHead"><h2>Supervised Descent Steering</h2><span id="steerGate" class="tiny">LOCKED</span></div>
      <div class="notice">Manual steering unlocks only in GUIDED_DESCENT or FINAL_APPROACH. Powered ascent, deployment, failsafe, and landed states remain locked.</div>
      <div class="slider"><label>Servo 1 <b id="s1v">0%</b></label><input id="s1" type="range" min="-0.25" max="0.25" step="0.01" value="0"></div>
      <div class="slider"><label>Servo 2 <b id="s2v">0%</b></label><input id="s2" type="range" min="-0.25" max="0.25" step="0.01" value="0"></div>
      <div class="commandRow"><button id="sendSteer" class="primary" onclick="sendServo()" disabled>Send Timed Steering</button><button onclick="zeroSliders()">Reset Sliders</button></div>
      <div id="manualStatus" class="status">Waiting for a safe descent state.</div>
    </article>

    <article class="card">
      <div class="cardHead"><h2>Ground Link</h2><span id="radio" class="tiny">--</span></div>
      <div class="rows">
        <div class="row"><span class="k">Ground Wi‑Fi</span><span id="wifi" class="v">--</span></div>
        <div class="row"><span class="k">Connected devices</span><span id="clients" class="v">--</span></div>
        <div class="row"><span class="k">LoRa RSSI / SNR</span><span id="signal" class="v">--</span></div>
        <div class="row"><span class="k">Commands sent / failed</span><span id="counts" class="v">--</span></div>
        <div class="row"><span class="k">Last command</span><span id="lastcmd" class="v">--</span></div>
      </div>
      <button class="danger full" onclick="disableRemote()">Disable Remote Until Payload Reboot</button>
    </article>
  </section>
  <footer class="footer"><span>PHOENIX RECOVERY · OFFLINE GROUND CONSOLE</span><span id="footerStatus">CONNECTING TO LOCAL RADIO…</span></footer>
</main>

<script>
const el=id=>document.getElementById(id);
let dropRecordingActive=false;
let armWaitUntil=0;
async function api(path){const r=await fetch(path); const j=await r.json(); if(!r.ok) throw new Error(j.error||'request failed'); return j;}
async function command(q,statusId='manualStatus'){
  const box=el(statusId); if(box) box.textContent='Sending…';
  try { const j=await api('/api/command?'+q); if(q.indexOf('type=armdrop')>=0) armWaitUntil=Date.now()+6500; if(box) box.textContent='Sent: '+j.status.last_command; await refresh(); return true; }
  catch(e){ if(box) box.textContent='Error: '+e.message; return false; }
}
function dropLocked(statusId){
  if(!dropRecordingActive) return false;
  const box=el(statusId); if(box) box.textContent='Locked while payload is recording locally.';
  return true;
}
function sendServo(){if(dropLocked('manualStatus'))return;command('type=servo&servo1='+encodeURIComponent(el('s1').value)+'&servo2='+encodeURIComponent(el('s2').value))}
function zeroSliders(){el('s1').value=0;el('s2').value=0;updateSliders()}
function neutral(){zeroSliders();command('type=neutral')}
function ping(){command('type=ping','targetStatus')}
function bench(n){if(dropLocked('benchStatus'))return;if(confirm('Confirm inert, unloaded bench test for Servo '+n+'?'))command('type=bench&servo='+n,'benchStatus')}
function armDrop(){if(confirm('Arm payload-owned drop-test recording? Servos will stay neutral.'))command('type=armdrop','dropStatus')}
function abortDrop(){if(confirm('Abort drop recording and command neutral?'))command('type=abortdrop','dropStatus')}
function requestLogIndex(){command('type=logindex','dropStatus')}
function disableRemote(){ if(confirm('Disable rocket LoRa remote control until rocket reboot?')) command('type=disable') }
function sendTarget(){if(dropLocked('targetStatus'))return;command('type=target&lat='+encodeURIComponent(el('lat').value)+'&lon='+encodeURIComponent(el('lon').value),'targetStatus')}
function updateSliders(){el('s1v').textContent=Math.round(+el('s1').value*100)+'%';el('s2v').textContent=Math.round(+el('s2').value*100)+'%'}
el('s1').addEventListener('input',updateSliders); el('s2').addEventListener('input',updateSliders); updateSliders();
function health(card,text,good,label){card.className='sensor '+(good?'good':'fail');text.textContent=label}
function num(v,d=1){return Number.isFinite(v)?v.toFixed(d):'--'}
function armBlockers(t){
  const b=[];
  if(!t.valid||!t.fresh)b.push('payload link');
  if(!t.imu_valid)b.push('IMU');
  if(!t.baro_valid)b.push('BARO');
  if(t.failsafe&&t.failsafe!=='NONE')b.push('failsafe '+t.failsafe);
  return b;
}
async function refresh(){
  try{
    const s=await api('/api/status'), t=s.telemetry;
    dropRecordingActive=!!t.drop_test_recording;
    const link=el('linkPill');link.className='pill '+(t.fresh?'live':t.valid?'stale':'offline');link.innerHTML='<i class="dot"></i>'+(t.fresh?'PAYLOAD LINK LIVE':t.valid?'TELEMETRY STALE':'NO PAYLOAD LINK');
    el('agePill').textContent=t.valid?(t.age_ms+' ms · '+t.rssi+' dBm'):'NO DATA';
    el('flight').textContent=t.flight_state;el('guidance').textContent=t.guidance_mode+' · '+t.failsafe;
    el('alt').textContent=num(t.altitude_agl_m);el('distance').textContent=num(t.distance_to_target_m,0);el('speed').textContent=num(t.ground_speed_mps);
    el('coords').textContent=t.gps_valid?(t.lat.toFixed(7)+', '+t.lon.toFixed(7)):'Waiting for valid fix';
    el('course').textContent=num(t.gps_course_deg)+'° / '+num(t.target_bearing_deg)+'°';el('heading').textContent=num(t.heading_error_deg)+'°';el('vspeed').textContent=num(t.vertical_speed_mps,2)+' m/s';
    el('gpsBadge').textContent=t.gps_valid?(t.satellites+' SATELLITES · FIX'):(t.satellites+' SATELLITES · NO FIX');
    health(el('gpsSensor'),el('gpsText'),t.gps_valid,t.gps_valid?'FIX · '+t.satellites+' SAT':'NO FIX');health(el('imuSensor'),el('imuText'),t.imu_valid,t.imu_valid?'HEALTHY':'ERROR');health(el('baroSensor'),el('baroText'),t.baro_valid,t.baro_valid?'HEALTHY':'ERROR');
    el('dropState').textContent=t.drop_test_state;el('dropId').textContent=t.drop_test_id?('#'+t.drop_test_id):'--';
    el('payloadTime').textContent=t.payload_time_ms?(t.payload_time_ms+' ms onboard'):'--';
    el('dropTimes').textContent='payload ms: arm '+(t.drop_test_armed_ms||'--')+' · release '+(t.drop_test_release_ms||'--')+' · landing '+(t.drop_test_landing_ms||'--');
    el('dropGate').textContent=t.drop_test_neutral_lock?'NEUTRAL LOCK':'PAYLOAD OWNED';el('dropGate').className='tiny '+(t.drop_test_recording?'ok':'warn');
    if(t.drop_test_recording){armWaitUntil=0;el('dropStatus').textContent='Payload recording locally. LoRa is preview-only and quiet.';}
    else if(Date.now()<armWaitUntil){const b=armBlockers(t);el('dropStatus').textContent=b.length?('Arm sent; waiting for payload. Check '+b.join(', ')+'.'):'Arm sent; waiting for payload to accept.';}
    else{el('dropStatus').textContent='Drop recorder idle or closed.';}
    el('failsafe').textContent=t.failsafe==='NONE'?'FAILSAFE CLEAR':'FAILSAFE '+t.failsafe;el('failsafe').className='tiny '+(t.failsafe==='NONE'?'ok':'bad');
    el('roll').textContent=num(t.roll_deg)+'°';el('pitch').textContent=num(t.pitch_deg)+'°';el('yaw').textContent=num(t.yaw_deg)+'°';
    el('servo1Text').textContent=Math.round(t.servo1_cmd*100)+'% · '+t.servo1_us+' µs';el('servo2Text').textContent=Math.round(t.servo2_cmd*100)+'% · '+t.servo2_us+' µs';
    el('servo1Fill').style.width=Math.min(100,Math.abs(t.servo1_cmd)*100)+'%';el('servo2Fill').style.width=Math.min(100,Math.abs(t.servo2_cmd)*100)+'%';
    const steer=(t.flight_state==='GUIDED_DESCENT'||t.flight_state==='FINAL_APPROACH')&&!dropRecordingActive;el('sendSteer').disabled=!steer;el('sendTarget').disabled=dropRecordingActive;el('bench1').disabled=dropRecordingActive;el('bench2').disabled=dropRecordingActive;el('steerGate').textContent=dropRecordingActive?'LOCKED · DROP RECORDING':(steer?'UNLOCKED FOR DESCENT':'LOCKED · '+t.flight_state);el('steerGate').className='tiny '+(steer?'ok':'warn');
    el('wifi').textContent=s.wifi_ok?s.ip:'OFF';el('clients').textContent=s.clients;el('radio').textContent=s.radio_ok?'RADIO READY':'RADIO ERROR';el('radio').className='tiny '+(s.radio_ok?'ok':'bad');
    el('signal').textContent=t.valid?(t.rssi+' dBm / '+num(t.snr)+' dB'):'--';el('counts').textContent=s.sent_count+' / '+s.failed_count;el('lastcmd').textContent=s.last_command;el('footerStatus').textContent=s.last_error?('ERROR · '+s.last_error):'LOCAL SYSTEM NOMINAL';
  }catch(e){
    el('linkPill').className='pill offline';el('linkPill').innerHTML='<i class="dot"></i>DASHBOARD ERROR';el('footerStatus').textContent=e.message;
  }
}
setInterval(refresh,500); refresh();
</script>
</body>
</html>
)HTML";
}

void setupWeb() {
    const IPAddress ap_ip(192, 168, 8, 1);
    const IPAddress gateway(192, 168, 8, 1);
    const IPAddress subnet(255, 255, 255, 0);

    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.softAPConfig(ap_ip, gateway, subnet);
    wifi_ok = WiFi.softAP(GROUND_AP_SSID,
                          GROUND_AP_PASSWORD,
                          GROUND_AP_CHANNEL,
                          false,
                          GROUND_AP_MAX_CLIENTS);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    dns_ok = dns.start(53, "*", ap_ip);

    server.on("/", HTTP_GET, []() { server.send(200, "text/html", htmlPage()); });
    server.on("/api/status", HTTP_GET, []() { sendJson(200, statusJson()); });
    server.on("/api/command", HTTP_GET, handleCommandApi);

    const char* portal_paths[] = {
        "/generate_204", "/gen_204", "/hotspot-detect.html",
        "/library/test/success.html", "/connecttest.txt", "/ncsi.txt",
        "/canonical.html", "/success.txt"
    };
    for (const char* path : portal_paths) {
        server.on(path, HTTP_GET, []() {
            server.sendHeader("Location", "http://192.168.8.1/", true);
            server.send(302, "text/plain", "");
        });
    }

    server.onNotFound([]() {
        server.sendHeader("Location", "http://192.168.8.1/", true);
        server.send(302, "text/plain", "");
    });
    server.begin();

    Serial.printf("Ground Wi-Fi %s: SSID=%s password=%s IP=%s DNS=%s\n",
                  wifi_ok ? "ready" : "failed",
                  GROUND_AP_SSID,
                  GROUND_AP_PASSWORD,
                  WiFi.softAPIP().toString().c_str(),
                  dns_ok ? "on" : "off");
}

void setupLoRa() {
    SPI.begin(cfg::PIN_LORA_SCK, cfg::PIN_LORA_MISO, cfg::PIN_LORA_MOSI, cfg::PIN_LORA_NSS);
    const int16_t state = radio.begin(
        cfg::LORA_FREQ_MHZ,
        cfg::LORA_BW_KHZ,
        cfg::LORA_SF,
        cfg::LORA_CR,
        cfg::LORA_SYNC_WORD,
        cfg::LORA_TX_POWER,
        8,
        1.6
    );

    if (state != RADIOLIB_ERR_NONE) {
        radio_ok = false;
        last_error = "LoRa start failed: RadioLib code " + String(state);
        Serial.println(last_error);
        Serial.println("Check that this is a Heltec WiFi LoRa 32 V4/V4.3 board.");
        return;
    }

    radio_ok = true;
    radio.setPacketReceivedAction(onLoRaPacketReceived);
    radio.startReceive();
    Serial.printf("LoRa ready at %.1f MHz\n", cfg::LORA_FREQ_MHZ);
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(250);
    Serial.println();
    Serial.println("PHOENIX LoRa ground control starting...");
    setupWeb();
    setupLoRa();
    printHelp();
}

void loop() {
    while (Serial.available()) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\n' || ch == '\r') {
            handleLine(input_line);
            input_line = "";
        } else if (input_line.length() < 140) {
            input_line += ch;
        }
    }

    if (dns_ok) dns.processNextRequest();
    server.handleClient();
    updateManualHold();
    pollTelemetry();
    delay(2);
}
