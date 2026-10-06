// PHOENIX RECOVERY — isolated Servo 1 PWM diagnostic.
//
// This temporary bench image deliberately starts no Wi-Fi, LoRa, sensors, or
// flight logic. On each boot it performs exactly one unloaded movement:
// 1500 us neutral -> 1700 us (40% of the 500 us configured travel) -> neutral.
// It exists to distinguish a GPIO/PWM/servo problem from the LoRa command path.

#include <Arduino.h>
#include <ESP32Servo.h>

#include "config.h"

namespace {

Servo servo1;
constexpr int NEUTRAL_US = 1500;
constexpr int TEST_US = 1700;
constexpr int STEP_US = 10;
constexpr uint32_t STARTUP_NEUTRAL_MS = 5000;
constexpr uint32_t STEP_INTERVAL_MS = 20;
constexpr uint32_t TEST_HOLD_MS = 1500;

void writeAndReport(int pulse_us) {
    servo1.writeMicroseconds(pulse_us);
    Serial.printf("Servo 1 PWM: %d us\n", pulse_us);
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("PHOENIX ISOLATED SERVO 1 DIAGNOSTIC");
    Serial.println("One cycle only: neutral -> 40% -> neutral");

    servo1.setPeriodHertz(50);
    servo1.attach(cfg::PIN_SERVO_LEFT,
                  static_cast<int>(cfg::SERVO_LEFT_MIN_US),
                  static_cast<int>(cfg::SERVO_LEFT_MAX_US));
    if (!servo1.attached()) {
        Serial.println("ERROR: GPIO4 PWM attach failed; no movement attempted");
        return;
    }

    writeAndReport(NEUTRAL_US);
    Serial.println("Waiting 5 seconds at neutral. Keep the unloaded bench clear.");
    delay(STARTUP_NEUTRAL_MS);

    Serial.println("Ramping Servo 1 to 1700 us");
    for (int pulse = NEUTRAL_US + STEP_US; pulse <= TEST_US; pulse += STEP_US) {
        writeAndReport(pulse);
        delay(STEP_INTERVAL_MS);
    }
    delay(TEST_HOLD_MS);

    Serial.println("Returning Servo 1 to neutral");
    for (int pulse = TEST_US - STEP_US; pulse >= NEUTRAL_US; pulse -= STEP_US) {
        writeAndReport(pulse);
        delay(STEP_INTERVAL_MS);
    }
    writeAndReport(NEUTRAL_US);
    Serial.println("DIAGNOSTIC COMPLETE; Servo 1 remains neutral");
}

void loop() {
    delay(1000);
}
