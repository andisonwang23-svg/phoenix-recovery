// ============================================================================
// PHOENIX RECOVERY — IMU / I2C starting-point diagnostic
//
// Build:
//   pio run -e imu_i2c_scan
//
// Flash:
//   pio run -e imu_i2c_scan -t upload
//
// Monitor:
//   pio device monitor --port /dev/cu.usbmodem11201 --baud 115200
//
// Purpose:
//   Minimal new-ESP32 bring-up code. This intentionally avoids the full rocket
//   controller, servos, Wi-Fi, LoRa, and GPS so the first question is simple:
//   "Can the ESP32 see and read the BNO085/BMP388 on I2C?"
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_BMP3XX.h>

#include <cmath>

namespace {

constexpr int I2C_SDA_PIN = 48;
constexpr int I2C_SCL_PIN = 47;
constexpr uint32_t I2C_CLOCK_HZ = 100000;  // Conservative for bring-up.

constexpr uint8_t BNO08X_ADDR_LOW = 0x4A;
constexpr uint8_t BNO08X_ADDR_HIGH = 0x4B;
constexpr uint8_t BMP388_ADDR_LOW = 0x76;
constexpr uint8_t BMP388_ADDR_HIGH = 0x77;

constexpr uint32_t SCAN_INTERVAL_MS = 2000;
constexpr uint32_t IMU_REPORT_INTERVAL_MS = 50;
constexpr float PHOENIX_DEG_PER_RAD = 57.2957795131f;

BNO08x bno;
Adafruit_BMP3XX bmp;
bool imu_started = false;
bool baro_started = false;
uint8_t imu_address = 0;
uint8_t baro_address = 0;
uint32_t last_scan_ms = 0;
uint32_t last_tilt_print_ms = 0;
uint32_t last_baro_print_ms = 0;

void quaternionToEuler(float w, float x, float y, float z,
                       float& roll_deg, float& pitch_deg, float& yaw_deg) {
    const float sinr_cosp = 2.0f * (w * x + y * z);
    const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll_deg = std::atan2(sinr_cosp, cosr_cosp) * PHOENIX_DEG_PER_RAD;

    const float sinp = 2.0f * (w * y - z * x);
    if (std::fabs(sinp) >= 1.0f) {
        pitch_deg = std::copysign(90.0f, sinp);
    } else {
        pitch_deg = std::asin(sinp) * PHOENIX_DEG_PER_RAD;
    }

    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw_deg = std::atan2(siny_cosp, cosy_cosp) * PHOENIX_DEG_PER_RAD;
}

bool probeAddress(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

void printWiringHelp() {
    Serial.println();
    Serial.println("No BNO085/BNO08x was detected.");
    Serial.println("Check this first:");
    Serial.println("  BNO085 VIN/3V3 -> Heltec 3V3");
    Serial.println("  BNO085 GND     -> Heltec GND");
    Serial.printf("  BNO085 SDA     -> GPIO%d\n", I2C_SDA_PIN);
    Serial.printf("  BNO085 SCL     -> GPIO%d\n", I2C_SCL_PIN);
    Serial.println("  BNO085 address should be 0x4A or 0x4B.");
    Serial.println("If the module uses a Heltec connector with different pins, change");
    Serial.println("I2C_SDA_PIN and I2C_SCL_PIN at the top of this diagnostic file.");
    Serial.println();
}

void printBarometerWiringHelp() {
    Serial.println();
    Serial.println("No BMP388/BMP3xx barometer was detected.");
    Serial.println("For boards labeled SCK and SDI, use them as I2C pins:");
    Serial.println("  BMP388 VIN/VCC -> Heltec 3V3");
    Serial.println("  BMP388 GND     -> Heltec GND");
    Serial.printf("  BMP388 SDI/SDA -> GPIO%d\n", I2C_SDA_PIN);
    Serial.printf("  BMP388 SCK/SCL -> GPIO%d\n", I2C_SCL_PIN);
    Serial.println("  BMP388 address should be 0x77 or 0x76.");
    Serial.println("  For I2C, leave SDO and CS disconnected unless your board docs say otherwise.");
    Serial.println();
}

void scanI2CBus() {
    Serial.println();
    Serial.printf("[I2C] Scanning SDA=GPIO%d SCL=GPIO%d @ %lu Hz...\n",
                  I2C_SDA_PIN, I2C_SCL_PIN, static_cast<unsigned long>(I2C_CLOCK_HZ));

    uint8_t found_count = 0;
    bool found_bno = false;
    bool found_bmp388 = false;

    for (uint8_t address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        const uint8_t error = Wire.endTransmission();
        if (error == 0) {
            ++found_count;
            Serial.printf("  found device at 0x%02X", address);
            if (address == BNO08X_ADDR_LOW || address == BNO08X_ADDR_HIGH) {
                found_bno = true;
                Serial.print("  <-- BNO085/BNO08x likely");
            } else if (address == BMP388_ADDR_LOW || address == BMP388_ADDR_HIGH) {
                found_bmp388 = true;
                Serial.print("  <-- BMP388/BMP3xx barometer likely");
            }
            Serial.println();
        } else if (error == 4) {
            Serial.printf("  unknown I2C error at 0x%02X\n", address);
        }
        delay(2);
    }

    if (found_count == 0) {
        Serial.println("  no I2C devices found");
    }

    if (!found_bno) {
        printWiringHelp();
    }
    if (found_bmp388) {
        Serial.println("[BARO] BMP388/BMP3xx likely detected.");
    } else {
        printBarometerWiringHelp();
    }
}

bool startIMUAt(uint8_t address) {
    if (!probeAddress(address)) {
        return false;
    }

    if (!bno.begin(address, Wire)) {
        Serial.printf("[IMU] Device answered at 0x%02X, but BNO08x begin failed.\n", address);
        return false;
    }

    imu_address = address;
    imu_started = true;
    bno.enableRotationVector(IMU_REPORT_INTERVAL_MS);
    bno.enableGyro(IMU_REPORT_INTERVAL_MS);
    bno.enableAccelerometer(IMU_REPORT_INTERVAL_MS);

    Serial.printf("[IMU] BNO085/BNO08x started at 0x%02X\n", imu_address);
    Serial.println("[IMU] Tilt output will print once per second.");
    return true;
}

void tryStartIMU() {
    if (imu_started) {
        return;
    }

    if (startIMUAt(BNO08X_ADDR_HIGH)) {
        return;
    }
    if (startIMUAt(BNO08X_ADDR_LOW)) {
        return;
    }

    Serial.println("[IMU] Not started yet.");
}

bool startBarometerAt(uint8_t address) {
    if (!probeAddress(address)) {
        return false;
    }

    if (!bmp.begin_I2C(address, &Wire)) {
        Serial.printf("[BARO] Device answered at 0x%02X, but BMP3xx begin failed.\n", address);
        return false;
    }

    baro_address = address;
    baro_started = true;
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_25_HZ);

    Serial.printf("[BARO] BMP388/BMP3xx started at 0x%02X\n", baro_address);
    Serial.println("[BARO] Pressure/temperature will print once per second.");
    return true;
}

void tryStartBarometer() {
    if (baro_started) {
        return;
    }

    if (startBarometerAt(BMP388_ADDR_HIGH)) {
        return;
    }
    if (startBarometerAt(BMP388_ADDR_LOW)) {
        return;
    }

    Serial.println("[BARO] Not started yet.");
}

void printTiltIfAvailable() {
    if (!imu_started) {
        return;
    }

    if (bno.wasReset()) {
        Serial.println("[IMU] Sensor reset detected; re-enabling reports.");
        bno.enableRotationVector(IMU_REPORT_INTERVAL_MS);
        bno.enableGyro(IMU_REPORT_INTERVAL_MS);
        bno.enableAccelerometer(IMU_REPORT_INTERVAL_MS);
    }

    if (!bno.getSensorEvent()) {
        return;
    }

    if (bno.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) {
        return;
    }

    const uint32_t now = millis();
    if (now - last_tilt_print_ms < 1000) {
        return;
    }
    last_tilt_print_ms = now;

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    quaternionToEuler(bno.getQuatReal(), bno.getQuatI(), bno.getQuatJ(), bno.getQuatK(),
                      roll, pitch, yaw);

    Serial.printf("[TILT] roll=%7.2f deg  pitch=%7.2f deg  yaw=%7.2f deg  accuracy=%u\n",
                  roll, pitch, yaw, bno.getQuatAccuracy());
}

void printBarometerIfAvailable() {
    if (!baro_started) {
        return;
    }

    const uint32_t now = millis();
    if (now - last_baro_print_ms < 1000) {
        return;
    }
    last_baro_print_ms = now;

    if (!bmp.performReading()) {
        Serial.println("[BARO] performReading failed.");
        return;
    }

    Serial.printf("[BARO] addr=0x%02X pressure=%8.2f hPa temp=%6.2f C altitude=%7.2f m\n",
                  baro_address, bmp.pressure / 100.0f, bmp.temperature, bmp.readAltitude(1013.25f));
}

}  // namespace

void setup() {
    Serial.begin(115200);
    const uint32_t start_ms = millis();
    while (!Serial && millis() - start_ms < 2000) {
        delay(10);
    }

    Serial.println();
    Serial.println("============================================================");
    Serial.println("PHOENIX RECOVERY — IMU / I2C STARTING POINT");
    Serial.println("This firmware only tests the I2C bus, BNO085, and BMP388.");
    Serial.println("No servos, Wi-Fi, LoRa, GPS, or flight control.");
    Serial.println("============================================================");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(50);

    scanI2CBus();
    tryStartIMU();
    tryStartBarometer();
}

void loop() {
    printTiltIfAvailable();
    printBarometerIfAvailable();

    const uint32_t now = millis();
    if (now - last_scan_ms >= SCAN_INTERVAL_MS) {
        last_scan_ms = now;
        scanI2CBus();
        tryStartIMU();
        tryStartBarometer();
    }

    delay(5);
}
