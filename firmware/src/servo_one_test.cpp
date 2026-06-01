// ============================================================
// servo_one_test.cpp — STANDALONE single-servo test
// ============================================================
//
// No micro-ROS, no WiFi, no ROS2. Just the ESP32 + one PCA9685
// driving ONE servo to center (90°) and holding it.
//
// Build/flash ONLY this (does not touch the micro-ROS firmware):
//   pio run -e servo_test -t upload
//   pio device monitor -e servo_test
//
// What it does:
//   - inits I2C (SDA 21 / SCL 22) and the PCA9685 at 0x40 @ 50 Hz
//   - drives TEST_CHANNEL to 90° (center) and holds
//   - prints a heartbeat every 2 s so you know it's alive
//
// To test a different channel, change TEST_CHANNEL below.
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "robot_config.h"   // reuse PIN_SDA/SCL, PCA addr, PWM limits

// ── Which servo to test ──────────────────────────────────────
static const uint8_t TEST_CHANNEL = 0;     // PCA9685 channel (0 = r_hip_roll)
static const float   TEST_ANGLE   = 90.0f; // hold position, degrees (center)

// PCA9685 driver at board 0 (0x40)
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA_ADDR_BOARD0);

// Map an angle (0–180°) to a PCA9685 "off" tick using the same
// SERVO_PWM_MIN..SERVO_PWM_MAX range the real firmware uses.
static uint16_t angle_to_ticks(float deg) {
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    float ratio = deg / 180.0f;
    return (uint16_t)(SERVO_PWM_MIN + ratio * (SERVO_PWM_MAX - SERVO_PWM_MIN));
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("=== STANDALONE single-servo test (no micro-ROS) ===");
    Serial.printf("I2C SDA=%d SCL=%d, PCA9685 @ 0x%02X, %d Hz\n",
                  PIN_SDA, PIN_SCL, PCA_ADDR_BOARD0, SERVO_FREQ_HZ);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);

    pca.begin();
    pca.setPWMFreq(SERVO_FREQ_HZ);   // 50 Hz for hobby servos
    delay(10);

    uint16_t ticks = angle_to_ticks(TEST_ANGLE);
    pca.setPWM(TEST_CHANNEL, 0, ticks);
    Serial.printf("Driving channel %u to %.0f deg (ticks=%u). Holding.\n",
                  TEST_CHANNEL, TEST_ANGLE, ticks);
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last >= 2000) {
        last = millis();
        Serial.printf("[%lus] holding ch %u at %.0f deg\n",
                      millis() / 1000, TEST_CHANNEL, TEST_ANGLE);
    }
}
