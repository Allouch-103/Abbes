// ============================================================
// main.cpp — Humanoid Robot ESP32 Firmware (PlatformIO)
// ============================================================
//
// This is now just the entry point. All logic lives in modules:
//
//   servo_controller  — PCA9685 servo management
//   mpu6050_raw       — Raw I2C IMU driver (no library)
//   microros_transport — micro-ROS lifecycle + callbacks
//
// Architecture:
//   Core 0: setup() + loop() (status printing, future watchdog)
//   Core 1: microROSTask (executor, callbacks, comms)
//
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

#include "robot_config.h"
#include "servo_controller.h"
#include "mpu6050_raw.h"
#include "microros_transport.h"

// ── IMU available flag (shared with microros_transport) ────
bool imu_available = false;

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n================================");
    Serial.println("  Humanoid Robot ESP32 Firmware ");
    Serial.println("  micro-ROS over WiFi (Jazzy)   ");
    Serial.println("================================\n");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // ── I2C bus ───────────────────────────────────────────
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);
    Serial.println("[I2C] Bus initialized (SDA=21, SCL=22, 100kHz)");

    // ── Servos ────────────────────────────────────────────
    servos_init();

    // ── IMU (raw I2C, no library) ─────────────────────────
    Serial.println("[MPU6050] Initializing (raw I2C)...");
    if (mpu6050_init(ACCEL_RANGE_8G, GYRO_RANGE_500, DLPF_21HZ)) {
        imu_available = true;
        Serial.println("[MPU6050] Ready (±8g accel, ±500°/s gyro, 21Hz LPF)");
    } else {
        Serial.println("[MPU6050] NOT FOUND — check wiring to GPIO 21/22");
        Serial.println("[MPU6050] Continuing without IMU...");
    }

    // ── Rest position ─────────────────────────────────────
    servos_move_to_rest_blocking();

    // ── Start micro-ROS on Core 1 ─────────────────────────
    microros_start_task();

    Serial.println("[Setup] Waiting for WiFi + agent connection...");
    Serial.printf("[Setup] Expected agent at: %s:%d\n", AGENT_IP, AGENT_PORT);
}

// ============================================================
// LOOP (Core 0 — free for future watchdog/safety)
// ============================================================

void loop() {
    static uint32_t last_print = 0;
    if (millis() - last_print > 5000) {
        last_print = millis();
        const char* state_names[] = {
            "WAITING_WIFI", "WAITING_AGENT", "CONNECTED", "DISCONNECTED"
        };
        Serial.printf("[Status] Agent: %s | WiFi: %s | Stack free: %d bytes\n",
                      state_names[(int)agent_state],
                      WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
                      uxTaskGetStackHighWaterMark(microros_task_handle) * 4);
    }

    delay(100);
}