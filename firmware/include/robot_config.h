#pragma once
// ============================================================
// robot_config.h — All robot parameters in one place
// ============================================================

// ── WiFi Configuration ���───────────────────────────────────
#define WIFI_SSID        "OTA"
#define WIFI_PASSWORD    "123456789"

// ── micro-ROS Agent Configuration ─────────────────────────
#define AGENT_IP         "10.82.42.50"
#define AGENT_PORT       8888

// ── Hardware Pins ─────────────────────────────────────────
#define PIN_SDA          21
#define PIN_SCL          22
#define PIN_LED          2

// ── PCA9685 I2C Addresses ─────────────────────────────────
#define PCA_ADDR_BOARD0  0x40
#define PCA_ADDR_BOARD1  0x41

// ── Servo PWM Settings ────────────────────────────────────
#define SERVO_FREQ_HZ    50
#define SERVO_PWM_MIN    102
#define SERVO_PWM_MAX    512

// ── Interpolation Settings ────────────────────────────────
#define MAX_DEG_PER_SECOND  120.0f
#define SERVO_UPDATE_HZ     100
#define MAX_DEG_PER_TICK    (MAX_DEG_PER_SECOND / SERVO_UPDATE_HZ)

// ── IMU Settings ──────────────────────────────────────────
// Now using raw I2C — config enums are in mpu6050_raw.h
// These old Adafruit defines are REMOVED:
//   MPU6050_RANGE_8_G, MPU6050_RANGE_500_DEG, MPU6050_BAND_21_HZ
#define IMU_PUBLISH_HZ   50

// ── Robot Dimensions ──────────────────────────────────────
#define ROBOT_HEIGHT_MM     500.0f
#define THIGH_LENGTH_MM     122.5f
#define SHANK_LENGTH_MM     123.0f
#define FOOT_HEIGHT_MM       50.0f
#define HIP_WIDTH_MM         80.0f

// ── Number of Joints ──────────────────────────────────────
#define NUM_JOINTS  18