#pragma once
// ============================================================
// mpu6050_raw.h — Raw I2C driver for MPU6050 (no library)
// ============================================================
//
// WHY RAW I2C?
// The Adafruit library adds ~15KB of flash and pulls in
// Adafruit_Sensor + BusIO dependencies. For our use case
// (read accel + gyro at 50Hz), raw I2C is simpler and
// more reliable — fewer layers that can fail.
//
// MPU6050 I2C PROTOCOL:
// 1. Send device address (0x68) + register address
// 2. Read back N bytes
// All sensor data is in big-endian 16-bit signed integers.
//
// ============================================================

#include <Arduino.h>
#include <Wire.h>

// ── MPU6050 I2C Address ────────────────────────────────────
// AD0 pin LOW  → 0x68 (default)
// AD0 pin HIGH → 0x69
#define MPU6050_ADDR        0x68

// ── MPU6050 Register Map ───────────────��───────────────────
// Full register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
#define MPU6050_REG_SMPLRT_DIV    0x19  // Sample Rate Divider
#define MPU6050_REG_CONFIG        0x1A  // DLPF (Digital Low Pass Filter)
#define MPU6050_REG_GYRO_CONFIG   0x1B  // Gyroscope range
#define MPU6050_REG_ACCEL_CONFIG  0x1C  // Accelerometer range
#define MPU6050_REG_ACCEL_XOUT_H  0x3B  // First byte of accel data (6 bytes: XH,XL,YH,YL,ZH,ZL)
#define MPU6050_REG_GYRO_XOUT_H   0x43  // First byte of gyro data  (6 bytes: XH,XL,YH,YL,ZH,ZL)
#define MPU6050_REG_PWR_MGMT_1    0x6B  // Power management (write 0 to wake up)
#define MPU6050_REG_WHO_AM_I      0x75  // Should return 0x68

// ── Sensitivity scale factors ──────────────────────────────
// These convert raw 16-bit integers to physical units.
//
// Accelerometer (from datasheet Table 6.2):
//   ±2g  → 16384 LSB/g
//   ±4g  → 8192  LSB/g
//   ±8g  → 4096  LSB/g   ← we use this
//   ±16g → 2048  LSB/g
//
// Gyroscope (from datasheet Table 6.1):
//   ±250°/s  → 131.0 LSB/(°/s)
//   ±500°/s  → 65.5  LSB/(°/s)   ← we use this
//   ±1000°/s → 32.8  LSB/(°/s)
//   ±2000°/s → 16.4  LSB/(°/s)

// ── Configuration enums ────────────────────────────────────
// Accel range: bits 4:3 of register 0x1C
enum MPU6050_AccelRange : uint8_t {
    ACCEL_RANGE_2G  = 0x00,  // ±2g
    ACCEL_RANGE_4G  = 0x08,  // ±4g
    ACCEL_RANGE_8G  = 0x10,  // ±8g   ← default for our robot
    ACCEL_RANGE_16G = 0x18,  // ±16g
};

// Gyro range: bits 4:3 of register 0x1B
enum MPU6050_GyroRange : uint8_t {
    GYRO_RANGE_250  = 0x00,  // ±250°/s
    GYRO_RANGE_500  = 0x08,  // ±500°/s  ← default for our robot
    GYRO_RANGE_1000 = 0x10,  // ±1000°/s
    GYRO_RANGE_2000 = 0x18,  // ±2000°/s
};

// DLPF bandwidth: bits 2:0 of register 0x1A
// Lower bandwidth = more filtering = less noise but more delay
enum MPU6050_DLPFBandwidth : uint8_t {
    DLPF_260HZ = 0,  // essentially no filter
    DLPF_184HZ = 1,
    DLPF_94HZ  = 2,
    DLPF_44HZ  = 3,
    DLPF_21HZ  = 4,  // ← good for 50Hz publish rate
    DLPF_10HZ  = 5,
    DLPF_5HZ   = 6,
};

// ── IMU data structure ─────────────────────────────────────
struct IMUData {
    float accel_x;   // m/s²
    float accel_y;   // m/s²
    float accel_z;   // m/s²
    float gyro_x;    // rad/s
    float gyro_y;    // rad/s
    float gyro_z;    // rad/s
};

// ── Public API ─────────────────────────────────────────────
bool mpu6050_init(MPU6050_AccelRange accel_range = ACCEL_RANGE_8G,
                  MPU6050_GyroRange  gyro_range  = GYRO_RANGE_500,
                  MPU6050_DLPFBandwidth dlpf_bw  = DLPF_21HZ);

bool mpu6050_read(IMUData& data);