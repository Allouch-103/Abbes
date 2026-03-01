// ============================================================
// mpu6050_raw.cpp — Raw I2C MPU6050 implementation
// ============================================================
//
// HOW I2C COMMUNICATION WORKS:
//
// I2C is a 2-wire protocol: SDA (data) and SCL (clock).
// Every device on the bus has a unique 7-bit address.
//
// To WRITE a register:
//   1. Master sends START
//   2. Master sends device address + WRITE bit
//   3. Device ACKs
//   4. Master sends register address
//   5. Device ACKs
//   6. Master sends data byte
//   7. Device ACKs
//   8. Master sends STOP
//
// To READ a register:
//   1. Master sends START
//   2. Master sends device address + WRITE bit
//   3. Device ACKs
//   4. Master sends register address
//   5. Master sends REPEATED START
//   6. Master sends device address + READ bit
//   7. Device sends data byte(s)
//   8. Master sends NACK on last byte
//   9. Master sends STOP
//
// Wire library handles all the low-level bit-banging.
// We just use Wire.beginTransmission/write/endTransmission
// for writes, and Wire.requestFrom/read for reads.
//
// ============================================================

#include "mpu6050_raw.h"

// ── Module-level state ─────────────────────────────────────
static float accel_scale = 1.0f;   // LSB → m/s²
static float gyro_scale  = 1.0f;   // LSB → rad/s

// ── Low-level I2C helpers ──────────────────────────────────

/**
 * writeRegister — Write one byte to an MPU6050 register
 *
 * Returns true on success (ACK received).
 */
static bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

/**
 * readRegister — Read one byte from an MPU6050 register
 */
static uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);  // false = repeated start (don't release bus)
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return Wire.read();
}

/**
 * readRegisters — Read N consecutive bytes starting at reg
 *
 * MPU6050 supports burst reads: after sending the start register,
 * the internal address pointer auto-increments on each read.
 * This lets us read 6 bytes (3 axes × 2 bytes) in one I2C transaction,
 * which is faster and ensures all axes are sampled at the same instant.
 */
static bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t count) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)MPU6050_ADDR, count);
    if (Wire.available() < count) return false;

    for (uint8_t i = 0; i < count; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

// ── Public functions ───────────────────────────────────────

bool mpu6050_init(MPU6050_AccelRange accel_range,
                  MPU6050_GyroRange  gyro_range,
                  MPU6050_DLPFBandwidth dlpf_bw) {

    // Step 1: Check WHO_AM_I register
    // Should return 0x68 (the I2C address without the R/W bit)
    uint8_t who = readRegister(MPU6050_REG_WHO_AM_I);
    if (who != 0x68 && who != 0x70) {
        Serial.printf("[MPU6050] WHO_AM_I = 0x%02X (expected 0x68 or 0x70)\n", who);
        return false;
    }
    Serial.printf("[MPU6050] WHO_AM_I = 0x%02X — OK\n", who);

    // Step 2: Wake up the device
    // On power-up, MPU6050 is in sleep mode (bit 6 of PWR_MGMT_1 = 1)
    // Writing 0x00 clears sleep bit and selects internal 8MHz oscillator
    if (!writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00)) {
        Serial.println("[MPU6050] Failed to wake up device");
        return false;
    }
    delay(100);  // Wait for oscillator to stabilize

    // Step 3: Configure DLPF (Digital Low Pass Filter)
    // Register 0x1A, bits 2:0
    // DLPF_21HZ → accel delay 8.5ms, gyro delay 8.3ms
    // This filters out vibration noise above 21Hz
    writeRegister(MPU6050_REG_CONFIG, dlpf_bw);

    // Step 4: Configure sample rate divider
    // Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
    // With DLPF enabled, Gyro Output Rate = 1kHz
    // SMPLRT_DIV = 9 → Sample Rate = 1000 / (1+9) = 100Hz
    // We publish at 50Hz, so 100Hz sampling gives us 2x oversampling
    writeRegister(MPU6050_REG_SMPLRT_DIV, 9);

    // Step 5: Set accelerometer range
    // Register 0x1C, bits 4:3
    writeRegister(MPU6050_REG_ACCEL_CONFIG, accel_range);

    // Step 6: Set gyroscope range
    // Register 0x1B, bits 4:3
    writeRegister(MPU6050_REG_GYRO_CONFIG, gyro_range);

    // Step 7: Compute scale factors
    // We convert raw int16 → physical units in mpu6050_read()
    //
    // Accel: raw / sensitivity * 9.80665 = m/s²
    // Gyro:  raw / sensitivity * (π/180) = rad/s
    switch (accel_range) {
        case ACCEL_RANGE_2G:  accel_scale = 9.80665f / 16384.0f; break;
        case ACCEL_RANGE_4G:  accel_scale = 9.80665f / 8192.0f;  break;
        case ACCEL_RANGE_8G:  accel_scale = 9.80665f / 4096.0f;  break;
        case ACCEL_RANGE_16G: accel_scale = 9.80665f / 2048.0f;  break;
    }

    switch (gyro_range) {
        case GYRO_RANGE_250:  gyro_scale = (M_PI / 180.0f) / 131.0f;  break;
        case GYRO_RANGE_500:  gyro_scale = (M_PI / 180.0f) / 65.5f;   break;
        case GYRO_RANGE_1000: gyro_scale = (M_PI / 180.0f) / 32.8f;   break;
        case GYRO_RANGE_2000: gyro_scale = (M_PI / 180.0f) / 16.4f;   break;
    }

    Serial.println("[MPU6050] Initialized via raw I2C");
    return true;
}

bool mpu6050_read(IMUData& data) {
    // Read 14 bytes starting at ACCEL_XOUT_H (0x3B):
    //   Bytes 0-5:  Accel X, Y, Z (each 2 bytes, big-endian, signed)
    //   Bytes 6-7:  Temperature (we skip this)
    //   Bytes 8-13: Gyro X, Y, Z (each 2 bytes, big-endian, signed)
    //
    // Reading all 14 bytes in one burst is faster than two separate
    // 6-byte reads, and guarantees all data is from the same sample.
    uint8_t buf[14];
    if (!readRegisters(MPU6050_REG_ACCEL_XOUT_H, buf, 14)) {
        return false;
    }

    // Combine high and low bytes into signed 16-bit integers
    // MPU6050 is big-endian: high byte first, then low byte
    // (int16_t) cast is critical — raw values are signed (-32768 to +32767)
    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6], buf[7] = temperature (skipped)
    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    // Convert to physical units
    data.accel_x = raw_ax * accel_scale;
    data.accel_y = raw_ay * accel_scale;
    data.accel_z = raw_az * accel_scale;
    data.gyro_x  = raw_gx * gyro_scale;
    data.gyro_y  = raw_gy * gyro_scale;
    data.gyro_z  = raw_gz * gyro_scale;

    return true;
}