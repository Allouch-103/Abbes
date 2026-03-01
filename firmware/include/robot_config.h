#pragma once
// ============================================================
// robot_config.h — All robot parameters in one place
// ============================================================
//
// WHY A SEPARATE CONFIG FILE?
// When you tune the robot (adjust speeds, joint limits, servo
// calibration), you only edit THIS file. The main code stays
// clean and untouched. This is standard embedded practice.
//
// ============================================================

// ── WiFi Configuration ────────────────────────────────────
#define WIFI_SSID        "OTA"
#define WIFI_PASSWORD    "123456789"

// ── micro-ROS Agent Configuration ─────────────────────────
#define AGENT_IP         "10.82.42.50"
#define AGENT_PORT       8888

// ── Hardware Pins ─────────────────────────────────────────
#define PIN_SDA          21
#define PIN_SCL          22

// Built-in LED — lights up solid when micro-ROS is connected
#define PIN_LED          2

// ── PCA9685 I2C Addresses ─────────────────────────────────
// Board 0 (default, no jumpers soldered): 0x40
// Board 1 (A0 jumper soldered):           0x41
//
// HOW I2C ADDRESSES WORK:
// The PCA9685 has 6 address pins (A0-A5) on the board.
// Each pin adds 2^n to the base address 0x40.
// A0 bridged = 0x40 + 1 = 0x41
// A1 bridged = 0x40 + 2 = 0x42
// A0+A1      = 0x40 + 3 = 0x43
// etc. up to 0x7F (supports 64 boards on one I2C bus!)
#define PCA_ADDR_BOARD0  0x40
#define PCA_ADDR_BOARD1  0x41

// ── Servo PWM Settings ────────────────────────────────────
// MG996R operates at 50Hz (20ms period)
// Pulse width range: 500µs (0°) to 2500µs (180°)
//
// PCA9685 PWM count calculation:
//   The PCA9685 has a 12-bit counter: 0 to 4095
//   At 50Hz, one period = 20ms
//   One count = 20ms / 4096 = 4.8828µs
//
//   MIN count = 500µs  / 4.8828µs = 102.4 → 102
//   MAX count = 2500µs / 4.8828µs = 512.0 → 512
//
// CALIBRATION NOTE:
// These are nominal values. Real servos vary ±10%.
// If a servo doesn't reach full range or goes past limits,
// tweak these values. Typical range: MIN=100-110, MAX=490-530.
#define SERVO_FREQ_HZ    50
#define SERVO_PWM_MIN    102    // = 0°
#define SERVO_PWM_MAX    512    // = 180°

// ── Interpolation Settings ────────────────────────────────
// Controls how fast servos are allowed to move.
//
// WHY LIMIT SPEED?
// MG996R no-load speed: ~333°/s at 6V
// If we command a 90° move instantly:
//   - Servo draws up to 2.5A for ~0.27 seconds
//   - 18 servos potentially = 45A peak → brownout/damage
//   - Sudden angular momentum disrupts balance
//
// At 120°/s max speed:
//   - 90° move takes 750ms (smooth, controlled)
//   - Current stays under 1A per servo
//   - Center of mass shifts gradually (better for balance)
//
// Increase this only once everything is working.
#define MAX_DEG_PER_SECOND  120.0f

// Servo update runs at 100Hz (every 10ms)
// Max movement per tick: 120°/s ÷ 100Hz = 1.2° per tick
#define SERVO_UPDATE_HZ     100
#define MAX_DEG_PER_TICK    (MAX_DEG_PER_SECOND / SERVO_UPDATE_HZ)

// ── IMU Settings ──────────────────────────────────────────
// MPU6050 configuration
// Accelerometer range: ±2g, ±4g, ±8g, ±16g
//   Smaller range = more resolution but clips if exceeded
//   ±8g is safe for walking robot (fall = ~5g impact)
//
// Gyroscope range: ±250, ±500, ±1000, ±2000 °/s
//   Walking joints rarely exceed 200°/s so ±500 is safe
//
// Low-pass filter bandwidth: 5, 10, 21, 44, 94, 184, 260 Hz
//   We update at 50Hz so we want filter cutoff ~21Hz
//   This removes vibration noise above 21Hz
#define IMU_ACCEL_RANGE  MPU6050_RANGE_8_G
#define IMU_GYRO_RANGE   MPU6050_RANGE_500_DEG
#define IMU_FILTER_BW    MPU6050_BAND_21_HZ

// IMU publish rate
#define IMU_PUBLISH_HZ   50

// ── Robot Dimensions ──────────────────────────────────────
// From Winter's anthropometric model scaled to H=50cm
// Used by the IK solver (Phase 4). Measure your actual robot!
// These are estimates — measure with a ruler and update.
//
// How to measure: with robot standing straight, measure:
//   THIGH_LENGTH: hip joint center to knee joint center
//   SHANK_LENGTH: knee joint center to ankle joint center
//   FOOT_HEIGHT:  ankle joint center to ground
#define ROBOT_HEIGHT_MM     500.0f
#define THIGH_LENGTH_MM     122.5f   // 0.245 × H
#define SHANK_LENGTH_MM     123.0f   // 0.246 × H
#define FOOT_HEIGHT_MM       50.0f   // ankle to ground
#define HIP_WIDTH_MM         80.0f   // distance between hip joints

// ── Number of Joints ──────────────────────────────────────
#define NUM_JOINTS  18