// ============================================================
// servo_controller.cpp — Servo management implementation
// ============================================================

#include "servo_controller.h"

// ── Hardware ───────────────────────────────────────────────
static Adafruit_PWMServoDriver pca[2] = {
    Adafruit_PWMServoDriver(PCA_ADDR_BOARD0, Wire),
    Adafruit_PWMServoDriver(PCA_ADDR_BOARD1, Wire),
};

// ── Servo state ────────────────────────────────────────────
float current_deg[NUM_JOINTS];
float target_deg[NUM_JOINTS];

// ── Internal helpers ───────────────────────────────────────

/**
 * angleToPWM — Convert degrees to PCA9685 PWM count
 *
 * PCA9685 at 50Hz: one period = 20ms, 12-bit counter (0-4095)
 * One count = 20ms / 4096 = 4.8828µs
 *
 * MG996R: 500µs (0°) → 102 counts, 2500µs (180°) → 512 counts
 */
static uint16_t angleToPWM(float angle, bool inverted) {
    float ratio = angle / 180.0f;
    if (inverted) ratio = 1.0f - ratio;
    return (uint16_t)(SERVO_PWM_MIN + ratio * (float)(SERVO_PWM_MAX - SERVO_PWM_MIN));
}

// ── Public functions ───────────────────────────────────────

void servo_set(uint8_t joint_idx, float angle_deg) {
    const JointConfig& j = JOINT_CONFIG[joint_idx];
    angle_deg = constrain(angle_deg, j.min_deg, j.max_deg);
    uint16_t pwm = angleToPWM(angle_deg, j.inverted);
    pca[j.pca_board].setPWM(j.pca_channel, 0, pwm);
}

void servos_init() {
    Serial.println("[PCA9685] Initializing board 0 (0x40)...");
    pca[0].begin();
    pca[0].setPWMFreq(SERVO_FREQ_HZ);
    delay(10);

    /*Serial.println("[PCA9685] Initializing board 1 (0x41)...");
    pca[1].begin();
    pca[1].setPWMFreq(SERVO_FREQ_HZ);
    delay(10);

    Serial.println("[PCA9685] Both boards ready");*/
}

void servos_move_to_rest_blocking() {
    Serial.println("Moving to rest position (1 second)...");
    const int STEPS = 50;
    for (int step = 0; step <= STEPS; step++) {
        float t = (float)step / STEPS;
        for (int i = 0; i < NUM_JOINTS; i++) {
            float angle = 90.0f + t * (JOINT_CONFIG[i].rest_deg - 90.0f);
            servo_set(i, angle);
            current_deg[i] = angle;
        }
        delay(20);
    }
    for (int i = 0; i < NUM_JOINTS; i++) {
        current_deg[i] = JOINT_CONFIG[i].rest_deg;
        target_deg[i]  = JOINT_CONFIG[i].rest_deg;
    }
    Serial.println("Rest position reached.");
}

/**
 * servos_interpolation_tick — Move current toward target
 *
 * Called every 10ms by servo_timer callback.
 * Velocity-limited: max MAX_DEG_PER_TICK per call.
 */
void servos_interpolation_tick() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        float error = target_deg[i] - current_deg[i];
        if (fabsf(error) <= MAX_DEG_PER_TICK) {
            current_deg[i] = target_deg[i];
        } else {
            current_deg[i] += copysignf(MAX_DEG_PER_TICK, error);
        }
        servo_set(i, current_deg[i]);
    }
}