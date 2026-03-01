#pragma once
// ============================================================
// servo_controller.h — Servo management (PCA9685)
// ============================================================

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "robot_config.h"
#include "joint_definitions.h"

// ── Public API ─────────────────────────────────────────────
void servos_init();
void servos_move_to_rest_blocking();
void servo_set(uint8_t joint_idx, float angle_deg);
void servos_interpolation_tick();

// ── Shared state (accessed by micro-ROS callbacks) ─────────
extern float current_deg[NUM_JOINTS];
extern float target_deg[NUM_JOINTS];