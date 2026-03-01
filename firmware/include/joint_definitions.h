#pragma once
// ============================================================
// joint_definitions.h — Servo hardware mapping
// ============================================================
//
// This file maps logical joint indices (0-17, matching your
// report Table 3.1) to physical hardware locations on the
// PCA9685 boards, and stores all joint limit data.
//
// JOINT INDEX CONVENTION:
// We use 0-based indexing in code (joint 0 = report's Joint 1)
// This matches C arrays and avoids off-by-one errors.
//
// ============================================================

#include <Arduino.h>
#include "robot_config.h"

// ── Joint configuration structure ─────────────────────────
struct JointConfig {
    const char* name;       // human-readable name for debug logging
    uint8_t  pca_board;     // 0 = board at 0x40, 1 = board at 0x41
    uint8_t  pca_channel;   // 0-15 on that board
    float    min_deg;       // minimum allowed angle (from Table 3.1)
    float    max_deg;       // maximum allowed angle (from Table 3.1)
    float    rest_deg;      // resting/safe position (from Table 3.1)
    bool     inverted;      // true if servo is mounted upside-down/mirrored
                            // inverted means 0° in code = 180° on physical servo
};

// ── Joint table ───────────────────────────────────────────
// Matches exactly your report Table 3.1, zero-indexed.
//
// About 'inverted':
// When you mount a servo on the left side of the robot as a
// mirror image of the right side, mechanically the same angle
// produces opposite rotation. Setting inverted=true flips the
// PWM calculation so your code can treat both sides identically.
// Start with all false, then flip any joints that move backwards.
//
// About the left knee asymmetry (rest: 70° vs right: 100°):
// This is in your original table. It may mean the left knee
// servo is mounted in the opposite orientation. Check physically
// and set inverted=true if needed, then set rest_deg to match.
//
const JointConfig JOINT_CONFIG[NUM_JOINTS] = {
    //  name                  board  ch   min    max    rest   inverted
    { "r_shoulder_pitch",     0,     0,   0.0f,  180.0f, 90.0f, false },  // 0
    { "r_shoulder_roll",      0,     1,   0.0f,  180.0f, 90.0f, false },  // 1
    { "r_elbow_roll",         0,     2,   0.0f,  180.0f, 90.0f, false },  // 2
    { "head_yaw",             0,     3,   0.0f,  180.0f, 90.0f, false },  // 3
    { "camera_pitch",         0,     4,   0.0f,  180.0f, 90.0f, false },  // 4
    { "r_hip_roll",           0,     5,   0.0f,  170.0f, 50.0f, false },  // 5
    { "r_hip_pitch",          0,     6,   0.0f,  170.0f, 80.0f, false },  // 6
    { "r_knee_pitch",         0,     7,   0.0f,  170.0f,100.0f, false },  // 7
    { "r_ankle_pitch",        0,     8,  10.0f,  180.0f,100.0f, false },  // 8
    { "r_ankle_roll",         0,     9,  20.0f,  150.0f, 90.0f, false },  // 9
    { "l_shoulder_pitch",     1,     0,   0.0f,  180.0f, 90.0f, false },  // 10
    { "l_shoulder_roll",      1,     1,   0.0f,  180.0f, 90.0f, false },  // 11
    { "l_elbow_roll",         1,     2,   0.0f,  180.0f, 90.0f, false },  // 12
    { "l_hip_roll",           1,     3,   0.0f,  170.0f, 50.0f, false },  // 13
    { "l_hip_pitch",          1,     4,   0.0f,  170.0f, 80.0f, false },  // 14
    { "l_knee_pitch",         1,     5,   0.0f,  170.0f, 70.0f, false },  // 15  ← note 70°
    { "l_ankle_pitch",        1,     6,  10.0f,  180.0f, 90.0f, false },  // 16
    { "l_ankle_roll",         1,     7,  20.0f,  150.0f, 90.0f, false },  // 17
};

// ── Convenience index names ───────────────────────────────
// Use these in code instead of magic numbers.
// e.g.: current_deg[JOINT_R_KNEE] instead of current_deg[7]
// Makes the code self-documenting.
enum JointIndex {
    JOINT_R_SHOULDER_PITCH  = 0,
    JOINT_R_SHOULDER_ROLL   = 1,
    JOINT_R_ELBOW_ROLL      = 2,
    JOINT_HEAD_YAW          = 3,
    JOINT_CAMERA_PITCH      = 4,
    JOINT_R_HIP_ROLL        = 5,
    JOINT_R_HIP_PITCH       = 6,
    JOINT_R_KNEE_PITCH      = 7,
    JOINT_R_ANKLE_PITCH     = 8,
    JOINT_R_ANKLE_ROLL      = 9,
    JOINT_L_SHOULDER_PITCH  = 10,
    JOINT_L_SHOULDER_ROLL   = 11,
    JOINT_L_ELBOW_ROLL      = 12,
    JOINT_L_HIP_ROLL        = 13,
    JOINT_L_HIP_PITCH       = 14,
    JOINT_L_KNEE_PITCH      = 15,
    JOINT_L_ANKLE_PITCH     = 16,
    JOINT_L_ANKLE_ROLL      = 17,
};