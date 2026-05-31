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
    uint8_t  pca_board;     // 0 = board at 0x40 (only board 0 is wired now)
    uint8_t  pca_channel;   // 0-15 on that board
    float    min_deg;       // minimum allowed angle (from Table 3.1)
    float    max_deg;       // maximum allowed angle (from Table 3.1)
    float    rest_deg;      // resting/safe position (from Table 3.1)
    bool     inverted;      // true if servo is mounted upside-down/mirrored
                            // inverted means 0° in code = 180° on physical servo
    bool     connected;     // false = no physical servo for this joint.
                            // We still keep the 18-joint /joint_commands
                            // contract with ROS2, but skip driving any joint
                            // that has no servo (servo_set() returns early).
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
// ── ONE PCA9685 (16 channels) — channel assignment ───────────────────────
// Only board 0 (0x40) is wired. 18 logical joints, but a single board has
// only 16 channels, so 2 joints have no servo. Priority (per build):
//   LEGS  (10) → channels 0–9
//   ARMS  (6)  → channels 10–15
//   HEAD_YAW + CAMERA_PITCH → NOT connected (no channel left)
//
// The array INDEX (the // N comment) is the /joint_commands index and must
// NOT change — it is the ROS2 contract. Only pca_channel / connected change.
//
// rest_deg is the BOOT/neutral position. Set to 90° (servo center) for ALL
// joints so it matches the ROS2 bridge's runtime default (CALIBRATION rest=90)
// — the robot powers on centered and the first /joint_commands causes no jump.
// min_deg/max_deg are kept as the per-joint safety clamp (recalibrate on mount).
const JointConfig JOINT_CONFIG[NUM_JOINTS] = {
    //  name                  board  ch   min    max    rest   inverted connected
    { "r_shoulder_pitch",     0,    10,   0.0f,  180.0f, 90.0f, false,  true  },  // 0
    { "r_shoulder_roll",      0,    11,   0.0f,  180.0f, 90.0f, false,  true  },  // 1
    { "r_elbow_roll",         0,    12,   0.0f,  180.0f, 90.0f, false,  true  },  // 2
    { "head_yaw",             0,   255,   0.0f,  180.0f, 90.0f, false,  false },  // 3  (no servo)
    { "camera_pitch",         0,   255,   0.0f,  180.0f, 90.0f, false,  false },  // 4  (no servo)
    { "r_hip_roll",           0,     0,   0.0f,  170.0f, 90.0f, false,  true  },  // 5
    { "r_hip_pitch",          0,     1,   0.0f,  170.0f, 90.0f, false,  true  },  // 6
    { "r_knee_pitch",         0,     2,   0.0f,  170.0f, 90.0f, false,  true  },  // 7
    { "r_ankle_pitch",        0,     3,  10.0f,  180.0f, 90.0f, false,  true  },  // 8
    { "r_ankle_roll",         0,     4,  20.0f,  150.0f, 90.0f, false,  true  },  // 9
    { "l_shoulder_pitch",     0,    13,   0.0f,  180.0f, 90.0f, false,  true  },  // 10
    { "l_shoulder_roll",      0,    14,   0.0f,  180.0f, 90.0f, false,  true  },  // 11
    { "l_elbow_roll",         0,    15,   0.0f,  180.0f, 90.0f, false,  true  },  // 12
    { "l_hip_roll",           0,     5,   0.0f,  170.0f, 90.0f, false,  true  },  // 13
    { "l_hip_pitch",          0,     6,   0.0f,  170.0f, 90.0f, false,  true  },  // 14
    { "l_knee_pitch",         0,     7,   0.0f,  170.0f, 90.0f, false,  true  },  // 15
    { "l_ankle_pitch",        0,     8,  10.0f,  180.0f, 90.0f, false,  true  },  // 16
    { "l_ankle_roll",         0,     9,  20.0f,  150.0f, 90.0f, false,  true  },  // 17
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