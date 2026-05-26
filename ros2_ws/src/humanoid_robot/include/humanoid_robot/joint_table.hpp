#pragma once
// ============================================================================
//  joint_table.hpp
//  Single source of truth for joint indices, servo limits, and rest positions.
//
//  Include this anywhere joint data is needed — IK, balance controller,
//  gait node — without pulling in any ROS headers.
// ============================================================================

#include <array>
#include <algorithm>

// ── Joint indices (Table 3.1) ─────────────────────────────────────────────────
namespace J {
  constexpr int R_SHOULDER_PITCH = 0;
  constexpr int R_SHOULDER_ROLL  = 1;
  constexpr int R_ELBOW_ROLL     = 2;
  constexpr int HEAD_YAW         = 3;
  constexpr int CAMERA_PITCH     = 4;
  constexpr int R_HIP_ROLL       = 5;
  constexpr int R_HIP_PITCH      = 6;
  constexpr int R_KNEE_PITCH     = 7;
  constexpr int R_ANKLE_PITCH    = 8;
  constexpr int R_ANKLE_ROLL     = 9;
  constexpr int L_SHOULDER_PITCH = 10;
  constexpr int L_SHOULDER_ROLL  = 11;
  constexpr int L_ELBOW_ROLL     = 12;
  constexpr int L_HIP_ROLL       = 13;
  constexpr int L_HIP_PITCH      = 14;
  constexpr int L_KNEE_PITCH     = 15;
  constexpr int L_ANKLE_PITCH    = 16;
  constexpr int L_ANKLE_ROLL     = 17;
  constexpr int COUNT            = 18;
}

// ── Per-joint servo limits and rest position [degrees] ───────────────────────
struct JointLimit { float min, max, rest; };

constexpr std::array<JointLimit, J::COUNT> LIMITS = {{
  //  min    max   rest    index  name
  {   0,   180,  90 },  //  0   r_shoulder_pitch
  {   0,   180,  90 },  //  1   r_shoulder_roll
  {   0,   180,  90 },  //  2   r_elbow_roll
  {   0,   180,  90 },  //  3   head_yaw
  {   0,   180,  90 },  //  4   camera_pitch
  {   0,   170,  50 },  //  5   r_hip_roll
  {   0,   170,  80 },  //  6   r_hip_pitch
  {   0,   170, 100 },  //  7   r_knee_pitch
  {  10,   180, 100 },  //  8   r_ankle_pitch
  {  20,   150,  90 },  //  9   r_ankle_roll
  {   0,   180,  90 },  // 10   l_shoulder_pitch
  {   0,   180,  90 },  // 11   l_shoulder_roll
  {   0,   180,  90 },  // 12   l_elbow_roll
  {   0,   170,  50 },  // 13   l_hip_roll
  {   0,   170,  80 },  // 14   l_hip_pitch
  {   0,   170,  70 },  // 15   l_knee_pitch
  {  10,   180,  90 },  // 16   l_ankle_pitch
  {  20,   150,  90 },  // 17   l_ankle_roll
}};

// Clamp a computed servo angle (degrees) to its physical range.
inline float clamp_joint(int idx, double deg) {
    return static_cast<float>(
        std::clamp(deg,
                   static_cast<double>(LIMITS[idx].min),
                   static_cast<double>(LIMITS[idx].max)));
}