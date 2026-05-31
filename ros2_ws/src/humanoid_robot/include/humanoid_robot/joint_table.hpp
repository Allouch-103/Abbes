#pragma once
#include <array>
#include <algorithm>

// Joint indices — ORDER MUST MATCH ros2_control block in humanoid.urdf.xacro
namespace J {
constexpr int R_HIP_ROLL       = 0;
constexpr int R_HIP_PITCH      = 1;
constexpr int R_KNEE_PITCH     = 2;
constexpr int R_ANKLE_PITCH    = 3;
constexpr int R_ANKLE_ROLL     = 4;
constexpr int L_HIP_ROLL       = 5;
constexpr int L_HIP_PITCH      = 6;
constexpr int L_KNEE_PITCH     = 7;
constexpr int L_ANKLE_PITCH    = 8;
constexpr int L_ANKLE_ROLL     = 9;
constexpr int R_SHOULDER_PITCH = 10;
constexpr int R_SHOULDER_ROLL  = 11;
constexpr int R_ELBOW_ROLL     = 12;
constexpr int L_SHOULDER_PITCH = 13;
constexpr int L_SHOULDER_ROLL  = 14;
constexpr int L_ELBOW_ROLL     = 15;
constexpr int HEAD_YAW         = 16;
constexpr int CAMERA_PITCH     = 17;
constexpr int COUNT            = 18;
}

struct JointLimit { float min, max, rest; };
// rest = 90 for ALL joints: URDF-zero (straight/neutral) maps to servo 90,
// the mechanical centre of a 0..180 servo. Matches firmware joint_definitions.h
// and the servo_bridge convention. min/max stay as the per-joint mechanical
// safety clamp (recalibrate on the mounted robot).
constexpr std::array<JointLimit, J::COUNT> LIMITS = {{
  //  min    max   rest    index  name
  {   0,   170,  90 },  //  0   r_hip_roll
  {   0,   170,  90 },  //  1   r_hip_pitch
  {   0,   170,  90 },  //  2   r_knee_pitch
  {  10,   180,  90 },  //  3   r_ankle_pitch
  {  20,   150,  90 },  //  4   r_ankle_roll
  {   0,   170,  90 },  //  5   l_hip_roll
  {   0,   170,  90 },  //  6   l_hip_pitch
  {   0,   170,  90 },  //  7   l_knee_pitch
  {  10,   180,  90 },  //  8   l_ankle_pitch
  {  20,   150,  90 },  //  9   l_ankle_roll
  {   0,   180,  90 },  // 10   r_shoulder_pitch
  {   0,   180,  90 },  // 11   r_shoulder_roll
  {   0,   180,  90 },  // 12   r_elbow_roll
  {   0,   180,  90 },  // 13   l_shoulder_pitch
  {   0,   180,  90 },  // 14   l_shoulder_roll
  {   0,   180,  90 },  // 15   l_elbow_roll
  {   0,   180,  90 },  // 16   head_yaw
  {   0,   180,  90 },  // 17   camera_pitch
}};

inline float clamp_joint(int idx, double deg) {
  return static_cast<float>(
    std::clamp(deg,
      static_cast<double>(LIMITS[idx].min),
      static_cast<double>(LIMITS[idx].max)));
}
