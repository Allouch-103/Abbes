"""
pose_definitions.py
===================

Named joint poses for the humanoid robot, plus helpers that convert a
pose into the three Float64MultiArray payloads expected by the
ros2_control forward controllers.

Conventions
-----------
- All angles are stored in RADIANS.
- All angles are in URDF convention: joint = 0 means the geometric
  reference pose (straight legs, arms hanging down, head forward).
- Joint orderings within each controller MUST match controllers.yaml
  in my_robot_bringup. If you change controllers.yaml, change the lists
  below to match — they are the single source of truth for this layer.

This module is pure Python (no rclpy imports) so it can be unit-tested
in isolation. ROS message construction lives in the action server.
"""

import math


# ─────────────────────────────────────────────────────────────
#  Joint orderings — must match controllers.yaml exactly
# ─────────────────────────────────────────────────────────────

LEG_JOINTS = [
    "r_hip_roll",
    "r_hip_pitch",
    "r_knee_pitch",
    "r_ankle_pitch",
    "r_ankle_roll",
    "l_hip_roll",
    "l_hip_pitch",
    "l_knee_pitch",
    "l_ankle_pitch",
    "l_ankle_roll",
]

ARM_JOINTS = [
    "r_shoulder_pitch",
    "r_shoulder_roll",
    "r_elbow_roll",
    "l_shoulder_pitch",
    "l_shoulder_roll",
    "l_elbow_roll",
]

HEAD_JOINTS = [
    "head_yaw",
    "camera_pitch",
]

ALL_JOINTS = LEG_JOINTS + ARM_JOINTS + HEAD_JOINTS   # 10 + 6 + 2 = 18


# ─────────────────────────────────────────────────────────────
#  Small helpers
# ─────────────────────────────────────────────────────────────

def deg(d: float) -> float:
    """Shorthand: degrees to radians, makes pose definitions readable."""
    return math.radians(d)


def _pose_with_overrides(**overrides) -> dict:
    """Return HOME_POSE with the given joints overridden.

    Example:
        _pose_with_overrides(r_hip_pitch=deg(-30))
        → all zeros, except r_hip_pitch = -0.524 rad
    """
    pose = {j: 0.0 for j in ALL_JOINTS}
    for joint, angle in overrides.items():
        if joint not in pose:
            raise KeyError(f"unknown joint: {joint!r}")
        pose[joint] = angle
    return pose


# ─────────────────────────────────────────────────────────────
#  Named poses
# ─────────────────────────────────────────────────────────────

# Home — all joints at URDF zero. Robot stands straight, arms down,
# head forward. This is the "reset" pose for everything.
HOME_POSE = {j: 0.0 for j in ALL_JOINTS}


# Sit — knees bent strongly + hips pitched back so CoM drops.
# Ankle pitch compensates to keep the foot flat-ish on the ground.
# Geometry: hip_pitch + knee_pitch + ankle_pitch = 0 keeps the shank
# vertical and the foot flat. Here we lean the torso back slightly
# so the robot sits rather than squats forward.
SIT_POSE = _pose_with_overrides(
    r_hip_pitch=deg(-60), r_knee_pitch=deg(120), r_ankle_pitch=deg(-60),
    l_hip_pitch=deg(-60), l_knee_pitch=deg(120), l_ankle_pitch=deg(-60),
)


# Bow — torso forward (hip pitch -30°), small knee bend for balance,
# small ankle pitch so the shank stays roughly vertical.
BOW_POSE = _pose_with_overrides(
    r_hip_pitch=deg(-30), r_knee_pitch=deg(20), r_ankle_pitch=deg(-10),
    l_hip_pitch=deg(-30), l_knee_pitch=deg(20), l_ankle_pitch=deg(-10),
)


# Wave-right — arm raised forward and slightly out, elbow bent.
# Sign convention for the LEFT elbow may be inverted relative to right
# (mirror-mounted). Verify visually in Gazebo and flip if needed.
WAVE_RIGHT_UP = _pose_with_overrides(
    r_shoulder_pitch=deg(0),
    r_shoulder_roll=deg(-90),
    r_elbow_roll=deg(0),
)
WAVE_RIGHT_OUT = _pose_with_overrides(
    r_shoulder_pitch=deg(0),
    r_shoulder_roll=deg(-90),
    r_elbow_roll=deg(-180),
)
WAVE_LEFT_UP = _pose_with_overrides(
    l_shoulder_pitch=deg(0),
    l_shoulder_roll=deg(90),
    l_elbow_roll=deg(0),
)
WAVE_LEFT_OUT = _pose_with_overrides(
    l_shoulder_pitch=deg(0),
    l_shoulder_roll=deg(90),
    l_elbow_roll=deg(180),
)


# March-step poses — lift one leg while keeping the other planted.
# Used by the Walk stub until the real ZMP gait generator is online.
# We do NOT shift weight laterally here, so on a balanced biped this
# will be unstable. For Gazebo with the robot floating or on a stable
# base it visually looks like marching in place.
MARCH_R_UP = _pose_with_overrides(
    r_hip_pitch=deg(30), r_knee_pitch=deg(30), r_ankle_pitch=deg(30),
)
MARCH_L_UP = _pose_with_overrides(
    l_hip_pitch=deg(30), l_knee_pitch=deg(30), l_ankle_pitch=deg(30),
)


# ─────────────────────────────────────────────────────────────
#  Conversion helpers — pose dict → controller arrays
# ─────────────────────────────────────────────────────────────

def pose_to_arrays(pose: dict) -> tuple[list, list, list]:
    """Convert a pose dict into three lists matching controller orderings.

    Returns:
        (leg_angles, arm_angles, head_angles)
        — each is a list of floats, in the order each controller expects.

    The caller wraps these in Float64MultiArray and publishes:

        leg_arr, arm_arr, head_arr = pose_to_arrays(pose)
        leg_pub.publish(Float64MultiArray(data=leg_arr))
        ...

    Raises:
        KeyError if pose is missing any joint listed in ALL_JOINTS.
    """
    leg = [pose[j] for j in LEG_JOINTS]
    arm = [pose[j] for j in ARM_JOINTS]
    head = [pose[j] for j in HEAD_JOINTS]
    return leg, arm, head
