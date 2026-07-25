#!/usr/bin/env python3
"""joint_map.py — pull the two 5-DOF leg vectors from a /joint_states message.
ROS-free so it is unit-testable. Order matches leg_kinematics.foot_position():
[hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll]."""
R_JOINTS = ['r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll']
L_JOINTS = ['l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll']

def legs_from_jointstate(names, positions):
    idx = {n: i for i, n in enumerate(names)}
    try:
        q_r = [float(positions[idx[n]]) for n in R_JOINTS]
        q_l = [float(positions[idx[n]]) for n in L_JOINTS]
    except (KeyError, IndexError):
        return None
    return q_r, q_l
