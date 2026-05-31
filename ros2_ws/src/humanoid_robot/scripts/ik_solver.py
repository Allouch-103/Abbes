#!/usr/bin/env python3
import numpy as np
import math

THIGH = 0.1225
SHANK = 0.1230

def solve_full_body_ik(com_traj, swing_traj, robot):
    N = len(com_traj)
    joint_traj = np.zeros((N, 18))
    hip_width = getattr(robot, 'hip_width', 0.08)

    for k in range(N):
        com_x = com_traj[k, 0]
        com_y = com_traj[k, 2]
        com_z = getattr(robot, 'Z_C', 0.2698)

        fr_x = swing_traj[k, 0]
        fr_y =  hip_width / 2.0
        fr_z = swing_traj[k, 2]

        fl_x = 0.0
        fl_y = -hip_width / 2.0
        fl_z = 0.0

        angles = [90.0] * 18

        def leg_ik(hip_pos, foot_pos):
            dx = foot_pos[0] - hip_pos[0]
            dy = foot_pos[1] - hip_pos[1]
            dz = foot_pos[2] - hip_pos[2]
            d  = math.sqrt(dx**2 + dy**2 + dz**2)
            d  = min(d, THIGH + SHANK - 0.001)
            cos_knee = (THIGH**2 + SHANK**2 - d**2) / (2*THIGH*SHANK)
            cos_knee = max(-1.0, min(1.0, cos_knee))
            knee = math.pi - math.acos(cos_knee)
            alpha = math.atan2(-dz, math.sqrt(dx**2 + dy**2))
            beta  = math.asin(max(-1.0, min(1.0, SHANK*math.sin(knee)/d)))
            hip_pitch  = alpha + beta
            hip_roll   = math.atan2(dy, -dz)
            ankle_pitch = -(hip_pitch + knee)
            return hip_roll, hip_pitch, knee, ankle_pitch, 0.0

        R2D = 180.0 / math.pi

        hip_r = [com_x, com_y + hip_width/2, com_z]
        hr, hp, kp, ap, ar = leg_ik(hip_r, [fr_x, fr_y, fr_z])
        angles[5]  = 50.0  + hr*R2D
        angles[6]  = 80.0  + hp*R2D
        angles[7]  = 100.0 - kp*R2D
        angles[8]  = 100.0 + ap*R2D
        angles[9]  = 90.0  + ar*R2D

        hip_l = [com_x, com_y - hip_width/2, com_z]
        hr, hp, kp, ap, ar = leg_ik(hip_l, [fl_x, fl_y, fl_z])
        angles[13] = 50.0  - hr*R2D
        angles[14] = 80.0  + hp*R2D
        angles[15] = 100.0 - kp*R2D
        angles[16] = 90.0  + ap*R2D
        angles[17] = 90.0  - ar*R2D

        joint_traj[k, :] = angles

    return joint_traj
