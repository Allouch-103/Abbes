#!/usr/bin/env python3
"""leg_kinematics.py — forward kinematics of one leg (Phase 1).
Foot position relative to the torso from the 5 leg joint angles. The estimator
needs this: a planted foot's position under the torso is an independent fix on
how the torso moves (the InEKF correct step). Pure Python; Phase 2 swaps in
Pinocchio and validates against this.
q = [hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll] (rad)."""
import math
THIGH_LEN = 0.1225
SHANK_LEN = 0.1230
FOOT_HEIGHT = 0.0500
TORSO_HEIGHT = 0.1200
HIP_OFFSET_Y = 0.0400

def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1,0,0),(0,c,-s),(0,s,c))
def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return ((c,0,s),(0,1,0),(-s,0,c))
def _matmul(A, B):
    return tuple(tuple(sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)) for i in range(3))
def _matvec(A, v):
    return tuple(sum(A[i][k]*v[k] for k in range(3)) for i in range(3))
def _vadd(a, b):
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2])

def foot_position(side, q, sole=False):
    """side 'r'/'l'; returns (x,y,z) of the foot in the TORSO frame (m)."""
    sign = -1.0 if side == 'r' else 1.0
    R = ((1,0,0),(0,1,0),(0,0,1))
    t = (0.0, sign*HIP_OFFSET_Y, -TORSO_HEIGHT/2.0)
    def step(Rj, tj):
        nonlocal R, t
        t = _vadd(t, _matvec(R, tj)); R = _matmul(R, Rj)
    step(_Rx(q[0]), (0,0,0))
    step(_Ry(q[1]), (0,0,0))
    step(_Ry(q[2]), (0,0,-THIGH_LEN))
    step(_Ry(q[3]), (0,0,-SHANK_LEN))
    step(_Rx(q[4]), (0,0,0))
    if sole:
        t = _vadd(t, _matvec(R, (0,0,-FOOT_HEIGHT)))
    return t
