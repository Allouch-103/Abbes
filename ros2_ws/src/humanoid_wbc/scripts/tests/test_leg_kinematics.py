#!/usr/bin/env python3
"""test_leg_kinematics.py — unit test for leg FK (pure Python)."""
import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from humanoid_wbc.estimation.leg_kinematics import (
    foot_position, THIGH_LEN, SHANK_LEN, FOOT_HEIGHT, TORSO_HEIGHT, HIP_OFFSET_Y)
checks = []
def ok(cond, label, got=None):
    checks.append(bool(cond))
    print(f"  [{'ok' if cond else 'XX'}] {label}" + ("" if got is None else f"  ({got})"))
def close(a, b, tol=1e-9): return all(abs(a[i]-b[i]) < tol for i in range(3))
zc = -(TORSO_HEIGHT/2 + THIGH_LEN + SHANK_LEN)
r = foot_position('r', [0,0,0,0,0]); l = foot_position('l', [0,0,0,0,0])
ok(close(r, (0.0,-HIP_OFFSET_Y,zc)), "right foot home pose", tuple(round(v,4) for v in r))
ok(close(l, (0.0, HIP_OFFSET_Y,zc)), "left foot home pose",  tuple(round(v,4) for v in l))
rs = foot_position('r', [0,0,0,0,0], sole=True)
ok(abs(rs[2]-(zc-FOOT_HEIGHT)) < 1e-9, "sole is foot_height below ankle", round(rs[2],4))
bent = foot_position('r', [0,0.3,-0.6,0.3,0])
ok(bent[2] > zc+1e-3, "knee flex raises the foot", round(bent[2],4))
hip = (0.0,-HIP_OFFSET_Y,-TORSO_HEIGHT/2)
rolled = foot_position('r', [0.4,0,0,0,0])
def dist(a,b): return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))
ok(abs(rolled[1]-(-HIP_OFFSET_Y)) > 1e-3, "hip roll moves foot in y", round(rolled[1],4))
ok(abs(dist(r,hip)-dist(rolled,hip)) < 1e-9, "leg length from hip preserved", round(dist(rolled,hip),4))
ok_all = all(checks)
print(f"test_leg_kinematics: {'PASS' if ok_all else 'FAIL'} ({sum(checks)}/{len(checks)})")
sys.exit(0 if ok_all else 1)
