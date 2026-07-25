#!/usr/bin/env python3
"""test_joint_map.py — leg extraction (no ROS)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from humanoid_wbc.estimation.joint_map import legs_from_jointstate
checks = []
def ok(cond, label, got=None):
    checks.append(bool(cond))
    print(f"  [{'ok' if cond else 'XX'}] {label}" + ("" if got is None else f"  ({got})"))
names = ['head_yaw','r_hip_roll','l_shoulder_pitch','r_hip_pitch','r_knee_pitch',
         'r_ankle_pitch','r_ankle_roll','l_hip_roll','l_hip_pitch','l_knee_pitch',
         'l_ankle_pitch','l_ankle_roll','r_shoulder_pitch']
positions = [0.0,0.11,0.0,0.12,0.13,0.14,0.15,0.51,0.52,0.53,0.54,0.55,0.0]
res = legs_from_jointstate(names, positions)
ok(res is not None, "extracts when all leg joints present")
q_r, q_l = res
ok(q_r == [0.11,0.12,0.13,0.14,0.15], "right leg order", q_r)
ok(q_l == [0.51,0.52,0.53,0.54,0.55], "left leg order", q_l)
ok(legs_from_jointstate(['r_hip_roll'],[0.1]) is None, "None on incomplete msg")
ok_all = all(checks)
print(f"test_joint_map: {'PASS' if ok_all else 'FAIL'} ({sum(checks)}/{len(checks)})")
sys.exit(0 if ok_all else 1)
