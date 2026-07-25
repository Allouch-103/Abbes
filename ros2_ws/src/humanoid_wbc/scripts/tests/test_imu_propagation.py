#!/usr/bin/env python3
"""test_imu_propagation.py — unit test for the InEKF predict step."""
import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from humanoid_wbc.estimation.imu_propagation import ImuPropagator, quat_to_euler, G
checks = []
def ok(cond, label, got=None):
    checks.append(bool(cond))
    print(f"  [{'ok' if cond else 'XX'}] {label}" + ("" if got is None else f"  ({got})"))
f = ImuPropagator(); dt = 0.005
for _ in range(2000): f.predict((0,0,0), (0,0,G), dt)
dv = math.sqrt(sum(c*c for c in f.v)); dp = math.sqrt(sum(c*c for c in f.p))
ok(dv < 1e-6, "at rest: velocity drift < 1e-6 m/s", f"{dv:.2e}")
ok(dp < 1e-6, "at rest: position drift < 1e-6 m", f"{dp:.2e}")
ok(f.valid, "becomes valid after warmup")
f = ImuPropagator(); w = 0.5
for _ in range(200): f.predict((0,0,w), (0,0,G), dt)
_,_,yaw = quat_to_euler(f.q)
ok(abs(yaw-0.5) < 1e-3, "yaw integrates to 0.5 rad", f"yaw={yaw:.4f}")
f = ImuPropagator()
ok(not f.predict((0,0,0), (50.0,50.0,50.0), dt), "garbage sample REJECTED")
ok(f.rejected == 1 and f.q == (1.0,0.0,0.0,0.0), "state untouched", f"rejected={f.rejected}")
ok(f.predict((0,0,0), (0,0,-G), dt), "plausible sample accepted")
ok_all = all(checks)
print(f"test_imu_propagation: {'PASS' if ok_all else 'FAIL'} ({sum(checks)}/{len(checks)})")
sys.exit(0 if ok_all else 1)
