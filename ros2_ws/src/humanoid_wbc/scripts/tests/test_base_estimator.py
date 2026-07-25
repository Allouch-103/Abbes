#!/usr/bin/env python3
"""test_base_estimator.py — contact-update KILLS IMU drift."""
import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from humanoid_wbc.estimation.imu_propagation import ImuPropagator, G
from humanoid_wbc.estimation.base_estimator import BaseEstimator
from humanoid_wbc.estimation.contact_schedule import Contacts
checks = []
def ok(cond, label, got=None):
    checks.append(bool(cond))
    print(f"  [{'ok' if cond else 'XX'}] {label}" + ("" if got is None else f"  ({got})"))
dt = 0.005; N = int(5.0/dt); BIAS = 0.1
straight = [0.0]*5; both_down = Contacts(True, True); TRUE_HEIGHT = 0.3555
bare = ImuPropagator()
for _ in range(N): bare.predict((0,0,0), (BIAS,0,G), dt)
drift = math.sqrt(bare.v[0]**2 + bare.v[1]**2)
ok(drift > 0.4, "bare IMU drifts (phantom velocity)", f"|v|={drift:.3f} m/s")
est = BaseEstimator(kv=0.5, kz=0.5)
for _ in range(N): est.step((0,0,0), (BIAS,0,G), dt, straight, straight, both_down)
vmag = math.sqrt(est.velocity[0]**2 + est.velocity[1]**2)
ok(vmag < 0.01, "contact-update HOLDS velocity near zero", f"|v|={vmag:.4f} m/s")
ok(vmag < drift/20, "estimator drift < 1/20 of bare drift", f"{vmag:.4f} vs {drift:.3f}")
ok(abs(est.position[2]-TRUE_HEIGHT) < 0.001, "base height recovered", f"z={est.position[2]:.4f} m")
ok(est.valid, "estimator reports valid")
ok_all = all(checks)
print(f"test_base_estimator: {'PASS' if ok_all else 'FAIL'} ({sum(checks)}/{len(checks)})")
sys.exit(0 if ok_all else 1)
