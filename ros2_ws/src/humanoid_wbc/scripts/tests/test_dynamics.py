#!/usr/bin/env python3
"""test_dynamics.py — Phase-2 acceptance: cross-validate Pinocchio against the
independent Phase-0 analytic model (estimate_model.py). Two methods agreeing is
stronger than either alone."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from humanoid_wbc.dynamics.robot_dyn import RobotDyn
import pinocchio as pin

ANALYTIC_MASS = 1.563
ANALYTIC_COM_Z = 0.2678
checks = []
def ok(cond, label, got=None):
    checks.append(bool(cond))
    print(f"  [{'ok' if cond else 'XX'}] {label}" + ("" if got is None else f"  ({got})"))

rd = RobotDyn()
print(f"  model: nq={rd.model.nq} nv={rd.model.nv} (floating base + {rd.model.nv-6} joints)")
ok(rd.model.nv-6 == 18, "18 actuated joints + floating base", f"nv={rd.model.nv}")

m = rd.mass; err = abs(m-ANALYTIC_MASS)/ANALYTIC_MASS*100
ok(err < 1.0, "total mass matches analytic model (<1%)",
   f"pinocchio {m:.4f} kg vs analytic {ANALYTIC_MASS} kg -> {err:.2f}%")

q0 = rd.neutral(); c = rd.com(q0); dz = abs(c[2]-ANALYTIC_COM_Z)*1000
ok(dz < 5.0, "home CoM height matches analytic (<5 mm)",
   f"pinocchio {c[2]*1000:.1f} mm vs analytic {ANALYTIC_COM_Z*1000:.1f} mm -> {dz:.1f} mm")
ok(abs(c[1]) < 1e-9, "home CoM dead-centre in y (L/R symmetry)", f"y={c[1]:.2e}")
ok(abs(c[0]) < 0.001, "home CoM x-offset < 1 mm (camera on front of head)",
   f"x={c[0]*1000:.2f} mm")

rng = np.random.default_rng(0); worst = 0.0
for _ in range(5):
    q = rd.neutral(); q[7:] = rng.uniform(-0.3, 0.3, rd.model.nq-7)
    v = np.zeros(rd.model.nv); v[6:] = rng.uniform(-0.5, 0.5, rd.model.nv-6)
    h = rd.centroidal_momentum(q, v)
    dt = 1e-6
    q_next = pin.integrate(rd.model, q, v*dt)
    h_lin_fd = rd.mass * (rd.com(q_next) - rd.com(q)) / dt
    denom = max(np.linalg.norm(h[:3]), 1e-9)
    worst = max(worst, np.linalg.norm(h[:3]-h_lin_fd)/denom*100)
ok(worst < 2.0, "centroidal momentum == m*d(CoM)/dt, 5 random states (<2%)", f"worst {worst:.3f}%")

g = rd.gravity_torque(q0); w = rd.mass*9.81
err_g = abs(abs(g[2])-w)/w*100
ok(err_g < 1.0, "gravity torque on base_z == total weight (<1%)",
   f"{abs(g[2]):.4f} N vs {w:.4f} N -> {err_g:.2f}%")

ok_all = all(checks)
print(f"test_dynamics: {'PASS' if ok_all else 'FAIL'} ({sum(checks)}/{len(checks)})")
sys.exit(0 if ok_all else 1)
