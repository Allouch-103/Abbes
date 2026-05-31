#!/usr/bin/env python3
"""
zmp_reference.py
────────────────
Stage 2: convert footstep list → sampled ZMP reference array.

Rules:
  • Single-support:  ZMP = stance foot position (x,y)
  • Double-support:  ZMP linearly interpolates between outgoing and
                     incoming foot positions (smooth CoM deceleration)

The output is a numpy array of shape (N, 2) pre-allocated for the full
motion, so the preview controller can index any future sample in O(1).

Reference: Modern Robotics Ch. 9; Sciavicco §3.1 (trajectory continuity).
"""

import numpy as np
from typing import List
from footstep_planner import Footstep


def build_zmp_reference(
    footsteps: List[Footstep],
    dt: float = 0.005,
    T_total: float = None,
) -> np.ndarray:
    """
    Parameters
    ----------
    footsteps : list of Footstep
    dt        : controller sample period (s)
    T_total   : total horizon (s); defaults to last lift-off + 1 s tail

    Returns
    -------
    zmp_ref : (N, 2)  columns = [x_zmp, y_zmp]
    """
    if T_total is None:
        T_total = footsteps[-1].t_lift + 1.0   # trailing still-stance period
    N = int(np.ceil(T_total / dt))
    zmp_ref = np.zeros((N, 2))

    for k in range(N):
        t = k * dt

        # ── single-support: ZMP at current stance foot ──────────────────
        active = next((fs for fs in footsteps
                       if fs.t_start <= t <= fs.t_lift), None)
        if active is not None:
            zmp_ref[k] = active.pos[:2]
            continue

        # ── double-support: linear interpolation ────────────────────────
        # foot that just lifted off
        prev = next((fs for fs in reversed(footsteps)
                     if fs.t_lift < t), footsteps[0])
        # foot about to land
        nxt  = next((fs for fs in footsteps
                     if fs.t_start > t), footsteps[-1])

        ds_len = max(nxt.t_start - prev.t_lift, 1e-9)
        alpha  = (t - prev.t_lift) / ds_len               # ∈ [0, 1]
        zmp_ref[k] = (1 - alpha) * prev.pos[:2] + alpha * nxt.pos[:2]

    return zmp_ref


if __name__ == "__main__":
    from footstep_planner import generate_footsteps
    import matplotlib.pyplot as plt

    steps = generate_footsteps(n_steps=6)
    ref   = build_zmp_reference(steps, dt=0.005)
    t     = np.arange(len(ref)) * 0.005

    plt.figure(figsize=(10, 3))
    plt.plot(t, ref[:, 0], label="ZMP x (fwd)")
    plt.plot(t, ref[:, 1], label="ZMP y (lat)")
    plt.xlabel("time (s)"); plt.ylabel("ZMP (m)")
    plt.legend(); plt.title("ZMP reference trajectory")
    plt.tight_layout(); plt.savefig("zmp_reference.png", dpi=120)
    print("Saved zmp_reference.png")
