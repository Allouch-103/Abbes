#!/usr/bin/env python3
"""
preview_controller.py
─────────────────────
Stage 3: Kajita (2003) preview controller for the Linear Inverted Pendulum.

Theory summary
──────────────
State:   x = [p, ṗ, p̈]ᵀ   (position, velocity, acceleration of CoM)
Input:   u = p⃛             (jerk — the control variable)
Output:  p_zmp = p - (z_c/g)·p̈   (ZMP from Newton-Euler on the LIPM)

Discrete state-space:
    x[k+1] = A·x[k] + B·u[k]
    p_zmp  = C·x[k]

Augmented system adds a ZMP-error integral state e[k]:
    e[k+1] = e[k] + (C·x[k] − p_ref[k])
    x_aug  = [e, p, ṗ, p̈]ᵀ

Control law (Kajita eq. 16):
    u[k] = −G_I·e[k] − G_x·x[k] − Σ_{i=0}^{N} G_p[i]·p_ref[k+i]

G_I, G_x are optimal feedback gains from DARE.
G_p[i] is the preview gain for a reference i steps ahead.

References
──────────
  Kajita et al. (2003) "Biped Walking Pattern Generation …"
  Modern Robotics Ch. 9
  Sciavicco §3.1, §6
"""

import numpy as np
from scipy.linalg import solve_discrete_are
from typing import Tuple

Gains = Tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]


def compute_preview_gains(
    dt: float,
    z_c: float,
    g: float = 9.81,
    Q: float = 1.0,
    R: float = 1e-6,
    N_preview: int = 300,
) -> Gains:
    """
    Compute LQR + preview gains offline.

    Returns: G_I, G_x (3,), G_p (N_preview,), A, B, C
    """
    # ── Discrete LIPM ────────────────────────────────────────────────────
    A = np.array([[1.0, dt, dt**2/2],
                  [0.0, 1.0, dt    ],
                  [0.0, 0.0, 1.0  ]])
    B = np.array([[dt**3/6], [dt**2/2], [dt]])
    C = np.array([[1.0, 0.0, -z_c/g]])   # ZMP output: p - (z_c/g)·p̈

    # ── Augmented system: x_aug = [e, p, ṗ, p̈] ─────────────────────────
    # e[k+1] = e[k] + C·x[k]   (reference subtracted online in control loop)
    # x[k+1] = A·x[k] + B·u[k]
    Aaug = np.block([[np.ones((1,1)), C      ],
                     [np.zeros((3,1)), A     ]])   # (4×4)
    Baug = np.vstack([np.zeros((1,1)), B])         # (4×1)
    Qaug = np.diag([Q, 0.0, 0.0, 0.0])

    # ── DARE → optimal gain ───────────────────────────────────────────────
    P    = solve_discrete_are(Aaug, Baug, Qaug, np.array([[R]]))
    BtPB = float((Baug.T @ P @ Baug).ravel()[0]) + R
    K    = ((1.0 / BtPB) * (Baug.T @ P @ Aaug)).ravel()  # (4,)
    G_I  = float(K[0])
    G_x  = K[1:]   # (3,)

    # ── Preview gains via closed-loop power iteration ─────────────────────
    Acl = Aaug - Baug @ K.reshape(1, -1)
    tmp = -(Acl.T) @ P @ np.array([[1.0], [0.0], [0.0], [0.0]])
    G_p = np.zeros(N_preview)
    for i in range(N_preview):
        G_p[i] = float((1.0 / BtPB) * (Baug.T @ tmp).ravel()[0])
        tmp = Acl.T @ tmp

    return G_I, G_x, G_p, A, B, C


def run_preview_controller(
    zmp_ref: np.ndarray,
    G_I: float,
    G_x: np.ndarray,
    G_p: np.ndarray,
    A: np.ndarray,
    B: np.ndarray,
    C: np.ndarray,
    dt: float,
) -> np.ndarray:
    """
    Simulate preview controller over full ZMP reference.

    Returns com_traj: (N, 6) — [x, ẋ, ẍ, y, ẏ, ÿ]

    Sign convention (Kajita 2003):
      e[k] += zmp_actual[k] - p_ref[k]   (integral of ZMP error)
      u[k]  = -G_I·e[k] - G_x·x[k] - G_p·preview[k:k+Np]
    """
    N    = len(zmp_ref)
    Np   = len(G_p)
    x_st = np.zeros(3)
    y_st = np.zeros(3)
    e_x  = 0.0
    e_y  = 0.0
    Bflat = B.flatten()
    com_traj = np.empty((N, 6))

    for k in range(N):
        zmp_x = float((C @ x_st).ravel()[0])
        zmp_y = float((C @ y_st).ravel()[0])

        # Error integral: accumulate (actual - reference), matches Kajita sign
        e_x += zmp_x - zmp_ref[k, 0]
        e_y += zmp_y - zmp_ref[k, 1]

        end  = min(k + Np, N)
        px   = zmp_ref[k:end, 0]
        py   = zmp_ref[k:end, 1]
        if len(px) < Np:
            px = np.pad(px, (0, Np - len(px)), mode='edge')
            py = np.pad(py, (0, Np - len(py)), mode='edge')

        u_x = -G_I * e_x - G_x @ x_st - G_p @ px
        u_y = -G_I * e_y - G_x @ y_st - G_p @ py

        x_st = A @ x_st + Bflat * u_x
        y_st = A @ y_st + Bflat * u_y
        com_traj[k] = np.concatenate([x_st, y_st])

    return com_traj


if __name__ == "__main__":
    from footstep_planner import generate_footsteps
    from zmp_reference     import build_zmp_reference
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    DT, ZC = 0.005, 0.30
    steps   = generate_footsteps(n_steps=8)
    zmp_ref = build_zmp_reference(steps, dt=DT)
    gains   = compute_preview_gains(DT, ZC)
    G_I, G_x, G_p, A, B, C = gains
    com     = run_preview_controller(zmp_ref, G_I, G_x, G_p, A, B, C, DT)

    t  = np.arange(len(com)) * DT
    zmp_actual_x = np.array([float((C @ com[k,:3]).ravel()[0]) for k in range(len(com))])

    plt.figure(figsize=(10, 4))
    plt.plot(t, zmp_ref[:len(com), 0], '--', alpha=0.7, label='ZMP ref x')
    plt.plot(t, zmp_actual_x, label='ZMP actual x')
    plt.plot(t, com[:, 0],    label='CoM x')
    plt.xlabel('time (s)'); plt.ylabel('m')
    plt.legend(); plt.title('Preview controller — ZMP tracking')
    plt.tight_layout(); plt.savefig('preview_result.png', dpi=120)
    print('Saved preview_result.png')
    err = np.max(np.abs(zmp_actual_x - zmp_ref[:len(com), 0]))
    print(f'Max ZMP tracking error: {err*1000:.3f} mm')
