#!/usr/bin/env python3
"""
footstep_planner.py
───────────────────
Stage 1: generate a timed sequence of L/R foot placements.

Design:  Keeps footstep geometry completely separate from dynamics.
         Downstream modules only consume the Footstep dataclass.
         Replace this module for terrain-aware planning (e.g. RRT footsteps)
         without touching anything else.

Reference: Modern Robotics Ch. 9 — trajectory generation framework.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List


@dataclass
class Footstep:
    pos: np.ndarray   # [x, y, z] world frame (foot sole centre)
    yaw: float        # heading (rad), 0 = forward
    side: str         # 'L' or 'R'
    t_start: float    # touchdown time (s)
    t_lift: float     # lift-off time (s)
    # foot half-extents for ZMP support polygon
    half_x: float = 0.10   # fore-aft half-length (m)
    half_y: float = 0.06   # lateral half-width  (m)


def generate_footsteps(
    v_forward: float = 0.3,      # desired forward speed (m/s)
    step_width: float = 0.18,    # lateral separation between feet (m)
    step_height: float = 0.05,   # swing apex above ground (m), stored for swing spline
    n_steps: int = 8,            # total number of footsteps
    dt_single: float = 0.7,      # single-support duration (s)
    dt_double: float = 0.1,      # double-support duration (s)
    yaw: float = 0.0,            # constant heading (rad); extend for turning gait
) -> List[Footstep]:
    """
    Return a list of Footstep objects placed along a straight line.

    The first two steps are shorter (half stride) to ramp up from stand-still,
    matching the initialisation strategy in Kajita (2003).

    Timing layout per step:
        |──── single-support ─────|── double ──|──── single ────── ...
        t_start                  t_lift        next t_start
    """
    steps: List[Footstep] = []
    stride = v_forward * dt_single          # metres per step
    sides = ['L', 'R']
    t = 0.0

    for i in range(n_steps):
        side = sides[i % 2]
        # lateral offset: L foot at +y, R foot at -y
        y_off = (step_width / 2) * (1 if side == 'L' else -1)
        # shorter first two steps to ramp from rest
        sx = stride * (0.5 if i < 2 else 1.0)
        pos = np.array([i * sx, y_off, 0.0])
        steps.append(Footstep(
            pos=pos, yaw=yaw, side=side,
            t_start=t, t_lift=t + dt_single,
        ))
        t += dt_single + dt_double

    return steps


if __name__ == "__main__":
    steps = generate_footsteps()
    for s in steps:
        print(f"{s.side}  x={s.pos[0]:.3f}  t=[{s.t_start:.2f}, {s.t_lift:.2f}]")
