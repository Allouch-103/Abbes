"""
motion_primitives.py
====================

Smooth trajectories between named poses.

A "trajectory" here is a *list* of pose dicts, one per control tick at
RATE_HZ. The action server in command_server_node.py iterates the list,
publishes each pose, and tracks progress as (index / len).

All trajectories return *lists* (not generators), which has two
benefits:
  1. The total length is known up-front — clean for progress reporting.
  2. Trajectories can be concatenated with the `+` operator: a bow
     trajectory is just (down + hold + up).

Easing
------
Linear interpolation between poses looks robotic and produces sudden
acceleration spikes at the endpoints. We use a cosine ease-in-out:
    e(t) = 0.5 - 0.5 * cos(π * t),   t ∈ [0, 1]
This starts and ends at zero velocity (acceleration is bounded), so
servo trajectories look natural and don't excite oscillations.

This module is pure Python — no rclpy imports — so it can be tested
without launching anything.
"""

import math
from .pose_definitions import (
    ALL_JOINTS,
    HOME_POSE, SIT_POSE, BOW_POSE,
    WAVE_RIGHT_UP, WAVE_RIGHT_OUT,
    WAVE_LEFT_UP,  WAVE_LEFT_OUT,
    MARCH_R_UP, MARCH_L_UP,
)


# Control rate. 50 Hz = a pose every 20 ms.
# 100 Hz works too if you want smoother motion at the cost of more
# message traffic; this must be ≤ controllers.yaml update_rate (100 Hz).
RATE_HZ = 50


# ─────────────────────────────────────────────────────────────
#  Building blocks
# ─────────────────────────────────────────────────────────────

def _ease(t: float) -> float:
    """Cosine ease-in-out from 0 to 1. Smooth start and stop."""
    return 0.5 - 0.5 * math.cos(math.pi * t)


def _blend(p_start: dict, p_end: dict, alpha: float) -> dict:
    """Linear blend between two poses by alpha ∈ [0, 1]."""
    return {j: p_start[j] * (1.0 - alpha) + p_end[j] * alpha
            for j in ALL_JOINTS}


def trajectory_between(p_start: dict, p_end: dict,
                       duration_s: float,
                       rate_hz: int = RATE_HZ) -> list:
    """Smooth interpolation from p_start to p_end over duration_s seconds.

    Returns a list of pose dicts of length (duration_s * rate_hz + 1).
    The first pose equals p_start, the last equals p_end.
    """
    n = max(1, int(duration_s * rate_hz))
    return [_blend(p_start, p_end, _ease(i / n)) for i in range(n + 1)]


def trajectory_hold(pose: dict, duration_s: float,
                    rate_hz: int = RATE_HZ) -> list:
    """Hold a single pose for duration_s seconds (constant copies)."""
    n = max(1, int(duration_s * rate_hz))
    return [pose] * n


# ─────────────────────────────────────────────────────────────
#  Named trajectories — one per high-level action
# ─────────────────────────────────────────────────────────────

def traj_stand_still(current_pose: dict, duration_s: float = 1.5) -> list:
    """Return to HOME from current pose."""
    return trajectory_between(current_pose, HOME_POSE, duration_s)


def traj_sit_down(current_pose: dict, duration_s: float = 2.5) -> list:
    """Slowly lower into the sit pose."""
    return trajectory_between(current_pose, SIT_POSE, duration_s)


def traj_bow(current_pose: dict,
             down_s: float = 1.0,
             hold_s: float = 0.5,
             up_s: float = 1.0) -> list:
    """Bow forward, hold briefly, return to HOME.

    Three concatenated sub-trajectories:
      [current → BOW]   bend forward
      [BOW held]        pause at the bottom
      [BOW → HOME]      stand back up
    """
    return (trajectory_between(current_pose, BOW_POSE, down_s)
            + trajectory_hold(BOW_POSE, hold_s)
            + trajectory_between(BOW_POSE, HOME_POSE, up_s))


def traj_wave(which: str, current_pose: dict,
              num_waves: int = 3,
              wave_period_s: float = 0.6) -> list:
    """Lift one arm, wave it side-to-side N times, lower it.

    `which` is "left" or "right".

    Phases:
      1. lift  : current → ARM_UP                (1.0 s)
      2. wave  : ARM_UP ↔ ARM_OUT × num_waves   (num_waves × wave_period_s)
      3. lower : ARM_UP → HOME                   (1.0 s)
    """
    if which == "right":
        up_pose, out_pose = WAVE_RIGHT_UP, WAVE_RIGHT_OUT
    elif which == "left":
        up_pose, out_pose = WAVE_LEFT_UP, WAVE_LEFT_OUT
    else:
        raise ValueError(f"which must be 'left' or 'right', got {which!r}")

    half_period = wave_period_s / 2.0
    traj = trajectory_between(current_pose, up_pose, duration_s=1.0)
    for _ in range(num_waves):
        traj += trajectory_between(up_pose, out_pose, half_period)
        traj += trajectory_between(out_pose, up_pose, half_period)
    traj += trajectory_between(up_pose, HOME_POSE, duration_s=1.0)
    return traj


def traj_march_in_place(num_steps: int,
                        step_s: float = 0.8) -> list:
    """STUB for the Walk action.

    Alternates left and right legs lifting in place. No CoM shift,
    so on a freestanding biped this will tip over — fine inside Gazebo
    when the robot is on a stable base, useful for verifying that the
    AI → action server → controller pipeline works end-to-end before
    the real ZMP gait generator is online.

    When the real gait generator lands, this function is replaced.
    Nothing else in the stack changes.
    """
    traj = []
    current = HOME_POSE
    for step in range(num_steps):
        target = MARCH_R_UP if step % 2 == 0 else MARCH_L_UP
        traj += trajectory_between(current, target, step_s / 2.0)
        traj += trajectory_between(target, HOME_POSE, step_s / 2.0)
        current = HOME_POSE
    return traj
