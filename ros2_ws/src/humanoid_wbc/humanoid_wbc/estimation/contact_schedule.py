#!/usr/bin/env python3
"""contact_schedule.py — gait-phase contact schedule (Phase 1, first unit).

The state estimator only trusts a foot's kinematics when that foot is on the
ground. With no F/T or current sensors, the gait clock is our floor source of
"is this foot in contact". Pure function of time — no ROS, no sensors.

Cycle (one L/R stride) from double support:
  [0, t_ds)              double support   both down
  [t_ds, t_ds+t_ss)      right swing      left down,  right UP
  [t_ds+t_ss, 2t_ds+t_ss) double support  both down
  [2t_ds+t_ss, cycle)    left swing       right down, left UP
  cycle = 2*(t_ds + t_ss)
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Contacts:
    left: bool
    right: bool

    @property
    def double_support(self) -> bool:
        return self.left and self.right


class ContactSchedule:
    def __init__(self, t_single: float = 0.7, t_double: float = 0.1):
        if t_single <= 0 or t_double < 0:
            raise ValueError("t_single>0 and t_double>=0 required")
        self.t_ss = float(t_single)
        self.t_ds = float(t_double)
        self.cycle = 2.0 * (self.t_ss + self.t_ds)

    def contacts(self, t: float) -> Contacts:
        phase = t % self.cycle
        ds, ss = self.t_ds, self.t_ss
        if phase < ds:
            return Contacts(True, True)
        if phase < ds + ss:
            return Contacts(left=True, right=False)
        if phase < 2 * ds + ss:
            return Contacts(True, True)
        return Contacts(left=False, right=True)
