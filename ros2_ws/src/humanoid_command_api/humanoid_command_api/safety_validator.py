"""
safety_validator.py
====================

AI Step 5 — the Safety Validator.

WHAT THIS IS
------------
A thin gate the dispatcher consults AFTER the LLM has chosen a tool+args
but BEFORE that tool is actually run on the robot (`execute_tool`). If the
gate says "deny", the action never becomes a ROS2 goal at all — instead the
denial reason is fed back to the model so the robot can *explain* why it
won't comply, rather than silently doing nothing.

    user text ──► LLM picks tool+args ──► [ SafetyValidator.check ] ──► execute_tool
                                                   │ denied
                                                   └──► reason string back to the model

WHY IT'S SEPARATE FROM THE COMMAND SERVER
-----------------------------------------
The command server already enforces, per-goal:
  - argument ranges / enums (walk distance 0.1–3.0, direction/which enums)
  - mutual exclusion (reject a new goal while another action runs)

This validator does NOT duplicate those. It adds checks the server has no
good place for, because they depend on *robot physical state* or on
*command history* rather than on a single goal message:

  1. TILT GATING (walk only) — don't start walking on an already-tilted,
     unstable base. We read the robot's current tilt via a status provider
     and refuse `walk` if pitch or roll exceeds a limit.
  2. RATE LIMITING — refuse a new motion if one was allowed too recently,
     so a chatty model can't fire a burst of rapid-fire motions.

DESIGN
------
Like `command_tools.py`, this file is deliberately FREE of rclpy. It is pure
policy + a little state, so it can be unit-tested without a running robot.
The one thing it needs from the outside — the robot's current tilt — is
passed in as a `status_provider` callable (the dispatcher hands it
`self.do_get_status`). The validator only calls it when a check actually
needs it (i.e. for `walk`), so non-walk commands cost no ROS round-trip.

SAFETY INVARIANTS
-----------------
- `stop` and `get_status` are NEVER blocked, by anything. `stop` is the
  emergency brake; `get_status` only reads. They also do not touch the
  rate-limit clock (issuing a status query must not delay the next motion).
- Fail-open on missing IMU: the command server reports a tilt of 0.0° when
  no IMU message has arrived. So with no IMU the tilt gate sees 0.0 and
  permits walking — the SAME conservative-but-not-blocking stance the
  server's own Walk handler takes. If you later want to hard-block walking
  when tilt is *unknown*, that's a deliberate policy change to make here.
"""

import time
from typing import Callable, Optional, Tuple


# Tools that bypass every safety check. The emergency stop must always be
# able to fire, and reading status can never be unsafe.
EXEMPT_TOOLS = {"stop", "get_status"}

# Default thresholds. Passed to the constructor so they're easy to tune
# (and to override in tests) without editing logic.
DEFAULT_TILT_LIMIT_DEG = 15.0   # refuse to walk if |pitch| or |roll| exceeds this
DEFAULT_MIN_INTERVAL_SEC = 1.5  # minimum gap between two allowed motions


class SafetyValidator:
    """Stateful gate: see module docstring. One instance per dispatcher."""

    def __init__(self,
                 tilt_limit_deg: float = DEFAULT_TILT_LIMIT_DEG,
                 min_interval_sec: float = DEFAULT_MIN_INTERVAL_SEC,
                 clock: Callable[[], float] = time.monotonic):
        self.tilt_limit_deg = tilt_limit_deg
        self.min_interval_sec = min_interval_sec
        # Injectable clock so rate-limiting is testable without real sleeps.
        # monotonic (not wall-clock) so it can't jump backwards on NTP steps.
        self._clock = clock
        # Timestamp of the last MOTION we allowed. None = none yet.
        self._last_motion_time: Optional[float] = None

    def check(self,
              name: str,
              args: dict,
              status_provider: Optional[Callable[[], dict]] = None
              ) -> Tuple[bool, str]:
        """Decide whether `name(args)` may run right now.

        Returns (allowed, reason). `reason` is "" when allowed, and a short
        human-readable explanation when denied (relayed to the model).

        `status_provider`, if given, is a zero-arg callable returning the
        same dict shape as the dispatcher's `do_get_status` (keys
        `tilt_pitch_deg`, `tilt_roll_deg`, ...). It is only invoked when a
        check needs live state (currently: `walk`).
        """
        args = args or {}

        # ── Invariant: stop / get_status are never gated, and don't
        #    advance the rate-limit clock. ────────────────────────────
        if name in EXEMPT_TOOLS:
            return True, ""

        now = self._clock()

        # ── Rate limiting ──────────────────────────────────────────
        if self._last_motion_time is not None:
            elapsed = now - self._last_motion_time
            if elapsed < self.min_interval_sec:
                wait = self.min_interval_sec - elapsed
                return (False,
                        f"Rate limited: last motion was {elapsed:.1f}s ago; "
                        f"wait {wait:.1f}s before the next one.")

        # ── Tilt gating (walk only) ────────────────────────────────
        if name == "walk":
            allowed, reason = self._check_tilt(status_provider)
            if not allowed:
                return False, reason

        # Passed every applicable check → this motion is going to run, so
        # start the rate-limit timer from now.
        self._last_motion_time = now
        return True, ""

    # ─────────────────────────────────────────────────────────────
    def _check_tilt(self,
                    status_provider: Optional[Callable[[], dict]]
                    ) -> Tuple[bool, str]:
        """Refuse to walk if the robot is already tilted past the limit."""
        if status_provider is None:
            # No way to read state → don't block (fail open). See module
            # docstring "Fail-open on missing IMU".
            return True, ""

        status = status_provider() or {}
        pitch = status.get("tilt_pitch_deg")
        roll = status.get("tilt_roll_deg")
        if pitch is None or roll is None:
            return True, ""   # status didn't carry tilt → fail open

        if abs(pitch) > self.tilt_limit_deg or abs(roll) > self.tilt_limit_deg:
            return (False,
                    f"Unsafe to walk: robot is tilted "
                    f"(pitch={pitch:.1f}°, roll={roll:.1f}°), "
                    f"limit is {self.tilt_limit_deg:.0f}°. "
                    f"Stabilize (e.g. stand_still) before walking.")
        return True, ""
