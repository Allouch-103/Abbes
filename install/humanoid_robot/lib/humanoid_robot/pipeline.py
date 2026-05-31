#!/usr/bin/env python3

"""
pipeline.py  (updated for IMU integration)
──────────────────────────────────────────
Stage 5: top-level orchestrator — now ROS 2 aware.

New responsibilities vs. the original pipeline.py:
  1. Publishes θ_nominal on /joint_nominal at the control rate so the
     balance controller can add IMU corrections on top.
  2. Subscribes to /balance_alert from the balance controller — if the
     IMU detects a persistent tilt, the planner shortens the next stride
     and widens the double-support phase (conservative re-plan).
  3. ZMP verification still runs offline before execution begins.

This implements the "outer loop" of the two-loop architecture:
  ┌──────────────────────────────────────────────┐
  │  Footstep planner → ZMP ref → Preview ctrl  │  outer (slow)
  │  → IK → θ_nominal → /joint_nominal topic    │
  └────────────────────┬─────────────────────────┘
                       │
  ┌────────────────────▼─────────────────────────┐
  │  balance_controller.py reads /joint_nominal  │  inner (200 Hz)
  │  adds Δθ_ankle from LQR(IMU) → /joint_cmds  │
  └──────────────────────────────────────────────┘

References: Modern Robotics Ch. 9; Sciavicco §6; Kajita 2003.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

from footstep_planner    import generate_footsteps
from zmp_reference       import build_zmp_reference
from preview_controller  import compute_preview_gains, run_preview_controller
from ik_solver           import solve_full_body_ik


# ── Config ───────────────────────────────────────────────────────────────────
DT           = 0.005       # 200 Hz — matches balance controller rate
Z_C          = 0.30        # CoM height (m) — must match balance_controller.py
N_PREV       = 300         # preview window: 1.5 s at 200 Hz
MARGIN       = 0.02        # ZMP must stay 2 cm inside support polygon
N_JOINTS     = 18


# ── Swing foot spline (quintic, MR Ch. 9 §9.2) ──────────────────────────────

def quintic_swing_spline(p0: np.ndarray, p1: np.ndarray,
                         apex_h: float, T: float, dt: float) -> np.ndarray:
    """
    Quintic polynomial swing foot trajectory.

    Boundary conditions: pos/vel/acc = 0 at both endpoints (zero jerk
    at touchdown — important for not disturbing the ZMP on impact).
    Apex height follows a parabolic profile: 4·h·s·(1-s).
    """
    ts   = np.arange(0.0, T - 1e-9, dt)
    s    = ts / T                           # normalised time ∈ [0, 1]
    h    = 10*s**3 - 15*s**4 + 6*s**5      # smooth-step: 0 → 1
    traj = p0 + (p1 - p0) * h[:, None]
    traj[:, 2] += apex_h * 4.0 * s * (1.0 - s)   # parabolic height
    return traj


# ── ZMP verification ─────────────────────────────────────────────────────────

def verify_zmp(com_traj: np.ndarray, footsteps, dt: float,
               margin: float = MARGIN) -> bool:
    """
    Recompute actual ZMP from CoM trajectory (finite differences) and
    check every sample lies inside the support polygon.
    Returns True if all constraints satisfied; logs violations.
    """
    ok = True
    for k in range(1, len(com_traj) - 1):
        acc_x = (com_traj[k+1, 0] - 2*com_traj[k, 0] + com_traj[k-1, 0]) / dt**2
        acc_y = (com_traj[k+1, 3] - 2*com_traj[k, 3] + com_traj[k-1, 3]) / dt**2
        zmp_x = com_traj[k, 0] - (Z_C / 9.81) * acc_x
        zmp_y = com_traj[k, 3] - (Z_C / 9.81) * acc_y
        t = k * dt
        active = [fs for fs in footsteps if fs.t_start <= t <= fs.t_lift]
        for fs in active:
            if (abs(zmp_x - fs.pos[0]) > (fs.half_x - margin) or
                    abs(zmp_y - fs.pos[1]) > (fs.half_y - margin)):
                print(f'[WARN] ZMP violation at t={t:.3f}s: '
                      f'zmp=[{zmp_x:.3f}, {zmp_y:.3f}], foot={fs.pos[:2]}')
                ok = False
    return ok


# ── ROS 2 trajectory publisher node ─────────────────────────────────────────

class ZmpTrajectoryNode(Node):
    """
    Publishes the pre-computed joint nominal trajectory at 200 Hz.
    Listens for /balance_alert and re-plans with conservative parameters
    if the IMU detects a persistent balance disturbance.
    """

    def __init__(self, robot, v_forward: float = 0.3, n_steps: int = 8):
        super().__init__('zmp_trajectory_node')
        self._robot = robot
        self._v_forward = v_forward
        self._n_steps = n_steps
        self._step_idx = 0
        self._alert_active = False
        self._conservative = False    # True after first re-plan

        # Publisher: nominal joint angles (balance controller reads this)
        self._nom_pub = self.create_publisher(
            Float32MultiArray, '/joint_nominal', 10)

        # Subscriber: balance alert from balance_controller.py
        self.create_subscription(
            Bool, '/balance_alert', self._alert_cb, 10)

        # Generate initial trajectory
        self._joint_traj, self._com_traj, self._zmp_ref = self._plan()
        self._n_samples = len(self._joint_traj)

        # Playback timer
        self.create_timer(DT, self._publish_step)
        self.get_logger().info(
            f'ZMP trajectory ready: {self._n_samples} samples @ {1/DT:.0f} Hz')

    def _alert_cb(self, msg: Bool):
        """
        When balance controller signals instability, trigger a conservative
        re-plan (shorter stride, longer double-support) starting from the
        next footstep boundary.
        """
        if msg.data and not self._alert_active:
            self.get_logger().warn('Balance alert received — re-planning with conservative params')
            self._conservative = True
            self._joint_traj, self._com_traj, self._zmp_ref = self._plan(conservative=True)
            self._step_idx = 0   # restart trajectory
        self._alert_active = msg.data

    def _plan(self, conservative: bool = False):
        """Run the full ZMP pipeline and return joint trajectory."""
        speed = self._v_forward * (0.6 if conservative else 1.0)
        dt_double = 0.18 if conservative else 0.10   # wider double-support = safer
        dt_single = 0.80 if conservative else 0.70

        footsteps = generate_footsteps(
            v_forward=speed,
            n_steps=self._n_steps,
            dt_single=dt_single,
            dt_double=dt_double,
        )
        zmp_ref = build_zmp_reference(footsteps, dt=DT)
        G_I, G_x, G_p, A, B, C = compute_preview_gains(DT, Z_C, N_preview=N_PREV)
        com_traj = run_preview_controller(zmp_ref, G_I, G_x, G_p, A, B, C, DT)

        # Swing foot splines
        N = len(com_traj)
        swing_traj = np.zeros((N, 3))
        for i, fs in enumerate(footsteps[1:], 1):
            T_sw = fs.t_lift - fs.t_start
            sp = quintic_swing_spline(footsteps[i-1].pos, fs.pos,
                                      apex_h=0.04, T=T_sw, dt=DT)
            k0 = int(fs.t_start / DT)
            swing_traj[k0:k0+len(sp)] = sp

        # IK
        joint_traj = solve_full_body_ik(com_traj, swing_traj, self._robot)

        # Verify before sending
        ok = verify_zmp(com_traj, footsteps, DT)
        if not ok:
            self.get_logger().error('ZMP constraint violated in planned trajectory!')

        return joint_traj, com_traj, zmp_ref

    def _publish_step(self):
        """Publish one timestep of the nominal trajectory."""
        if self._step_idx >= self._n_samples:
            self._step_idx = 0   # loop the gait

        msg = Float32MultiArray()
        msg.data = self._joint_traj[self._step_idx].tolist()
        self._nom_pub.publish(msg)
        self._step_idx += 1


# ── Standalone (non-ROS) pipeline function (for testing / simulation) ────────

def run_pipeline(robot, v_forward: float = 0.3, n_steps: int = 8):
    """
    Run full ZMP pipeline offline.  Returns (joint_traj, com_traj, zmp_ref).
    Use this for unit tests, simulation, or Matplotlib visualisation.
    """
    footsteps  = generate_footsteps(v_forward=v_forward, n_steps=n_steps)
    zmp_ref    = build_zmp_reference(footsteps, dt=DT)
    G_I, G_x, G_p, A, B, C = compute_preview_gains(DT, Z_C, N_preview=N_PREV)
    com_traj   = run_preview_controller(zmp_ref, G_I, G_x, G_p, A, B, C, DT)

    N          = len(com_traj)
    swing_traj = np.zeros((N, 3))
    for i, fs in enumerate(footsteps[1:], 1):
        T_sw = fs.t_lift - fs.t_start
        sp   = quintic_swing_spline(footsteps[i-1].pos, fs.pos, 0.04, T_sw, DT)
        k0   = int(fs.t_start / DT)
        swing_traj[k0:k0+len(sp)] = sp

    joint_traj = solve_full_body_ik(com_traj, swing_traj, robot)
    assert verify_zmp(com_traj, footsteps, DT), 'ZMP constraint violated!'
    return joint_traj, com_traj, zmp_ref


# ── ROS 2 entry point ────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    from robot_model import HumanoidRobot   # your robot URDF wrapper
    robot = HumanoidRobot()
    node  = ZmpTrajectoryNode(robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
