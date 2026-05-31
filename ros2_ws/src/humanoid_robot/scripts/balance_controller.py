#!/usr/bin/env python3
"""
balance_controller.py
─────────────────────
ROS 2 node: IMU-feedback inner-loop balance controller.

WHAT THIS NODE DOES
───────────────────
  Replaces the previous PID ankle controller with an LQR regulator
  while keeping exactly the same ROS interface so nothing else changes.

  Subscribes:
    /tilt_degrees  (geometry_msgs/Vector3Stamped)  — filtered tilt angles
    /imu           (sensor_msgs/Imu)               — raw gyro (D term)
    /joint_commands (std_msgs/Float32MultiArray)    — θ_nominal from ZMP planner

  Publishes:
    /joint_commands (std_msgs/Float32MultiArray)   — θ_nominal + Δθ_ankle

WHY LQR INSTEAD OF PID
───────────────────────
  PID treats pitch and roll independently and tunes three scalar gains.
  The physical ankle, however, couples the two axes — a pitch correction
  torque slightly changes the roll moment arm.  LQR handles this via the
  full state vector x = [pitch, roll, ω_pitch, ω_roll]ᵀ and finds the
  optimal gain matrix K (2×4) in one solve.

  Concretely:
    u = −K · (x − x_ref)        (x_ref = zero — upright stance)

  K is computed offline by solving the continuous-time algebraic
  Riccati equation (CARE), traded once at node startup.  At runtime
  the cost is a single 2×4 matrix-vector multiply per cycle.

  Q penalises angle errors (stability).
  R penalises ankle joint torque (protects servos + energy).

LQR DESIGN RATIONALE (from your resources)
───────────────────────────────────────────
  The LIPM used in the preview controller gives us the state-space
  model of the ankle directly.  The linearised ankle dynamics are:

      ẋ = A·x + B·u
      A = [[0, 0, 1, 0],      (pitch integrates → pitch rate)
           [0, 0, 0, 1],      (roll  integrates → roll  rate)
           [g/l, 0, 0, 0],    (pitch accel from gravity, l=CoM height)
           [0, g/l, 0, 0]]    (roll  accel — same model)

      B = [[0, 0],
           [0, 0],
           [1/I_pitch, 0],    (ankle pitch torque → pitch accel)
           [0, 1/I_roll]]     (ankle roll  torque → roll  accel)

  where g = 9.81, l ≈ z_c (CoM height), I = moment of inertia.
  Reference: Sciavicco §3.1 (LIPM state-space); Modern Robotics Ch. 8.

ZMP RE-PLANNING HOOK
────────────────────
  If the tilt exceeds REPLANNING_THRESHOLD_DEG for REPLANNING_HOLD_CYCLES
  consecutive cycles, this node publishes a /balance_alert (std_msgs/Bool)
  that the ZMP planner subscribes to.  The planner responds by shortening
  the next step's stride and widening the support polygon.

TOPICS (identical to old PID node):
  IN:  /tilt_degrees  geometry_msgs/Vector3Stamped
  IN:  /imu           sensor_msgs/Imu
  IN:  /joint_commands std_msgs/Float32MultiArray   ← NEW (from ZMP planner)
  OUT: /joint_commands std_msgs/Float32MultiArray
  OUT: /balance_alert  std_msgs/Bool               ← NEW

LIVE TUNING (no recompile — same as before):
  ros2 param set /balance_controller q_angle  1.0
  ros2 param set /balance_controller q_rate   0.1
  ros2 param set /balance_controller r_torque 0.01
  ros2 param set /balance_controller enabled  false

BUILD:
  cd ~/Abbes/ros2_ws && colcon build --packages-select humanoid_robot

RUN:
  ros2 run humanoid_robot balance_controller
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Bool

import numpy as np
from scipy.linalg import solve_continuous_are


# ── Hardware constants (tune to your robot) ──────────────────────────────────
N_JOINTS           = 18      # total joints published on /joint_commands
ANKLE_PITCH_IDX    = 3       # index in the 18-float array — left ankle pitch
ANKLE_ROLL_IDX     = 4       # index in the 18-float array — left ankle roll
ANKLE_PITCH_IDX_R  = 8      # right ankle pitch
ANKLE_ROLL_IDX_R   = 9      # right ankle roll

Z_C                = 0.30    # CoM height above ankle (m) — measure on your robot
I_PITCH            = 0.08    # approx moment of inertia about ankle pitch axis (kg·m²)
I_ROLL             = 0.06    # approx moment of inertia about ankle roll  axis (kg·m²)
G                  = 9.81

# Safety limits
MAX_ANKLE_CORRECTION_DEG = 12.0
REPLANNING_THRESHOLD_DEG = 8.0    # tilt above this triggers ZMP re-plan
REPLANNING_HOLD_CYCLES   = 10     # must hold for N cycles to avoid false triggers


# ── LQR gain solver (run once at startup) ────────────────────────────────────

def compute_lqr_gains(q_angle: float, q_rate: float, r_torque: float):
    """
    Solve CARE for the ankle balance LQR.

    State:  x = [pitch_rad, roll_rad, pitch_rate_rad_s, roll_rate_rad_s]
    Input:  u = [ankle_pitch_torque, ankle_roll_torque]

    Returns K (2×4) — the feedback gain matrix.
    Control law:  u = −K · x    (reference = 0 = upright)
    """
    # Continuous-time state-space (linearised inverted pendulum at ankle)
    A = np.array([
        [0,      0,      1,      0     ],
        [0,      0,      0,      1     ],
        [G/Z_C,  0,      0,      0     ],   # gravity destabilises pitch
        [0,      G/Z_C,  0,      0     ],   # same for roll
    ])
    B = np.array([
        [0,          0         ],
        [0,          0         ],
        [1/I_PITCH,  0         ],
        [0,          1/I_ROLL  ],
    ])
    # Cost matrices
    # Q: penalise tilt angles more than rates (stability > compliance)
    Q = np.diag([q_angle, q_angle, q_rate, q_rate])
    # R: penalise torque (servo protection + energy efficiency)
    R = np.diag([r_torque, r_torque])

    # Solve continuous-time algebraic Riccati equation
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P   # (2×4)
    return K


# ── ROS 2 Node ────────────────────────────────────────────────────────────────

class BalanceController(Node):

    def __init__(self):
        super().__init__('balance_controller')

        # ── Declare tunable parameters ────────────────────────────────────
        self.declare_parameter('q_angle',  1.0)    # tilt error weight
        self.declare_parameter('q_rate',   0.1)    # rate error weight
        self.declare_parameter('r_torque', 0.01)   # torque cost
        self.declare_parameter('enabled',  True)
        self.add_on_set_parameters_callback(self._on_param_change)

        # ── Compute initial LQR gains ─────────────────────────────────────
        self._recompute_gains()

        # ── Internal state ────────────────────────────────────────────────
        self._tilt_pitch = 0.0        # deg (from /tilt_degrees)
        self._tilt_roll  = 0.0        # deg
        self._rate_pitch = 0.0        # rad/s (from /imu gyro)
        self._rate_roll  = 0.0        # rad/s
        self._nominal    = np.zeros(N_JOINTS)   # from ZMP planner
        self._alert_count = 0

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            Vector3Stamped, '/tilt_degrees',
            self._tilt_cb, 10)
        self.create_subscription(
            Imu, '/imu',
            self._imu_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/joint_commands',
            self._nominal_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Float32MultiArray, '/joint_commands', 10)        
        self._alert_pub = self.create_publisher(Bool, '/balance_alert', 10)
        # ── Control timer (200 Hz) ────────────────────────────────────────
        self.create_timer(0.005, self._control_loop)

        self.get_logger().info('Balance controller ready (LQR mode)')

    # ── Parameter callback: hot-reload gains without restart ─────────────
    def _on_param_change(self, params):
        for p in params:
            if p.name in ('q_angle', 'q_rate', 'r_torque'):
                self._recompute_gains()
                self.get_logger().info(f'Gains recomputed: {p.name}={p.value}')
        return rclpy.parameter.SetParametersResult(successful=True)

    def _recompute_gains(self):
        qa = self.get_parameter('q_angle').value
        qr = self.get_parameter('q_rate').value
        rt = self.get_parameter('r_torque').value
        self._K = compute_lqr_gains(qa, qr, rt)   # (2×4)
        self.get_logger().info(f'LQR K =\n{self._K}')

    # ── Subscriber callbacks ──────────────────────────────────────────────
    def _tilt_cb(self, msg: Vector3Stamped):
        self._tilt_roll  = msg.vector.x   # deg
        self._tilt_pitch = msg.vector.y   # deg

    def _imu_cb(self, msg: Imu):
        self._rate_pitch = msg.angular_velocity.y   # rad/s
        self._rate_roll  = msg.angular_velocity.x   # rad/s

    def _nominal_cb(self, msg: Float32MultiArray):
        """Receive nominal joint angles from ZMP trajectory planner."""
        if len(msg.data) == N_JOINTS:
            self._nominal = np.array(msg.data)

    # ── Main control loop (200 Hz) ────────────────────────────────────────
    def _control_loop(self):
        enabled = self.get_parameter('enabled').value

        # ── Build state vector ────────────────────────────────────────────
        x = np.array([
            np.deg2rad(self._tilt_pitch),   # pitch (rad)
            np.deg2rad(self._tilt_roll),    # roll  (rad)
            self._rate_pitch,               # pitch rate (rad/s)
            self._rate_roll,                # roll  rate (rad/s)
        ])

        # ── LQR control law: u = −K·x (torque → ankle angle via mapping) ──
        # u is a 2-vector of ankle corrective torques (N·m).
        # Convert to ankle angle corrections (rad) by dividing by stiffness.
        # For position-controlled servos, treat the torque as a proportional
        # angle increment: Δθ = u / k_stiffness.
        K_STIFFNESS = 5.0   # approximate servo stiffness (N·m/rad) — tune this
        u = -self._K @ x                         # (2,): [Δτ_pitch, Δτ_roll]
        delta_pitch_rad = u[0] / K_STIFFNESS     # ankle pitch correction (rad)
        delta_roll_rad  = u[1] / K_STIFFNESS     # ankle roll  correction (rad)

        # ── Clamp corrections ─────────────────────────────────────────────
        max_rad = np.deg2rad(MAX_ANKLE_CORRECTION_DEG)
        delta_pitch_rad = np.clip(delta_pitch_rad, -max_rad, max_rad)
        delta_roll_rad  = np.clip(delta_roll_rad,  -max_rad, max_rad)

        # ── Apply corrections on top of nominal trajectory ────────────────
        # This is the key integration point: ZMP planner sets baseline,
        # IMU loop adjusts only the ankle joints.
        cmds = self._nominal.copy()
        if enabled:
            cmds[ANKLE_PITCH_IDX]   += np.rad2deg(delta_pitch_rad)
            cmds[ANKLE_ROLL_IDX]    += np.rad2deg(delta_roll_rad)
            cmds[ANKLE_PITCH_IDX_R] += np.rad2deg(delta_pitch_rad)
            cmds[ANKLE_ROLL_IDX_R]  += np.rad2deg(delta_roll_rad)

        # ── Publish joint commands ────────────────────────────────────────
        out = Float32MultiArray()
        out.data = cmds.tolist()
        self._cmd_pub.publish(out)

        # ── ZMP re-planning alert ─────────────────────────────────────────
        # If tilt is dangerously large, signal the ZMP planner to shorten stride.
        tilt_mag = np.sqrt(self._tilt_pitch**2 + self._tilt_roll**2)
        if tilt_mag > REPLANNING_THRESHOLD_DEG:
            self._alert_count += 1
        else:
            self._alert_count = max(0, self._alert_count - 1)

        alert_active = self._alert_count >= REPLANNING_HOLD_CYCLES
        alert_msg = Bool()
        alert_msg.data = alert_active
        self._alert_pub.publish(alert_msg)

        if alert_active:
            self.get_logger().warn(
                f'Balance alert: tilt={tilt_mag:.1f}° — signalling ZMP re-plan')


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
