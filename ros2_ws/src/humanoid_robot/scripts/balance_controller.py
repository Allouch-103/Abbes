#!/usr/bin/env python3
"""
balance_controller.py — LQR inner loop with open-loop fallback.

Open loop:  if IMU silent > 0.5s → pass /joint_commands through unchanged
Closed loop: if IMU active → apply LQR ankle correction
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Bool

import numpy as np
from scipy.linalg import solve_continuous_are
import time

# ── Joint indices (URDF ros2_control order) ───────────────────────────────────
N_JOINTS           = 18
ANKLE_PITCH_IDX    = 3    # r_ankle_pitch
ANKLE_ROLL_IDX     = 4    # r_ankle_roll
ANKLE_PITCH_IDX_R  = 8    # l_ankle_pitch
ANKLE_ROLL_IDX_R   = 9    # l_ankle_roll

# ── Physical constants ────────────────────────────────────────────────────────
Z_C     = 0.2698
I_PITCH = 0.08
I_ROLL  = 0.06
G       = 9.81

# ── Safety ────────────────────────────────────────────────────────────────────
MAX_ANKLE_CORRECTION_DEG = 12.0
REPLANNING_THRESHOLD_DEG = 8.0
REPLANNING_HOLD_CYCLES   = 10
IMU_TIMEOUT_SEC          = 0.5   # open loop if no IMU for this long


def compute_lqr_gains(q_angle, q_rate, r_torque):
    A = np.array([
        [0,     0,     1,     0    ],
        [0,     0,     0,     1    ],
        [G/Z_C, 0,     0,     0    ],
        [0,     G/Z_C, 0,     0    ],
    ])
    B = np.array([
        [0,          0        ],
        [0,          0        ],
        [1/I_PITCH,  0        ],
        [0,          1/I_ROLL ],
    ])
    Q = np.diag([q_angle, q_angle, q_rate, q_rate])
    R = np.diag([r_torque, r_torque])
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K


class BalanceController(Node):

    def __init__(self):
        super().__init__('balance_controller')

        self.declare_parameter('q_angle',  1.0)
        self.declare_parameter('q_rate',   0.1)
        self.declare_parameter('r_torque', 0.01)
        self.declare_parameter('enabled',  True)
        self.add_on_set_parameters_callback(self._on_param_change)

        self._recompute_gains()

        # ── State ─────────────────────────────────────────────────────────
        self._tilt_pitch  = 0.0
        self._tilt_roll   = 0.0
        self._rate_pitch  = 0.0
        self._rate_roll   = 0.0
        self._nominal     = np.zeros(N_JOINTS)
        self._alert_count = 0
        self._last_imu_t  = None   # None = never received

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Vector3Stamped, '/tilt_degrees', self._tilt_cb, 10)
        self.create_subscription(Imu,            '/imu',          self._imu_cb,  10)
        self.create_subscription(Float32MultiArray, '/joint_commands',
            self._nominal_cb,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))

        # ── Publishers ────────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(Float32MultiArray, '/joint_commands_corrected', 10)
        self._alert_pub = self.create_publisher(Bool, '/balance_alert', 10)

        self.create_timer(0.005, self._control_loop)
        self.get_logger().info('Balance controller ready (LQR + open-loop fallback)')

    def _on_param_change(self, params):
        for p in params:
            if p.name in ('q_angle', 'q_rate', 'r_torque'):
                self._recompute_gains()
        return rclpy.parameter.SetParametersResult(successful=True)

    def _recompute_gains(self):
        qa = self.get_parameter('q_angle').value
        qr = self.get_parameter('q_rate').value
        rt = self.get_parameter('r_torque').value
        self._K = compute_lqr_gains(qa, qr, rt)
        self.get_logger().info(f'LQR K computed')

    def _tilt_cb(self, msg: Vector3Stamped):
        self._tilt_roll  = msg.vector.x
        self._tilt_pitch = msg.vector.y

    def _imu_cb(self, msg: Imu):
        self._rate_pitch = msg.angular_velocity.y
        self._rate_roll  = msg.angular_velocity.x
        self._last_imu_t = time.monotonic()   # stamp every IMU message

    def _nominal_cb(self, msg: Float32MultiArray):
        if len(msg.data) == N_JOINTS:
            self._nominal = np.array(msg.data)

    def _control_loop(self):
        enabled    = self.get_parameter('enabled').value
        cmds       = self._nominal.copy()
        now        = time.monotonic()

        # ── Decide: open loop or closed loop ─────────────────────────────
        imu_alive = (
            self._last_imu_t is not None and
            (now - self._last_imu_t) < IMU_TIMEOUT_SEC
        )

        if enabled and imu_alive:
            # ── CLOSED LOOP — LQR correction ──────────────────────────────
            x = np.array([
                np.deg2rad(self._tilt_pitch),
                np.deg2rad(self._tilt_roll),
                self._rate_pitch,
                self._rate_roll,
            ])
            K_STIFFNESS     = 5.0
            u               = -self._K @ x
            delta_pitch_rad = np.clip(u[0] / K_STIFFNESS,
                                      -np.deg2rad(MAX_ANKLE_CORRECTION_DEG),
                                       np.deg2rad(MAX_ANKLE_CORRECTION_DEG))
            delta_roll_rad  = np.clip(u[1] / K_STIFFNESS,
                                      -np.deg2rad(MAX_ANKLE_CORRECTION_DEG),
                                       np.deg2rad(MAX_ANKLE_CORRECTION_DEG))

            cmds[ANKLE_PITCH_IDX]   += np.rad2deg(delta_pitch_rad)
            cmds[ANKLE_ROLL_IDX]    += np.rad2deg(delta_roll_rad)
            cmds[ANKLE_PITCH_IDX_R] += np.rad2deg(delta_pitch_rad)
            cmds[ANKLE_ROLL_IDX_R]  += np.rad2deg(delta_roll_rad)

            mode = 'CLOSED'
        else:
            # ── OPEN LOOP — pass nominal through unchanged ─────────────────
            mode = 'OPEN (no IMU)' if not imu_alive else 'OPEN (disabled)'

        # Log mode changes (not every tick)
        if not hasattr(self, '_last_mode') or self._last_mode != mode:
            self.get_logger().info(f'Balance mode: {mode}')
            self._last_mode = mode

        # ── Publish ───────────────────────────────────────────────────────
        out      = Float32MultiArray()
        out.data = cmds.tolist()
        self._cmd_pub.publish(out)

        # ── ZMP re-planning alert ─────────────────────────────────────────
        tilt_mag = np.sqrt(self._tilt_pitch**2 + self._tilt_roll**2)
        if tilt_mag > REPLANNING_THRESHOLD_DEG:
            self._alert_count += 1
        else:
            self._alert_count = max(0, self._alert_count - 1)

        alert_msg      = Bool()
        alert_msg.data = self._alert_count >= REPLANNING_HOLD_CYCLES
        self._alert_pub.publish(alert_msg)


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
