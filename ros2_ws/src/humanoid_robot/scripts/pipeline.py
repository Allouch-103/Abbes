#!/usr/bin/env python3
"""
pipeline.py — outer loop ZMP trajectory planner.
Publishes /com_trajectory + /foot_trajectory for ik_vectors_DLS.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

from footstep_planner   import generate_footsteps
from zmp_reference      import build_zmp_reference
from preview_controller import compute_preview_gains, run_preview_controller

DT          = 0.005
Z_C         = 0.30    # CoM height above ground -- must match Gazebo spawn z
FOOT_HEIGHT = 0.05    # ankle joint height above ground (= URDF foot_height)
N_PREV   = 300
N_JOINTS = 18

def quintic_swing_spline(p0, p1, apex_h, T, dt):
    ts   = np.arange(0.0, T - 1e-9, dt)
    s    = ts / T
    h    = 10*s**3 - 15*s**4 + 6*s**5
    traj = p0 + (p1 - p0) * h[:, None]
    traj[:, 2] += apex_h * 4.0 * s * (1.0 - s)
    return traj

class ZmpTrajectoryNode(Node):
    def __init__(self, v_forward=0.03, n_steps=4):
        super().__init__('zmp_trajectory_node')
        self._v_forward    = v_forward
        self._n_steps      = n_steps
        self._step_idx     = 0
        self._init_counter = 0
        self._init_done    = False

        qos = rclpy.qos.QoSProfile(depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        self._com_pub  = self.create_publisher(PoseStamped, '/com_trajectory', qos)
        self._foot_pub = self.create_publisher(PoseArray,   '/foot_trajectory', qos)

        self.create_subscription(Bool, '/balance_alert', self._alert_cb, 10)

        self._com_traj, self._swing_traj, self._footsteps = self._plan()
        self._n_samples = len(self._com_traj)

        self.create_timer(DT, self._publish_step)
        self.get_logger().info(
            f'ZMP trajectory ready: {self._n_samples} samples @ {1/DT:.0f} Hz')

    def _alert_cb(self, msg):
        if msg.data:
            self.get_logger().warn('Balance alert — replanning conservatively')
            self._com_traj, self._swing_traj, self._footsteps = self._plan(conservative=True)
            self._n_samples = len(self._com_traj)
            self._step_idx  = 0

    def _plan(self, conservative=False):
        speed     = self._v_forward * (0.6 if conservative else 1.0)
        dt_double = 0.18 if conservative else 0.10
        dt_single = 0.80 if conservative else 0.70

        footsteps = generate_footsteps(
            v_forward=speed, n_steps=self._n_steps,
            dt_single=dt_single, dt_double=dt_double)
        zmp_ref = build_zmp_reference(footsteps, dt=DT)
        G_I, G_x, G_p, A, B, C = compute_preview_gains(DT, Z_C, N_preview=N_PREV)
        com_traj = run_preview_controller(zmp_ref, G_I, G_x, G_p, A, B, C, DT)

        N = len(com_traj)
        swing_traj = np.zeros((N, 3))
        for i, fs in enumerate(footsteps[1:], 1):
            T_sw = fs.t_lift - fs.t_start
            sp   = quintic_swing_spline(footsteps[i-1].pos, fs.pos, 0.04, T_sw, DT)
            k0   = int(fs.t_start / DT)
            swing_traj[k0:k0+len(sp)] = sp

        return com_traj, swing_traj, footsteps

    def _publish_step(self):
        if not self._init_done:
            self._init_counter += 1
            com_msg = PoseStamped()
            com_msg.header.frame_id = 'world'
            com_msg.pose.position.x = 0.0
            com_msg.pose.position.y = 0.0
            com_msg.pose.position.z = Z_C
            self._com_pub.publish(com_msg)
            feet_msg = PoseArray()
            feet_msg.header.frame_id = 'world'
            r, l = Pose(), Pose()
            r.position.y =  0.04;  r.position.z = FOOT_HEIGHT
            l.position.y = -0.04;  l.position.z = FOOT_HEIGHT
            feet_msg.poses = [r, l]
            self._foot_pub.publish(feet_msg)
            if self._init_counter >= int(2.0 / DT):
                self._init_done = True
                self.get_logger().info('Init complete — starting trajectory')
            return

        if self._step_idx >= self._n_samples:
            self._step_idx = 0

        k     = self._step_idx
        com_x = self._com_traj[k, 0]
        com_y = self._com_traj[k, 1]
        com_z = Z_C
        sw_x  = self._swing_traj[k, 0]
        sw_z  = self._swing_traj[k, 2]

        com_msg = PoseStamped()
        com_msg.header.frame_id = 'world'
        com_msg.pose.position.x = com_x
        com_msg.pose.position.y = com_y
        com_msg.pose.position.z = com_z
        self._com_pub.publish(com_msg)

        feet_msg = PoseArray()
        feet_msg.header.frame_id = 'world'
        r, l = Pose(), Pose()
        r.position.x = sw_x;  r.position.y =  0.04;  r.position.z = sw_z
        l.position.x = 0.0;   l.position.y = -0.04;  l.position.z = FOOT_HEIGHT
        feet_msg.poses = [r, l]
        self._foot_pub.publish(feet_msg)

        self._step_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = ZmpTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
