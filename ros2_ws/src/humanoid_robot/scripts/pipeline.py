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

DT       = 0.005
Z_C      = 0.22
N_PREV   = 300
N_JOINTS = 18

APEX_H   = 0.04   # swing foot clearance (m)
HIP_Y    = 0.04   # half hip width = foot y offset (m)


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

        self._com_traj, self._foot_traj, self._footsteps = self._plan()
        self._n_samples = len(self._com_traj)

        self.create_timer(DT, self._publish_step)
        self.get_logger().info(
            f'ZMP trajectory ready: {self._n_samples} samples @ {1/DT:.0f} Hz')

    def _alert_cb(self, msg):
        if msg.data:
            self.get_logger().warn('Balance alert — replanning conservatively')
            #self._com_traj, self._foot_traj, self._footsteps = self._plan(conservative=True)
            #self._n_samples = len(self._com_traj)
            #self._step_idx  = 0

    def _plan(self, conservative=False):
        speed     = self._v_forward * (0.6 if conservative else 1.0)
        dt_double = 0.40 if conservative else 0.30
        dt_single = 1.20 if conservative else 1.00

        footsteps = generate_footsteps(
            v_forward=speed, n_steps=self._n_steps,
            dt_single=dt_single, dt_double=dt_double)
        zmp_ref = build_zmp_reference(footsteps, dt=DT)
        G_I, G_x, G_p, A, B, C = compute_preview_gains(DT, Z_C, N_preview=N_PREV)
        com_traj = run_preview_controller(zmp_ref, G_I, G_x, G_p, A, B, C, DT)

        N = len(com_traj)
        foot_traj = self._build_foot_trajectories(footsteps, N)

        return com_traj, foot_traj, footsteps

    def _build_foot_trajectories(self, footsteps, N):
        """
        Build per-timestep (x,y,z) for both feet.
        Returns foot_traj of shape (N, 2, 3):
          axis-1 index 0 = +y foot (IK poses[0], the +y-side hip)
          axis-1 index 1 = -y foot (IK poses[1], the -y-side hip)

        Footstep convention (footstep_planner):
          'L' steps have pos[1] > 0 (+y) — these are the stance phases where
          the +y foot is planted and the -y foot swings.
          'R' steps have pos[1] < 0 (-y) — -y planted, +y swings.
        """
        foot_traj = np.zeros((N, 2, 3))

        # Last known planted position for each foot side
        # Index 0 = +y side, index 1 = -y side
        last_pos = np.array([
            [0.0,  HIP_Y, 0.0],
            [0.0, -HIP_Y, 0.0],
        ], dtype=float)

        # Fill entire array with resting positions first (covers double-support gaps)
        foot_traj[:, 0] = last_pos[0]
        foot_traj[:, 1] = last_pos[1]

        for i, fs in enumerate(footsteps):
            # Which side is STANCE for this footstep?
            stance_idx = 0 if fs.pos[1] > 0 else 1   # +y=0, -y=1
            swing_idx  = 1 - stance_idx

            k_start = int(fs.t_start / DT)
            k_end   = min(int(fs.t_lift / DT), N)
            if k_start >= N:
                break

            # Stance foot: hold at this footstep's landed position, z=0
            stance_pos = np.array([fs.pos[0], fs.pos[1], 0.0])
            foot_traj[k_start:k_end, stance_idx] = stance_pos

            # Swing foot: find where it will land next (next footstep for this side)
            swing_y_sign = 1.0 if swing_idx == 0 else -1.0
            next_for_swing = next(
                (nfs for nfs in footsteps[i + 1:]
                 if np.sign(nfs.pos[1]) == swing_y_sign),
                None
            )

            swing_start = last_pos[swing_idx].copy()
            swing_dst   = (np.array([next_for_swing.pos[0], next_for_swing.pos[1], 0.0])
                           if next_for_swing is not None else swing_start.copy())

            # Quintic spline across the stance phase duration
            n_swing = k_end - k_start
            if n_swing > 0:
                for j in range(n_swing):
                    s = j / max(n_swing - 1, 1)
                    h = 10*s**3 - 15*s**4 + 6*s**5
                    pos = swing_start + (swing_dst - swing_start) * h
                    pos[2] = APEX_H * 4.0 * s * (1.0 - s)
                    foot_traj[k_start + j, swing_idx] = pos

            # Advance memory of where each foot last was
            last_pos[stance_idx] = stance_pos
            if next_for_swing is not None:
                last_pos[swing_idx] = swing_dst

        return foot_traj

    def _publish_step(self):
        if not self._init_done:
            self._init_counter += 1
            STAND_Z = 0.27
            ramp_n  = int(3.0 / DT)
            a = min(1.0, self._init_counter / float(ramp_n))
            com_msg = PoseStamped()
            com_msg.header.frame_id = 'world'
            com_msg.pose.position.x = 0.0
            com_msg.pose.position.y = 0.0
            com_msg.pose.position.z = STAND_Z + a * (Z_C - STAND_Z)
            self._com_pub.publish(com_msg)
            feet_msg = PoseArray()
            feet_msg.header.frame_id = 'world'
            r, l = Pose(), Pose()
            r.position.y =  HIP_Y
            l.position.y = -HIP_Y
            feet_msg.poses = [r, l]
            self._foot_pub.publish(feet_msg)
            if self._init_counter >= int(4.5 / DT):
                self._init_done = True
                self.get_logger().info('Init complete (eased into crouch) — starting trajectory')
            return

        if self._step_idx >= self._n_samples:
            self._step_idx = 0

        k = self._step_idx

        com_msg = PoseStamped()
        com_msg.header.frame_id = 'world'
        com_msg.pose.position.x = float(self._com_traj[k, 0])
        com_msg.pose.position.y = float(self._com_traj[k, 1])
        com_msg.pose.position.z = Z_C
        self._com_pub.publish(com_msg)

        # foot_traj shape: (N, 2, 3) — index 0 = +y foot, index 1 = -y foot
        f0 = self._foot_traj[k, 0]
        f1 = self._foot_traj[k, 1]

        feet_msg = PoseArray()
        feet_msg.header.frame_id = 'world'
        r, l = Pose(), Pose()
        r.position.x = float(f0[0]); r.position.y = float(f0[1]); r.position.z = float(f0[2])
        l.position.x = float(f1[0]); l.position.y = float(f1[1]); l.position.z = float(f1[2])
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