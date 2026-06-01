#!/usr/bin/env python3
"""
pipeline.py — outer loop ZMP trajectory planner.
Publishes /com_trajectory + /foot_trajectory for ik_vectors_DLS.

Foot bookkeeping (rewritten): each foot is PLANTED during its own stance
intervals and SWINGS (with a lift) during the OTHER foot's stance — properly
alternating, matched to the IK's y-convention (poses[0] = +y foot, poses[1] =
-y foot, same as ik_vectors_DLS hip_r at +y / hip_l at -y).
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
Z_C      = 0.22          # hip height the IK tracks (< leg reach so knees bend)
N_PREV   = 300
SWING_H  = 0.02          # swing-foot apex lift (m) — small for gentle stepping


def _build_foot_trajs(footsteps, N, dt, apex):
    """Return (f_plus, f_minus): trajectories for the +y foot (ik poses[0]) and
    the -y foot (ik poses[1]).

    CORRECT timing: a footstep is the STANCE during its [t_start, t_lift] (the
    ZMP/CoM is over it then). The OTHER foot SWINGS during that window — so the
    swing foot lifts only while the CoM is fully over the stance foot. (The old
    code swung a foot during the double-support transition, while the CoM was
    still over it → tipped toward the lifting side.)
    """
    L = sorted([fs for fs in footsteps if fs.pos[1] > 0], key=lambda s: s.t_start)  # +y
    R = sorted([fs for fs in footsteps if fs.pos[1] < 0], key=lambda s: s.t_start)  # -y
    fL = np.zeros((N, 3)); fR = np.zeros((N, 3))
    # current planted positions (updated as each foot lands)
    curL = L[0].pos.astype(float).copy(); curL[2] = 0.0
    curR = R[0].pos.astype(float).copy(); curR[2] = 0.0
    stances = sorted(footsteps, key=lambda s: s.t_start)
    for k in range(N):
        t = k * dt
        stance = next((s for s in stances if s.t_start <= t <= s.t_lift), None)
        if stance is None:                       # double support: hold both
            fL[k] = curL; fR[k] = curR
            continue
        stance_plus = stance.pos[1] > 0          # is the +y(L) foot the stance?
        own = L if stance_plus else R
        oth = R if stance_plus else L
        # swing foot goes from its placement BEFORE this stance to the one AFTER
        prev = [p for p in oth if p.t_lift <= stance.t_start]
        nxt  = [p for p in oth if p.t_start >= stance.t_lift]
        p0 = (prev[-1].pos if prev else (curR if stance_plus else curL)).astype(float)
        p1 = (nxt[0].pos if nxt else p0).astype(float)
        a  = (t - stance.t_start) / (stance.t_lift - stance.t_start)
        h  = 10*a**3 - 15*a**4 + 6*a**5
        sw = (p0 + (p1 - p0) * h); sw = sw.astype(float).copy()
        sw[2] = apex * np.sin(np.pi * a) ** 2
        st = stance.pos.astype(float).copy(); st[2] = 0.0
        if stance_plus:
            fL[k] = st; fR[k] = sw
            curL = st.copy(); curR = sw.copy(); curR[2] = 0.0
        else:
            fR[k] = st; fL[k] = sw
            curR = st.copy(); curL = sw.copy(); curL[2] = 0.0
    return fL, fR


class ZmpTrajectoryNode(Node):
    # v_forward=0 → march in place (isolates lateral balance). Raise to walk fwd.
    def __init__(self, v_forward=0.0, n_steps=8):
        super().__init__('zmp_trajectory_node')
        # v_forward as a ROS param: 0 = march in place, >0 = walk forward (m/s).
        self.declare_parameter('v_forward', v_forward)
        self.declare_parameter('n_steps', n_steps)
        self._v_forward    = float(self.get_parameter('v_forward').value)
        self._n_steps      = int(self.get_parameter('n_steps').value)
        self._step_idx     = 0
        self._init_counter = 0
        self._init_done    = False

        qos = rclpy.qos.QoSProfile(depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self._com_pub  = self.create_publisher(PoseStamped, '/com_trajectory', qos)
        self._foot_pub = self.create_publisher(PoseArray,   '/foot_trajectory', qos)
        self.create_subscription(Bool, '/balance_alert', self._alert_cb, 10)

        self._com_traj, self._fplus, self._fminus, self._footsteps = self._plan()
        self._n_samples = len(self._com_traj)

        self.create_timer(DT, self._publish_step)
        self.get_logger().info(
            f'ZMP trajectory ready: {self._n_samples} samples @ {1/DT:.0f} Hz '
            f'(v_forward={self._v_forward})')

    def _alert_cb(self, msg):
        if msg.data:
            self.get_logger().warn('Balance alert — replanning conservatively')
            self._com_traj, self._fplus, self._fminus, self._footsteps = \
                self._plan(conservative=True)
            self._n_samples = len(self._com_traj)
            self._step_idx  = 0

    def _plan(self, conservative=False):
        speed     = self._v_forward * (0.6 if conservative else 1.0)
        # SHORT single-support (less time on one foot) + LONG double-support
        # (both feet down = stable) → minimise the unstable single-support window.
        dt_double = 0.60 if conservative else 0.50
        dt_single = 0.45 if conservative else 0.40
        footsteps = generate_footsteps(
            v_forward=speed, n_steps=self._n_steps,
            dt_single=dt_single, dt_double=dt_double)
        zmp_ref  = build_zmp_reference(footsteps, dt=DT)
        G_I, G_x, G_p, A, B, C = compute_preview_gains(DT, Z_C, N_preview=N_PREV)
        com_traj = run_preview_controller(zmp_ref, G_I, G_x, G_p, A, B, C, DT)
        N = len(com_traj)
        f_plus, f_minus = _build_foot_trajs(footsteps, N, DT, SWING_H)
        return com_traj, f_plus, f_minus, footsteps

    def _publish_step(self):
        if not self._init_done:
            self._init_counter += 1
            # EASE into the crouch (sim snaps joints): ramp CoM 0.27→Z_C over 3s,
            # then hold ~1.5s so balance settles, before stepping.
            STAND_Z = 0.27
            a = min(1.0, self._init_counter / float(int(3.0 / DT)))
            com = PoseStamped(); com.header.frame_id = 'world'
            com.pose.position.z = STAND_Z + a * (Z_C - STAND_Z)
            self._com_pub.publish(com)
            feet = PoseArray(); feet.header.frame_id = 'world'
            r, l = Pose(), Pose()
            r.position.y = +0.04
            l.position.y = -0.04
            feet.poses = [r, l]
            self._foot_pub.publish(feet)
            if self._init_counter >= int(4.5 / DT):
                self._init_done = True
                self.get_logger().info('Init complete (eased into crouch) — stepping')
            return

        if self._step_idx >= self._n_samples:
            self._step_idx = 0
        k = self._step_idx

        # com_traj columns are [x, ẋ, ẍ, y, ẏ, ÿ] — the lateral CoM is col 3,
        # NOT col 1 (that's ẋ). Reading col 1 gave ZERO weight-shift → the robot
        # never got its CoM over the stance foot and always tipped sideways.
        com = PoseStamped(); com.header.frame_id = 'world'
        com.pose.position.x = float(self._com_traj[k, 0])   # forward
        com.pose.position.y = float(self._com_traj[k, 3])   # lateral (FIX: was [k,1])
        com.pose.position.z = Z_C
        self._com_pub.publish(com)

        feet = PoseArray(); feet.header.frame_id = 'world'
        r, l = Pose(), Pose()    # r = +y foot (poses[0]), l = -y foot (poses[1])
        r.position.x, r.position.y, r.position.z = map(float, self._fplus[k])
        l.position.x, l.position.y, l.position.z = map(float, self._fminus[k])
        feet.poses = [r, l]
        self._foot_pub.publish(feet)

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
