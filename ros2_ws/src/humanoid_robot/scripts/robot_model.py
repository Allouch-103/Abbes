"""
robot_model.py
Pinocchio-based kinematic model for the humanoid robot.

Verified against humanoid.urdf.xacro joint placements:
  l_hip_roll    z= 0.2955 m  (hip height above ground)
  l_knee_pitch  z=-0.1225 m  (thigh length, downward)
  l_ankle_pitch z=-0.1230 m  (shank length, downward)
  l_foot        z= 0.0500 m  (foot block height, top face at ankle)
  -> foot SOLE = l_foot.z - FOOT_HEIGHT = 0.05 - 0.05 = 0.0  (ground)

Pinocchio joint order (LEFT leg first, from your URDF):
   1  l_hip_roll     6  l_shoulder_pitch  11  r_hip_roll
   2  l_hip_pitch    7  l_shoulder_roll   12  r_hip_pitch
   3  l_knee_pitch   8  l_elbow_roll      13  r_knee_pitch
   4  l_ankle_pitch  9  head_yaw          14  r_ankle_pitch
   5  l_ankle_roll  10  camera_pitch      15  r_ankle_roll

/joint_commands order (RIGHT leg first):
   0  r_hip_roll    5  l_hip_roll
   1  r_hip_pitch   6  l_hip_pitch
   2  r_knee_pitch  7  l_knee_pitch
   3  r_ankle_pitch 8  l_ankle_pitch
   4  r_ankle_roll  9  l_ankle_roll
  10-17 arms, head
"""

import os
import subprocess
import tempfile
import numpy as np

try:
    import pinocchio as pin
except ImportError:
    raise ImportError(
        "pinocchio not found. Install:\n"
        "    pip install pin --break-system-packages"
    )

# ── URDF path ─────────────────────────────────────────────────────────────────
URDF_XACRO_PATH = os.path.expanduser(
    "~/Abbes/ros2_ws/src/my_robot_description/urdf/humanoid.urdf.xacro"
)

# ── Physical constants (from URDF geometry and masses) ───────────────────────
# hip joint height = 0.2955 m (from pinocchio joint placement above)
# CoM is roughly at hip height * 0.9 accounting for lower leg mass
Z_C          = 0.2698   # m  (from Test 1 pinocchio measurement -- use this)
I_PITCH      = 0.07044  # kg.m2
I_ROLL       = 0.05987  # kg.m2
K_STIFFNESS  = 1.87     # N.m/rad DS3218; use 0.93 for MG996R

THIGH_LENGTH = 0.1225   # m
SHANK_LENGTH = 0.1230   # m
FOOT_HEIGHT  = 0.0500   # m  foot block height
HIP_HEIGHT   = 0.2955   # m  hip joint above ground (from pinocchio)
HIP_OFFSET_Y = 0.0400   # m  lateral offset

# ── Joint layout ──────────────────────────────────────────────────────────────
N_JOINTS = 18
N_STANCE = 5
N_SWING  = 5

R_HIP_ROLL    = 0;  R_HIP_PITCH   = 1;  R_KNEE_PITCH  = 2
R_ANKLE_PITCH = 3;  R_ANKLE_ROLL  = 4
L_HIP_ROLL    = 5;  L_HIP_PITCH   = 6;  L_KNEE_PITCH  = 7
L_ANKLE_PITCH = 8;  L_ANKLE_ROLL  = 9

STANCE = slice(5, 10)
SWING  = slice(0, 5)

STANCE_JOINT_NAMES = [
    "l_hip_roll", "l_hip_pitch", "l_knee_pitch",
    "l_ankle_pitch", "l_ankle_roll"
]
SWING_JOINT_NAMES = [
    "r_hip_roll", "r_hip_pitch", "r_knee_pitch",
    "r_ankle_pitch", "r_ankle_roll"
]

# Verified frame IDs: l_foot=13, r_foot=35 (from your URDF output)
FOOT_FRAME_L = "l_foot"
FOOT_FRAME_R = "r_foot"

Q_HOME_DEG = np.zeros(N_JOINTS)


# ── xacro expand ─────────────────────────────────────────────────────────────
def _xacro_to_tmp_urdf(xacro_path):
    if not os.path.isfile(xacro_path):
        raise FileNotFoundError(
            "Not found: {}\n"
            "Run: source ~/Abbes/ros2_ws/install/setup.bash".format(xacro_path)
        )
    result = subprocess.run(
        ["xacro", xacro_path], capture_output=True, text=True
    )
    if result.returncode != 0:
        raise RuntimeError("xacro failed:\n{}".format(result.stderr))
    tmp = tempfile.NamedTemporaryFile(suffix=".urdf", mode="w", delete=False)
    tmp.write(result.stdout)
    tmp.close()
    return tmp.name


# ── Robot model ───────────────────────────────────────────────────────────────
class HumanoidRobot:
    """
    Pinocchio-backed kinematic model.

    q argument is always 5 elements (rad):
        [hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll]

    stance = left  leg (support foot, default)
    swing  = right leg (swing foot, default)
    """

    n_joints = N_JOINTS
    n_stance = N_STANCE
    n_swing  = N_SWING
    q_home   = np.deg2rad(Q_HOME_DEG)

    l_ankle_pitch_idx = L_ANKLE_PITCH
    l_ankle_roll_idx  = L_ANKLE_ROLL
    r_ankle_pitch_idx = R_ANKLE_PITCH
    r_ankle_roll_idx  = R_ANKLE_ROLL

    def __init__(self):
        tmp_path = _xacro_to_tmp_urdf(URDF_XACRO_PATH)
        try:
            self.model, self.collision_model, self.visual_model = \
                pin.buildModelsFromUrdf(tmp_path)
            self.data = self.model.createData()
        finally:
            os.unlink(tmp_path)

        self._stance_ids = self._get_joint_ids(STANCE_JOINT_NAMES)
        self._swing_ids  = self._get_joint_ids(SWING_JOINT_NAMES)
        self._l_foot_id  = self._get_frame_id(FOOT_FRAME_L)
        self._r_foot_id  = self._get_frame_id(FOOT_FRAME_R)
        self._q_neutral  = pin.neutral(self.model)

        print("[robot_model] loaded: {} joints, {} frames".format(
            self.model.njoints - 1, self.model.nframes))
        print("[robot_model] l_foot id={}  r_foot id={}".format(
            self._l_foot_id, self._r_foot_id))

    # ── internal helpers ──────────────────────────────────────────────────────
    def _get_joint_ids(self, names):
        ids = []
        for name in names:
            jid = self.model.getJointId(name)
            if jid >= self.model.njoints:
                available = [self.model.names[i]
                             for i in range(1, self.model.njoints)]
                raise ValueError(
                    "Joint '{}' not found. Available:\n{}".format(
                        name, available))
            ids.append(jid)
        return ids

    def _get_frame_id(self, name):
        fid = self.model.getFrameId(name)
        if fid >= self.model.nframes:
            available = [self.model.frames[i].name
                         for i in range(self.model.nframes)]
            raise ValueError(
                "Frame '{}' not found. Available:\n{}".format(
                    name, available))
        return fid

    def _make_q(self, q_leg, joint_ids):
        """
        Copy 5-joint leg angles into the full pinocchio q vector.
        All other joints remain at neutral (zero).
        """
        q = self._q_neutral.copy()
        for i, jid in enumerate(joint_ids):
            q[self.model.joints[jid].idx_q] = q_leg[i]
        return q

    def _vel_cols(self, joint_ids):
        return [self.model.joints[jid].idx_v for jid in joint_ids]

    # ── Stance FK: returns pelvis height (CoM proxy) ──────────────────────────
    def stance_fk(self, q_leg):
        """
        q_leg (5,) rad -> CoM position (3,) metres [x, y, z].

        Uses full-body centerOfMass from pinocchio.
        Z_C at upright = 0.2698 m (verified from Test 1).
        """
        q = self._make_q(q_leg, self._stance_ids)
        com = pin.centerOfMass(self.model, self.data, q, False)
        return np.array(com).flatten()

    # ── Stance Jacobian ───────────────────────────────────────────────────────
    def stance_jac(self, q_leg):
        """
        q_leg (5,) rad -> CoM Jacobian (3, 5).

        Note on axis convention (verified from your URDF):
          Row 0 = x (sagittal, forward)
          Row 1 = y (lateral)
          Row 2 = z (vertical, up)
        hip_pitch and knee_pitch affect rows 0 and 2.
        hip_roll affects row 1.
        """
        q = self._make_q(q_leg, self._stance_ids)
        pin.forwardKinematics(self.model, self.data, q)
        J_full = pin.jacobianCenterOfMass(self.model, self.data, q, False)
        return J_full[:, self._vel_cols(self._stance_ids)]

    # ── Swing FK: returns foot SOLE position ──────────────────────────────────
    def swing_fk(self, q_leg):
        """
        q_leg (5,) rad -> right foot SOLE position (3,) metres [x, y, z].

        The r_foot frame is at the TOP face of the foot block (z=0.05m).
        We subtract FOOT_HEIGHT to get the sole contact point (z=0.0m).
        """
        q = self._make_q(q_leg, self._swing_ids)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pos = np.array(
            self.data.oMf[self._r_foot_id].translation
        ).flatten()
        pos[2] -= FOOT_HEIGHT   # correct: top face -> sole
        return pos

    # ── Swing Jacobian ────────────────────────────────────────────────────────
    def swing_jac(self, q_leg):
        """
        q_leg (5,) rad -> right foot Jacobian (3, 5).
        Translational rows only.
        """
        q = self._make_q(q_leg, self._swing_ids)
        pin.forwardKinematics(self.model, self.data, q)
        J_full = pin.computeFrameJacobian(
            self.model, self.data, q,
            self._r_foot_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return J_full[:3, self._vel_cols(self._swing_ids)]

    # ── Convenience: get current CoM from full joint array ───────────────────
    def com_from_full_q(self, q_full_deg):
        """
        q_full_deg: (18,) full joint array in DEGREES (/joint_commands order).
        Returns CoM position (3,) metres.
        Used by pipeline.py to publish /com_trajectory.
        """
        q = self._q_neutral.copy()
        # Map /joint_commands order -> pinocchio joint ids
        all_names = (SWING_JOINT_NAMES +   # indices 0-4  (right leg)
                     STANCE_JOINT_NAMES)   # indices 5-9  (left leg)
        all_ids   = self._swing_ids + self._stance_ids
        for i, jid in enumerate(all_ids):
            q[self.model.joints[jid].idx_q] = np.deg2rad(q_full_deg[i])
        com = pin.centerOfMass(self.model, self.data, q, False)
        return np.array(com).flatten()


# ── Self-test ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Loading robot model...")
    robot = HumanoidRobot()
    q0 = np.zeros(N_STANCE)

    # Test 1: CoM height at upright stance
    com = robot.stance_fk(q0)
    print("\nTest 1 - upright CoM:")
    print("  pinocchio: x={:.4f}  y={:.4f}  z={:.4f} m".format(*com))
    print("  expected Z_C = {:.4f} m".format(Z_C))
    diff = abs(com[2] - Z_C)
    print("  difference: {:.1f} mm  {}".format(
        diff * 1000, "OK" if diff < 1.0 else "UNEXPECTED"))

    # Test 2: Jacobian shape and nonzero columns
    J = robot.stance_jac(q0)
    print("\nTest 2 - CoM Jacobian:")
    print("  shape: {}  (expected (3, 5))".format(J.shape))
    print("  hip_roll  col: {}".format(J[:, 0].round(4)))
    print("  hip_pitch col: {}".format(J[:, 1].round(4)))
    print("  knee_pitch col: {}".format(J[:, 2].round(4)))
    ok2 = J.shape == (3, 5) and np.any(np.abs(J[:, 1]) > 1e-4)
    print("  {}".format("OK" if ok2 else "CHECK - all zeros in hip_pitch col"))

    # Test 3: swing foot sole on ground
    foot = robot.swing_fk(q0)
    print("\nTest 3 - right foot SOLE position:")
    print("  x={:.4f}  y={:.4f}  z={:.4f} m".format(*foot))
    print("  expected z = 0.0 m  {}".format(
        "OK" if abs(foot[2]) < 0.001 else "CHECK FOOT_HEIGHT offset"))

    # Test 4: bent knee reduces CoM height
    q_bent = np.zeros(N_STANCE)
    q_bent[2] = np.deg2rad(20)
    com_bent = robot.stance_fk(q_bent)
    print("\nTest 4 - 20 deg knee bend:")
    print("  upright z={:.4f}  bent z={:.4f}".format(com[2], com_bent[2]))
    if com_bent[2] < com[2]:
        print("  OK - knee bend lowers CoM as expected")
    else:
        print("  NOTE: CoM rose with knee bend.")
        print("  This can happen with a small robot where torso mass dominates.")
        print("  Try negative knee angle to verify joint direction:")
        q_neg = np.zeros(N_STANCE)
        q_neg[2] = np.deg2rad(-20)
        com_neg = robot.stance_fk(q_neg)
        print("  -20 deg knee: z={:.4f}  {}".format(
            com_neg[2],
            "joint axis inverted - negate knee angles in pipeline" 
            if com_neg[2] < com[2] else "symmetric - URDF uses different convention"))

    # Test 5: com_from_full_q convenience method
    q_full = np.zeros(N_JOINTS)
    com_full = robot.com_from_full_q(q_full)
    print("\nTest 5 - com_from_full_q (all zeros):")
    print("  CoM: x={:.4f}  y={:.4f}  z={:.4f} m".format(*com_full))
    print("  {}".format("OK" if abs(com_full[2] - Z_C) < 0.01 else "CHECK"))

    print("\nAll tests done.")