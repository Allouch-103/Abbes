#!/usr/bin/env python3
"""robot_dyn.py — floating-base dynamics via Pinocchio (Phase 2).
Gives the whole-body QP everything it needs from the URDF: total mass, CoM,
the CENTROIDAL MOMENTUM MATRIX A_G(q) (h_G = A_G @ v — a column per joint, so
arm/torso motion is visible to balance, not just legs), mass matrix, gravity.
Model: floating base + 18 joints -> nq=25, nv=24."""
import os, re, subprocess, tempfile
import numpy as np
import pinocchio as pin

DEFAULT_XACRO = os.path.join(os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "my_robot_description", "urdf", "humanoid.urdf.xacro")

def expand_xacro(xacro_path):
    """Expand .xacro -> plain .urdf temp file. Falls back to resolving
    $(find pkg) from the file's own location when no ROS index is available."""
    xacro_path = os.path.abspath(xacro_path)
    try:
        import xacro
        try:
            urdf_txt = xacro.process_file(xacro_path).toxml()
        except Exception:
            pkg_root = os.path.dirname(os.path.dirname(xacro_path))
            src_root = os.path.dirname(pkg_root)
            txt = open(xacro_path).read()
            def _sub(mm):
                cand = os.path.join(src_root, mm.group(1))
                return cand if os.path.isdir(cand) else pkg_root
            txt = re.sub(r'\$\(find ([^)]+)\)', _sub, txt)
            fd, tmp_x = tempfile.mkstemp(suffix='.xacro', dir=os.path.dirname(xacro_path))
            with os.fdopen(fd, 'w') as f: f.write(txt)
            try: urdf_txt = xacro.process_file(tmp_x).toxml()
            finally: os.remove(tmp_x)
    except ImportError:
        urdf_txt = subprocess.check_output(['xacro', xacro_path]).decode()
    fd, path = tempfile.mkstemp(suffix='.urdf')
    with os.fdopen(fd, 'w') as f: f.write(urdf_txt)
    return path

class RobotDyn:
    def __init__(self, xacro_path=None):
        urdf = expand_xacro(xacro_path or DEFAULT_XACRO)
        self.model = pin.buildModelFromUrdf(urdf, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        os.remove(urdf)
    @property
    def mass(self): return pin.computeTotalMass(self.model)
    def neutral(self): return pin.neutral(self.model)
    def com(self, q): return pin.centerOfMass(self.model, self.data, q)
    def centroidal_momentum_matrix(self, q):
        """A_G(q) (6,nv): h_G = A_G @ v. Rows 0-2 linear, 3-5 angular."""
        return pin.computeCentroidalMap(self.model, self.data, q)
    def centroidal_momentum(self, q, v):
        return self.centroidal_momentum_matrix(q) @ v
    def gravity_torque(self, q):
        return pin.computeGeneralizedGravity(self.model, self.data, q)
    def mass_matrix(self, q):
        return pin.crba(self.model, self.data, q)
