#!/usr/bin/env python3
"""base_estimator.py — CORRECT step: contact-aided base estimation (Phase 1).
A planted foot is fixed in the world: p_foot_world = p_base + R*f(q) = const.
Differentiating gives a base-velocity measurement from the legs:
    v_base = -w x (R*f) - R*(df/dt)
and a height measurement (planted sole on ground): z_base = -[R*f_sole]_z.
Blended into the IMU-propagated state with fixed gains kv, kz. This is where the
base-velocity signal (that the reactive stack lacked) comes from. Fixed gains =
steady-state simplification of the InEKF; measurement model is the real one."""
from humanoid_wbc.estimation.imu_propagation import ImuPropagator, _qrotate
from humanoid_wbc.estimation.leg_kinematics import foot_position

def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

class BaseEstimator:
    def __init__(self, kv=0.5, kz=0.5):
        self.prop = ImuPropagator(); self.kv = kv; self.kz = kz
        self._prev = {'r': None, 'l': None}
    @property
    def valid(self): return self.prop.valid
    @property
    def velocity(self): return self.prop.v
    @property
    def position(self): return self.prop.p
    @property
    def orientation(self): return self.prop.q
    def step(self, gyro, accel, dt, q_r, q_l, contacts):
        self.prop.predict(gyro, accel, dt)
        q = self.prop.q
        omega_w = _qrotate(q, gyro)
        v_meas, z_meas = [], []
        for side, c in (('r', contacts.right), ('l', contacts.left)):
            f = foot_position(side, q_r if side == 'r' else q_l, sole=True)
            Rf = _qrotate(q, f)
            if c:
                z_meas.append(-Rf[2])
                prev = self._prev[side]
                if prev is not None:
                    fdot = ((f[0]-prev[0])/dt, (f[1]-prev[1])/dt, (f[2]-prev[2])/dt)
                    Rfdot = _qrotate(q, fdot); wxRf = _cross(omega_w, Rf)
                    v_meas.append((-wxRf[0]-Rfdot[0], -wxRf[1]-Rfdot[1], -wxRf[2]-Rfdot[2]))
                self._prev[side] = f
            else:
                self._prev[side] = None
        if v_meas:
            vm = tuple(sum(v[i] for v in v_meas)/len(v_meas) for i in range(3))
            v = self.prop.v
            self.prop.v = tuple(v[i] + self.kv*(vm[i]-v[i]) for i in range(3))
        if z_meas:
            zm = sum(z_meas)/len(z_meas); p = self.prop.p
            self.prop.p = (p[0], p[1], p[2] + self.kz*(zm - p[2]))
