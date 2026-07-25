#!/usr/bin/env python3
"""imu_propagation.py — PREDICT step of the base-state estimator (Phase 1).
Strapdown IMU integration (orientation/velocity/position) with a startup-
transient guard that rejects physically-impossible samples (the M0 -169.8 deg
failure mode). Pure Python so it is testable anywhere.
Frames: q = world<-body unit quaternion (w,x,y,z); v,p world; a_world =
R(q)*a_body - (0,0,g)."""
import math
G = 9.81

def _norm(v): return math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])
def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def _qmul(a, b):
    w1,x1,y1,z1 = a; w2,x2,y2,z2 = b
    return (w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
            w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2)
def _qnorm(q):
    n = math.sqrt(sum(c*c for c in q)) or 1.0
    return tuple(c/n for c in q)
def _q_from_omega(gyro, dt):
    th = (gyro[0]*dt, gyro[1]*dt, gyro[2]*dt); a = _norm(th)
    if a < 1e-9: return _qnorm((1.0, th[0]/2, th[1]/2, th[2]/2))
    s = math.sin(a/2)/a
    return (math.cos(a/2), th[0]*s, th[1]*s, th[2]*s)
def _qrotate(q, v):
    qv = (q[1], q[2], q[3]); t = _cross(qv, v); t = (2*t[0], 2*t[1], 2*t[2])
    c = _cross(qv, t)
    return (v[0]+q[0]*t[0]+c[0], v[1]+q[0]*t[1]+c[1], v[2]+q[0]*t[2]+c[2])
def quat_to_euler(q):
    w,x,y,z = q
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    sp = max(-1.0, min(1.0, 2*(w*y-z*x))); pitch = math.asin(sp)
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw

class ImuPropagator:
    def __init__(self, max_gyro=25.0, accel_band=(0.5, 2.0), n_warmup=10):
        self.max_gyro = max_gyro; self.accel_band = accel_band
        self.n_warmup = n_warmup; self.reset()
    def reset(self):
        self.q = (1.0,0.0,0.0,0.0); self.v = (0.0,0.0,0.0); self.p = (0.0,0.0,0.0)
        self._good = 0; self.rejected = 0; self.valid = False
    def _plausible(self, gyro, accel):
        amag = _norm(accel)/G
        if amag < self.accel_band[0] or amag > self.accel_band[1]: return False
        if _norm(gyro) > self.max_gyro: return False
        return True
    def predict(self, gyro, accel, dt):
        if not self._plausible(gyro, accel):
            self.rejected += 1; return False
        self.q = _qnorm(_qmul(self.q, _q_from_omega(gyro, dt)))
        aw = _qrotate(self.q, accel); aw = (aw[0], aw[1], aw[2]-G)
        self.v = (self.v[0]+aw[0]*dt, self.v[1]+aw[1]*dt, self.v[2]+aw[2]*dt)
        self.p = (self.p[0]+self.v[0]*dt, self.p[1]+self.v[1]*dt, self.p[2]+self.v[2]*dt)
        self._good += 1
        if self._good >= self.n_warmup: self.valid = True
        return True
