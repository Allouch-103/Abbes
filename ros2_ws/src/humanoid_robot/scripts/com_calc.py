#!/usr/bin/env python3
"""Compute the robot's CoM x relative to the foot center, from /tf + URDF masses.
Tells us exactly how far FORWARD the CoM is of the support, so we can set
com_x_offset to cancel it. Uses transforms relative to base_link (robot_state_
publisher provides these from /joint_states — no world pose needed)."""
import time, rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

# link -> mass (kg), from the URDF inertials (mass is at each link's frame origin)
MASS = {
    'torso':0.40,'neck':0.02,'head':0.10,'camera_link':0.02,
    'r_hip_bracket':0.03,'r_thigh':0.12,'r_shank':0.10,'r_ankle_bracket':0.03,'r_foot':0.06,
    'l_hip_bracket':0.03,'l_thigh':0.12,'l_shank':0.10,'l_ankle_bracket':0.03,'l_foot':0.06,
    'r_shoulder_link':0.001,'r_upper_arm':0.08,'r_forearm':0.06,'r_hand':0.03,
    'l_shoulder_link':0.001,'l_upper_arm':0.08,'l_forearm':0.06,'l_hand':0.03,
}

def main():
    rclpy.init(); n = Node('com_calc')
    buf = Buffer(); TransformListener(buf, n)
    t0 = time.time()
    while time.time()-t0 < 4.0:               # let tf buffer fill
        rclpy.spin_once(n, timeout_sec=0.1)
    from rclpy.time import Time
    pos = {}
    for link in list(MASS) + ['r_foot','l_foot']:
        try:
            tf = buf.lookup_transform('base_link', link, Time())
            pos[link] = (tf.transform.translation.x, tf.transform.translation.z)
        except Exception as e:
            print(f"  (no tf for {link}: {e})")
    if 'torso' not in pos:
        print("FAILED: no tf — is the sim + robot_state_publisher running?"); rclpy.shutdown(); return
    M = sum(MASS[k] for k in MASS if k in pos)
    com_x = sum(MASS[k]*pos[k][0] for k in MASS if k in pos) / M
    foot_x = 0.5*(pos['r_foot'][0] + pos['l_foot'][0]) if 'r_foot' in pos and 'l_foot' in pos else 0.0
    excess = com_x - foot_x
    print(f"\nCoM x (rel base_link)   = {com_x:+.4f} m")
    print(f"foot center x           = {foot_x:+.4f} m")
    print(f"CoM is {excess:+.4f} m {'FORWARD' if excess>0 else 'behind'} the feet")
    print(f"\n=> to center it, change com_x_offset by about {-excess:+.4f} m")
    rclpy.shutdown()

main()
