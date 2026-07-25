#!/usr/bin/env python3
"""March in place: alternate weight-shift + brief foot lift (dynamic stepping)."""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Bool

DT          = 0.005
Z_C         = 0.22
Z_SPAWN     = 0.30
FOOT_HEIGHT = 0.05
HIP_Y       = 0.04
LIFT_Z      = 0.02      # foot lift height (dynamic — returns each step)
COM_SHIFT_Y = 0.045     # CoM shift over the stance foot

N_INIT  = int(8.0 / DT)   # crouch
N_SHIFT = int(1.5 / DT)   # weight transfer (double support)
N_SWING = int(1.2 / DT)   # foot lift + return (single support)


def smooth(s):
    s = max(0.0, min(1.0, s))
    return 10*s**3 - 15*s**4 + 6*s**5


class MarchNode(Node):
    def __init__(self):
        super().__init__('zmp_trajectory_node')
        self._state   = 'init'
        self._phase_t = 0
        self._com_y   = 0.0
        self._com_y0  = 0.0
        self._stance  = 'R'

        qos = rclpy.qos.QoSProfile(depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self._com_pub  = self.create_publisher(PoseStamped, '/com_trajectory', qos)
        self._foot_pub = self.create_publisher(PoseArray,   '/foot_trajectory', qos)
        self._ss_pub   = self.create_publisher(Bool, '/single_support', 10)
        self.create_timer(DT, self._tick)
        self.get_logger().info('March node ready')

    def _tick(self):
        self._phase_t += 1

        if self._state == 'init':
            frac = self._phase_t / N_INIT
            z = Z_SPAWN - (Z_SPAWN - Z_C) * frac
            self._pub(0.0, FOOT_HEIGHT, FOOT_HEIGHT, z, False)
            if self._phase_t >= N_INIT:
                self._begin_shift('R')
                self.get_logger().info('Crouch done — marching')
            return

        if self._state == 'shift':          # move weight over stance foot, both feet down
            frac   = smooth(self._phase_t / N_SHIFT)
            target = COM_SHIFT_Y if self._stance == 'R' else -COM_SHIFT_Y
            self._com_y = self._com_y0 + (target - self._com_y0) * frac
            self._pub(self._com_y, FOOT_HEIGHT, FOOT_HEIGHT, Z_C, False)
            if self._phase_t >= N_SHIFT:
                self._state = 'swing'; self._phase_t = 0
            return

        if self._state == 'swing':          # lift the NON-stance foot up and back down
            frac = self._phase_t / N_SWING
            lift = LIFT_Z * np.sin(np.pi * min(frac, 1.0))
            if self._stance == 'R':
                r_z, l_z = FOOT_HEIGHT, FOOT_HEIGHT + lift
            else:
                r_z, l_z = FOOT_HEIGHT + lift, FOOT_HEIGHT
            self._pub(self._com_y, r_z, l_z, Z_C, True)
            if self._phase_t >= N_SWING:
                self._begin_shift('L' if self._stance == 'R' else 'R')
            return

    def _begin_shift(self, stance):
        self._state   = 'shift'
        self._phase_t = 0
        self._com_y0  = self._com_y
        self._stance  = stance

    def _pub(self, com_y, r_z, l_z, com_z, single_support):
        com = PoseStamped(); com.header.frame_id = 'world'
        com.pose.position.x = 0.0
        com.pose.position.y = com_y
        com.pose.position.z = com_z
        self._com_pub.publish(com)
        r, l = Pose(), Pose()
        r.position.y =  HIP_Y; r.position.z = r_z
        l.position.y = -HIP_Y; l.position.z = l_z
        fa = PoseArray(); fa.header.frame_id = 'world'; fa.poses = [r, l]
        self._foot_pub.publish(fa)
        ss = Bool(); ss.data = single_support
        self._ss_pub.publish(ss)


def main(args=None):
    rclpy.init(args=args)
    node = MarchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()