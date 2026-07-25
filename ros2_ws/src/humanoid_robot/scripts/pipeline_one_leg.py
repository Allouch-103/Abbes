#!/usr/bin/env python3
"""One-leg stand with single-support flag for gated hip-roll balance."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Bool

DT           = 0.005
Z_C          = 0.22
Z_SPAWN      = 0.30
FOOT_HEIGHT  = 0.05
HIP_Y        = 0.04
LIFT_Z       = 0.02
COM_STANCE_Y = 0.055
TUCK_Y = 0.015    # raised-foot y pulls in from -0.04 toward centerline


class OneLegNode(Node):
    def __init__(self):
        super().__init__('zmp_trajectory_node')
        self._counter       = 0
        self._init_done     = False
        self._shift_done    = False
        self._shift_counter = 0
        self._lift_counter  = 0
        self._n_init  = int(8.0 / DT)
        self._n_shift = int(3.0 / DT)
        self._n_lift  = int(3.0 / DT)

        qos = rclpy.qos.QoSProfile(depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self._com_pub  = self.create_publisher(PoseStamped, '/com_trajectory', qos)
        self._foot_pub = self.create_publisher(PoseArray,   '/foot_trajectory', qos)
        self._ss_pub   = self.create_publisher(Bool, '/single_support', 10)
        self.create_timer(DT, self._tick)
        self.get_logger().info('One-leg stand node ready')

    def _tick(self):
        # Phase 1: height ramp, both feet down — double support
        if not self._init_done:
            self._counter += 1
            frac  = self._counter / self._n_init
            z_now = Z_SPAWN - (Z_SPAWN - Z_C) * frac
            self._pub(0.0, FOOT_HEIGHT, FOOT_HEIGHT, z_now, False)
            if self._counter >= self._n_init:
                self._init_done = True
                self.get_logger().info('Init complete — starting lateral shift')
            return

        # Phase 2: CoM shifts over stance foot, both feet down — double support
        if not self._shift_done:
            self._shift_counter += 1
            frac  = self._shift_counter / self._n_shift
            com_y = COM_STANCE_Y * frac
            self._pub(com_y, FOOT_HEIGHT, FOOT_HEIGHT, Z_C, False)
            if self._shift_counter >= self._n_shift:
                self._shift_done = True
                self.get_logger().info('Lateral shift complete — starting foot lift')
            return

        # Phase 3: left foot rises — SINGLE SUPPORT
        if self._lift_counter < self._n_lift:
            self._lift_counter += 1
            frac = self._lift_counter / self._n_lift
            l_z  = FOOT_HEIGHT + LIFT_Z * frac
            self._pub(COM_STANCE_Y, FOOT_HEIGHT, l_z, Z_C, True)
            if self._lift_counter >= self._n_lift:
                self.get_logger().info('Foot lift complete — one-leg hold')
            return

        # Phase 4: hold — SINGLE SUPPORT
        self._pub(COM_STANCE_Y, FOOT_HEIGHT, FOOT_HEIGHT + LIFT_Z, Z_C, True)

    def _pub(self, com_y, r_z, l_z, com_z, single_support,l_y=-HIP_Y):
        com_msg = PoseStamped()
        com_msg.header.frame_id = 'world'
        com_msg.pose.position.x = 0.0
        com_msg.pose.position.y = com_y
        com_msg.pose.position.z = com_z
        self._com_pub.publish(com_msg)

        r, l = Pose(), Pose()
        r.position.y =  HIP_Y;  r.position.z = r_z
        l.position.y = l_y;  l.position.z = l_z
        feet_msg = PoseArray()
        feet_msg.header.frame_id = 'world'
        feet_msg.poses = [r, l]
        self._foot_pub.publish(feet_msg)

        ss = Bool(); ss.data = single_support
        self._ss_pub.publish(ss)


def main(args=None):
    rclpy.init(args=args)
    node = OneLegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()