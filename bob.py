#!/usr/bin/env python3
"""Gentle vertical CoM bob (double support) to move the base and validate the estimator."""
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
DT=0.005; Z_SPAWN=0.30; Z_MID=0.27; Z_AMP=0.025; FREQ=0.3; FOOT_HEIGHT=0.05; HIP_Y=0.04
class Bob(Node):
    def __init__(self):
        super().__init__('bob_node')
        self._c=0; self._n=int(2.0/DT); self._init=False; self._t=0.0
        q=rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self._com=self.create_publisher(PoseStamped,'/com_trajectory',q)
        self._foot=self.create_publisher(PoseArray,'/foot_trajectory',q)
        self.create_timer(DT,self._tick)
        self.get_logger().info('bob: ease to crouch, then vertical bob')
    def _tick(self):
        if not self._init:
            self._c+=1; f=self._c/self._n
            self._pub(Z_SPAWN-(Z_SPAWN-Z_MID)*f)
            if self._c>=self._n: self._init=True; self.get_logger().info('bobbing now')
            return
        self._t+=DT
        self._pub(Z_MID+Z_AMP*math.sin(2*math.pi*FREQ*self._t))
    def _pub(self,z):
        c=PoseStamped(); c.header.frame_id='world'; c.pose.position.z=z
        self._com.publish(c)
        r,l=Pose(),Pose()
        r.position.y=HIP_Y; r.position.z=FOOT_HEIGHT
        l.position.y=-HIP_Y; l.position.z=FOOT_HEIGHT
        fa=PoseArray(); fa.header.frame_id='world'; fa.poses=[r,l]
        self._foot.publish(fa)
def main():
    rclpy.init(); n=Bob()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
