#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import math

class WalkTest(Node):
    def __init__(self):
        super().__init__('walk_test')
        qos = rclpy.qos.QoSProfile(depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.com_pub  = self.create_publisher(PoseStamped, '/com_trajectory', qos)
        self.foot_pub = self.create_publisher(PoseArray,   '/foot_trajectory', qos)
        
        self.t = 0.0
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info('Walk test started')

    def tick(self):
        self.t += self.dt
        
        com_y = 0.02 * math.sin(self.t * 0.5)
        com_z = 0.19

        fr_x = 0.04 * math.sin(self.t)
        fr_z = max(0.0, 0.03 * math.sin(self.t))

        fl_x = -0.04 * math.sin(self.t)
        fl_z = max(0.0, 0.03 * math.sin(self.t + math.pi))

        com = PoseStamped()
        com.header.frame_id = 'world'
        com.pose.position.x = 0.0
        com.pose.position.y = com_y
        com.pose.position.z = com_z
        self.com_pub.publish(com)

        feet = PoseArray()
        feet.header.frame_id = 'world'
        r, l = Pose(), Pose()
        r.position.x = fr_x;  r.position.y =  0.04; r.position.z = fr_z
        l.position.x = fl_x;  l.position.y = -0.04; l.position.z = fl_z
        feet.poses = [r, l]
        self.foot_pub.publish(feet)

def main():
    rclpy.init()
    rclpy.spin(WalkTest())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
