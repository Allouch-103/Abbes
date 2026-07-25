import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ball_interfaces.msg import BallState

class BallStatePublisher(Node):
    def __init__(self):
        super().__init__('ball_state_publisher')
        self.sub = self.create_subscription(
            Odometry, '/model/sim_ball/odometry', self.odom_cb, 10)
        self.pub = self.create_publisher(BallState, 'ball/state', 10)
        self.get_logger().info('ball_state_publisher started')

    def odom_cb(self, msg: Odometry):
        state = BallState()
        state.position = msg.pose.pose.position
        state.velocity = msg.twist.twist.linear
        speed = (state.velocity.x**2 + state.velocity.y**2) ** 0.5
        state.is_moving = speed > 0.01
        self.pub.publish(state)

def main():
    rclpy.init()
    node = BallStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
