#!/usr/bin/env python3
"""estimator_node.py — ROS 2 node wrapping BaseEstimator (Phase 1).
Sub /imu + /joint_states -> pub /base_state (humanoid_msgs/BaseState).
Runs predict+correct once per IMU msg. Contact = both-down default (standing);
gait layer will publish contact later. Gazebo Imu.linear_acceleration is specific
force (includes +g), which ImuPropagator expects."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from humanoid_msgs.msg import BaseState
from humanoid_wbc.estimation.base_estimator import BaseEstimator
from humanoid_wbc.estimation.contact_schedule import Contacts
from humanoid_wbc.estimation.joint_map import legs_from_jointstate

NOMINAL_DT = 0.005

class EstimatorNode(Node):
    def __init__(self):
        super().__init__('base_estimator')
        self.est = BaseEstimator()
        self._names = []; self._positions = []
        self._last_t = None; self._last_gyro = (0.0, 0.0, 0.0)
        be = QoSProfile(depth=10); be.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, be)
        self._pub = self.create_publisher(BaseState, '/base_state', 10)
        self.get_logger().info('Base estimator up — /imu + /joint_states -> /base_state')

    def _js_cb(self, msg):
        self._names = list(msg.name); self._positions = list(msg.position)

    def _imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_t is None:
            self._last_t = t; return
        dt = t - self._last_t; self._last_t = t
        if dt <= 0.0 or dt > 0.1: dt = NOMINAL_DT
        legs = legs_from_jointstate(self._names, self._positions)
        if legs is None: return
        q_r, q_l = legs
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self._last_gyro = gyro
        contacts = Contacts(left=True, right=True)
        self.est.step(gyro, accel, dt, q_r, q_l, contacts)
        self._publish(msg.header.stamp, contacts)

    def _publish(self, stamp, contacts):
        m = BaseState()
        m.header.stamp = stamp; m.header.frame_id = 'base_link'
        w, x, y, z = self.est.orientation
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = w, x, y, z
        px, py, pz = self.est.position
        m.position.x, m.position.y, m.position.z = px, py, pz
        vx, vy, vz = self.est.velocity
        m.linear_velocity.x, m.linear_velocity.y, m.linear_velocity.z = vx, vy, vz
        gx, gy, gz = self._last_gyro
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gx, gy, gz
        m.left_contact = contacts.left; m.right_contact = contacts.right
        m.valid = self.est.valid
        self._pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = EstimatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
