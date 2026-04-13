#!/usr/bin/env python3
"""
test_ik.py — send static CoM + foot positions, watch /joint_commands

Run this AFTER starting the IK node to verify it computes correct angles
before connecting the real gait generator.

Usage:
    python3 test_ik.py [test_name]

Tests:
    standing    — feet directly below hips, expect ~100° knee
    step_right  — right foot forward 4cm, verify hip/knee/ankle change
    lean_left   — CoM 3cm left, verify hip_roll/ankle_roll change
    reachable   — foot beyond workspace, verify clamping warning

Requirements:
    pip3 install rclpy (or source ROS2 setup.bash)
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Float32MultiArray
import math


JOINT_NAMES = [
    "r_shoulder_pitch", "r_shoulder_roll", "r_elbow_roll",
    "head_yaw", "camera_pitch",
    "r_hip_roll", "r_hip_pitch", "r_knee_pitch", "r_ankle_pitch", "r_ankle_roll",
    "l_shoulder_pitch", "l_shoulder_roll", "l_elbow_roll",
    "l_hip_roll", "l_hip_pitch", "l_knee_pitch", "l_ankle_pitch", "l_ankle_roll",
]


class IKTester(Node):
    def __init__(self, test_name: str):
        super().__init__("ik_tester")
        self.test_name = test_name
        self.received = False

        # Publishers
        self.com_pub = self.create_publisher(PoseStamped, "/com_trajectory", 10)
        self.foot_pub = self.create_publisher(PoseArray, "/foot_trajectory", 10)

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            "/joint_commands",
            self.cmd_cb,
            10,
        )

        # Send test after 0.5s to let IK node start
        self.timer = self.create_timer(0.5, self.send_test)
        self.get_logger().info(f"IK tester started, test: {test_name}")

    def send_test(self):
        self.timer.cancel()

        com_msg = PoseStamped()
        foot_msg = PoseArray()

        HIP_W = 0.04  # half hip width
        COM_H = 0.19

        if self.test_name == "standing":
            # Feet directly below hips — expect q_knee ≈ 101°, q_hip ≈ 0°
            com_msg.pose.position.x = 0.0
            com_msg.pose.position.y = 0.0
            com_msg.pose.position.z = COM_H

            r = Pose(); r.position.x = 0.0; r.position.y = +HIP_W; r.position.z = 0.0
            l = Pose(); l.position.x = 0.0; l.position.y = -HIP_W; l.position.z = 0.0

        elif self.test_name == "step_right":
            # Right foot 4cm forward — expect r_hip_pitch > 80°
            com_msg.pose.position.z = COM_H

            r = Pose(); r.position.x = 0.04; r.position.y = +HIP_W; r.position.z = 0.0
            l = Pose(); l.position.x = 0.0;  l.position.y = -HIP_W; l.position.z = 0.0

        elif self.test_name == "lean_left":
            # CoM shifts 3cm left — expect hip_roll change
            com_msg.pose.position.y = -0.03
            com_msg.pose.position.z = COM_H

            r = Pose(); r.position.x = 0.0; r.position.y = +HIP_W; r.position.z = 0.0
            l = Pose(); l.position.x = 0.0; l.position.y = -HIP_W; l.position.z = 0.0

        elif self.test_name == "reachable":
            # Foot 30cm forward — beyond workspace, should clamp + warn
            com_msg.pose.position.z = COM_H

            r = Pose(); r.position.x = 0.30; r.position.y = +HIP_W; r.position.z = 0.0
            l = Pose(); l.position.x = 0.0;  l.position.y = -HIP_W; l.position.z = 0.0

        else:
            self.get_logger().error(f"Unknown test: {self.test_name}")
            return

        foot_msg.poses = [r, l]

        # Publish 5 times to ensure delivery
        for _ in range(5):
            self.com_pub.publish(com_msg)
            self.foot_pub.publish(foot_msg)

        self.get_logger().info(f"Published test '{self.test_name}', waiting for /joint_commands...")
        self.get_logger().info("  CoM: (%.3f, %.3f, %.3f)" %
                               (com_msg.pose.position.x,
                                com_msg.pose.position.y,
                                com_msg.pose.position.z))
        self.get_logger().info("  R foot: (%.3f, %.3f, %.3f)" %
                               (r.position.x, r.position.y, r.position.z))

    def cmd_cb(self, msg: Float32MultiArray):
        if self.received:
            return
        self.received = True

        print("\n── /joint_commands received ──────────────────────────")
        for i, (name, val) in enumerate(zip(JOINT_NAMES, msg.data)):
            marker = " ←" if i in (5, 6, 7, 8, 9, 13, 14, 15, 16, 17) else ""
            print(f"  [{i:2d}] {name:<22}  {val:7.2f}°{marker}")

        print("\n── Expected for 'standing' ───────────────────────────")
        print("  r_knee_pitch ≈ 0.0°   (from servo cmd: 100 - ~101 ≈ -1°")
        print("  NOTE: knee_servo = 100 - q_knee_deg")
        print("  q_knee_deg ≈ 101°  →  servo = 100 - 101 = -1° → clamped to 0°")
        print("  q_hip_pitch ≈ 0°   →  servo idx 6 ≈ 80°")
        print("  q_ankle ≈ -101°    →  servo idx 8 ≈ 100 + (-101) = -1° → clamped to 10°")
        print()


def main():
    test = sys.argv[1] if len(sys.argv) > 1 else "standing"
    rclpy.init()
    node = IKTester(test)
    # Spin for 3 seconds to receive response
    import time
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
