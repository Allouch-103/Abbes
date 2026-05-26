"""
test_motions.py
===============

Standalone tester. NOT part of the final pipeline — only used in
Pass 2 of Step 2.B to visually verify that each motion primitive
looks correct in Gazebo before we wrap them in action servers.

Usage
-----
    ros2 run humanoid_command_api test_motions <motion> [arg]

Available motions:
    home            return to HOME pose
    sit             lower into sitting pose
    bow             bow forward, hold, return
    wave right      wave right hand
    wave left       wave left hand
    march <N>       march in place N steps (default 4)

The script publishes one pose per tick to:
    /leg_controller/commands
    /arm_controller/commands
    /head_controller/commands
At RATE_HZ (50 Hz). When the trajectory is exhausted, the node exits.

Important: this assumes the robot's current pose is HOME at startup.
If it isn't, the first interpolation will snap from HOME to the first
trajectory pose, which may look discontinuous. To reset between tests,
run `... test_motions home` first.
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .pose_definitions import HOME_POSE, pose_to_arrays
from .motion_primitives import (
    RATE_HZ,
    traj_stand_still, traj_sit_down, traj_bow,
    traj_wave, traj_march_in_place,
)


class MotionTester(Node):
    def __init__(self, motion: str, arg: str | None):
        super().__init__("motion_tester")

        # ── publishers, matching controllers.yaml ──────────────
        self.leg_pub  = self.create_publisher(
            Float64MultiArray, "/leg_controller/commands",  10)
        self.arm_pub  = self.create_publisher(
            Float64MultiArray, "/arm_controller/commands",  10)
        self.head_pub = self.create_publisher(
            Float64MultiArray, "/head_controller/commands", 10)

        # ── build the trajectory ───────────────────────────────
        self.traj = self._build_trajectory(motion, arg)
        if not self.traj:
            self.get_logger().error(f"empty trajectory for motion={motion!r}")
            rclpy.shutdown()
            return

        self.idx = 0
        self.get_logger().info(
            f"motion '{motion}' → {len(self.traj)} poses "
            f"≈ {len(self.traj) / RATE_HZ:.1f}s")

        # ── start the publishing timer ─────────────────────────
        self.create_timer(1.0 / RATE_HZ, self._tick)

    def _build_trajectory(self, motion: str, arg: str | None) -> list:
        if motion == "home":
            return traj_stand_still(HOME_POSE)
        if motion == "sit":
            return traj_sit_down(HOME_POSE)
        if motion == "bow":
            return traj_bow(HOME_POSE)
        if motion == "wave":
            side = arg or "right"
            return traj_wave(side, HOME_POSE)
        if motion == "march":
            n = int(arg) if arg else 4
            return traj_march_in_place(num_steps=n)
        self.get_logger().error(f"unknown motion: {motion!r}")
        return []

    def _tick(self):
        if self.idx >= len(self.traj):
            self.get_logger().info("trajectory complete, shutting down")
            rclpy.shutdown()
            return

        pose = self.traj[self.idx]
        leg, arm, head = pose_to_arrays(pose)
        self.leg_pub.publish(Float64MultiArray(data=leg))
        self.arm_pub.publish(Float64MultiArray(data=arm))
        self.head_pub.publish(Float64MultiArray(data=head))

        # Log progress every second
        if self.idx % RATE_HZ == 0:
            pct = 100.0 * self.idx / len(self.traj)
            self.get_logger().info(f"  progress {pct:5.1f}%")

        self.idx += 1


def main():
    if len(sys.argv) < 2:
        print("usage: ros2 run humanoid_command_api test_motions "
              "<home|sit|bow|wave|march> [arg]")
        sys.exit(1)

    motion = sys.argv[1]
    arg = sys.argv[2] if len(sys.argv) > 2 else None

    rclpy.init()
    node = MotionTester(motion, arg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
