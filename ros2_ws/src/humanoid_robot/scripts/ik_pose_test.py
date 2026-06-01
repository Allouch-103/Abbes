#!/usr/bin/env python3
"""
ik_pose_test.py — IK sign-verification tool.

Publishes a SLOW cycle of clearly-named STATIC targets on /com_trajectory +
/foot_trajectory (the same topics ik_vectors_DLS consumes), so you can watch
the robot in Gazebo and report which pose looks wrong. This isolates the IK
joint-sign correctness from the gait dynamics and from balance.

Frame (matches pipeline.py / ik_vectors_DLS):
  /com_trajectory  PoseStamped : CoM position in world (x fwd, y, z up)
  /foot_trajectory PoseArray   : [right foot, left foot] world positions
  right foot sits at y = +0.04, left foot at y = -0.04, ground = z 0.

Run (balance OFF, no pipeline):
  T1: ros2 launch my_robot_bringup my_robot_gazebo.launch.py
  T2: ros2 launch humanoid_robot ik_test.launch.py

Each pose holds for HOLD_S seconds and logs what you SHOULD see.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

Z = 0.2698        # standing CoM height (= robot_params com_height_m)
HW = 0.04         # half hip width (foot y offset)
HOLD_S = 5.0      # seconds per pose

# (name, com[x,y,z], right_foot[x,y,z], left_foot[x,y,z], "what you should see")
POSES = [
    ("STAND",
     (0.0, 0.0, Z), (0.0, +HW, 0.0), (0.0, -HW, 0.0),
     "straight standing, feet flat on ground"),
    ("BENT",
     (0.0, 0.0, 0.20), (0.0, +HW, 0.0), (0.0, -HW, 0.0),
     "a DEEP ~70deg knee-bend crouch (closed-form IK = deterministic, reliable). "
     "May wobble/slowly tip (CoM offset not perfectly tuned for this depth) but "
     "the KNEE SHAPE is reliable — view from the side to judge fwd/back."),
    ("SQUAT",
     (0.0, 0.0, 0.21), (0.0, +HW, 0.0), (0.0, -HW, 0.0),
     "both knees BEND and hips sink straight down, feet stay flat "
     "(knees should fold like a human, NOT hyperextend backward)"),
    ("LEAN_FORWARD",
     (0.04, 0.0, Z), (0.0, +HW, 0.0), (0.0, -HW, 0.0),
     "whole body leans FORWARD over the toes, feet stay flat"),
    ("WEIGHT_RIGHT",
     (0.0, +HW, Z), (0.0, +HW, 0.0), (0.0, -HW, 0.0),
     "body shifts sideways over the RIGHT foot (the y=+ side)"),
    ("LIFT_LEFT_FOOT",
     (0.0, +HW, Z), (0.0, +HW, 0.0), (0.0, -HW, 0.04),
     "weight on right leg, LEFT foot lifts ~4cm off the ground "
     "(this is the core stepping move)"),
]


import time as _time

RAMP_S = 3.0   # seconds to EASE from standing into the held pose (sim snaps joints,
               # so a step command lurches the robot over; ramp the CoM/feet down)


class IKPoseTest(Node):
    def __init__(self):
        super().__init__('ik_pose_test')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.com_pub = self.create_publisher(PoseStamped, '/com_trajectory', qos)
        self.foot_pub = self.create_publisher(PoseArray, '/foot_trajectory', qos)
        self.i = -1
        self.hold_mode = False
        self.t0 = _time.time()
        self.create_timer(0.02, self._publish)      # 50 Hz stream of current pose
        # 'hold' param: if set to a pose name (e.g. BENT), EASE into that pose over
        # RAMP_S then hold it — a static stance to tune balance against.
        self.declare_parameter('hold', '')
        hold = str(self.get_parameter('hold').value).strip().upper()
        names = [p[0] for p in POSES]
        if hold in names:
            self.i = names.index(hold)
            self.hold_mode = True
            self.get_logger().info(
                f"HOLDING pose: {hold} (easing in over {RAMP_S}s) — {POSES[self.i][4]}")
        else:
            self.create_timer(HOLD_S, self._next)   # cycle every HOLD_S
            self._next()

    def _next(self):
        self.i = (self.i + 1) % len(POSES)
        name, _, _, _, expect = POSES[self.i]
        self.get_logger().info(f"\n===== POSE: {name} =====\n  EXPECT: {expect}")

    def _publish(self):
        _, com, rf, lf, _ = POSES[self.i]
        # In hold mode, ramp from the STAND pose (index 0) into the target so the
        # robot eases down instead of the sim snapping the joints and tipping.
        if self.hold_mode:
            a = min(1.0, (_time.time() - self.t0) / RAMP_S)
            s_com, s_rf, s_lf = POSES[0][1], POSES[0][2], POSES[0][3]
            com = tuple(s + a * (t - s) for s, t in zip(s_com, com))
            rf  = tuple(s + a * (t - s) for s, t in zip(s_rf, rf))
            lf  = tuple(s + a * (t - s) for s, t in zip(s_lf, lf))
        m = PoseStamped(); m.header.frame_id = 'world'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = com
        self.com_pub.publish(m)
        fa = PoseArray(); fa.header.frame_id = 'world'
        r, l = Pose(), Pose()
        r.position.x, r.position.y, r.position.z = rf
        l.position.x, l.position.y, l.position.z = lf
        fa.poses = [r, l]
        self.foot_pub.publish(fa)


def main():
    rclpy.init()
    try:
        rclpy.spin(IKPoseTest())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
