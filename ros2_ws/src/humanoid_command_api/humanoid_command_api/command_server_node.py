"""
command_server_node.py
======================

The single node that exposes the robot's command API to anyone above it
(LLM dispatcher, web UI, manual ros2 action send_goal calls). It hosts:

  5 action servers:
    /humanoid/walk         (Walk.action)
    /humanoid/stand_still  (StandStill.action)
    /humanoid/sit_down     (SitDown.action)
    /humanoid/wave_hand    (WaveHand.action)
    /humanoid/bow          (Bow.action)

  2 service servers:
    /humanoid/stop         (Stop.srv)
    /humanoid/get_status   (GetStatus.srv)

It subscribes to:
    /joint_states          to track current pose (for smooth trajectory starts)
    /imu                   to track tilt (for Walk safety and GetStatus)

It publishes to:
    /leg_controller/commands   (Float64MultiArray, 10 values, radians)
    /arm_controller/commands   (Float64MultiArray,  6 values, radians)
    /head_controller/commands  (Float64MultiArray,  2 values, radians)

Design notes
------------
- Mutual exclusion: a single `self.active_action` string tracks what's
  currently running. New goals are rejected if anything else is active,
  except Stop (which cancels then runs).
- Cancellation: every trajectory loop checks `goal_handle.is_cancel_requested`
  between ticks, so Stop and client-initiated cancels both work within
  one control period (~20 ms).
- Smooth restart: trajectories always interpolate from `self.current_pose`
  (the last commanded pose), not from a hardcoded HOME. So if the robot
  is mid-bow when you Wave, the wave starts from the bow pose, not from
  HOME — no snap.
- IMU: roll/pitch are extracted from quaternion. If the IMU isn't
  publishing, GetStatus returns NaN and Walk does not abort on tilt.
"""

import math
import threading
import time
from typing import Callable, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu

from humanoid_msgs.action import Walk, StandStill, SitDown, WaveHand, Bow
from humanoid_msgs.srv import Stop, GetStatus

from .pose_definitions import (
    ALL_JOINTS, LEG_JOINTS, ARM_JOINTS, HEAD_JOINTS,
    HOME_POSE, pose_to_arrays,
)
from .motion_primitives import (
    RATE_HZ,
    traj_stand_still, traj_sit_down, traj_bow,
    traj_wave, traj_march_in_place,
)


# ─────────────────────────────────────────────────────────────
#  Walk-stub parameter
#
#  Until the real ZMP gait generator is online, Walk is faked as
#  "march in place for distance / STEP_LENGTH_M seconds". This is
#  the only line that links the (fake) Walk action to the march
#  primitive — when Phase 5 lands, replace it with a call into
#  the real gait generator.
# ─────────────────────────────────────────────────────────────
STEP_LENGTH_M = 0.3   # rough "distance equivalent" per march step


class CommandServerNode(Node):

    def __init__(self):
        super().__init__("humanoid_command_server")

        # ── State ─────────────────────────────────────────────
        self.current_pose: dict = dict(HOME_POSE)
        self.last_joint_state: Optional[JointState] = None
        self.last_imu: Optional[Imu] = None

        # active_action is the human-readable name of whatever
        # is currently running ("walking", "waving", "idle", ...).
        # Reset to "idle" by execute_trajectory when an action ends.
        self.active_action: str = "idle"

        # Reference to the currently-running goal handle, so Stop can
        # cancel it. None when no action is running.
        self.active_goal_handle = None

        self.stop_requested: bool = False

        # Held while an action runs end-to-end. Stop acquires it
        # briefly to ensure exclusive return-to-home.
        self.action_lock = threading.Lock()

        # ── Callback groups ──────────────────────────────────
        # Subscriptions are reentrant so they keep flowing during
        # action execution. Actions are mutually exclusive
        # against themselves but not against the subscriptions.
        self.sub_cb = ReentrantCallbackGroup()
        self.act_cb = ReentrantCallbackGroup()
        self.srv_cb = MutuallyExclusiveCallbackGroup()

        # ── Publishers ───────────────────────────────────────
        self.leg_pub  = self.create_publisher(
            Float64MultiArray, "/leg_controller/commands",  10)
        self.arm_pub  = self.create_publisher(
            Float64MultiArray, "/arm_controller/commands",  10)
        self.head_pub = self.create_publisher(
            Float64MultiArray, "/head_controller/commands", 10)

        # ── Subscriptions ────────────────────────────────────
        self.create_subscription(
            JointState, "/joint_states",
            self._on_joint_state, 10, callback_group=self.sub_cb)
        self.create_subscription(
            Imu, "/imu",
            self._on_imu, 10, callback_group=self.sub_cb)

        # ── Action servers ───────────────────────────────────
        self.walk_server = ActionServer(
            self, Walk, "/humanoid/walk",
            execute_callback=self._execute_walk,
            goal_callback=self._goal_walk,
            cancel_callback=self._on_cancel,
            callback_group=self.act_cb)

        self.stand_server = ActionServer(
            self, StandStill, "/humanoid/stand_still",
            execute_callback=self._execute_stand_still,
            goal_callback=self._goal_generic,
            cancel_callback=self._on_cancel,
            callback_group=self.act_cb)

        self.sit_server = ActionServer(
            self, SitDown, "/humanoid/sit_down",
            execute_callback=self._execute_sit_down,
            goal_callback=self._goal_generic,
            cancel_callback=self._on_cancel,
            callback_group=self.act_cb)

        self.wave_server = ActionServer(
            self, WaveHand, "/humanoid/wave_hand",
            execute_callback=self._execute_wave,
            goal_callback=self._goal_wave,
            cancel_callback=self._on_cancel,
            callback_group=self.act_cb)

        self.bow_server = ActionServer(
            self, Bow, "/humanoid/bow",
            execute_callback=self._execute_bow,
            goal_callback=self._goal_generic,
            cancel_callback=self._on_cancel,
            callback_group=self.act_cb)

        # ── Service servers ──────────────────────────────────
        self.create_service(
            Stop, "/humanoid/stop",
            self._handle_stop, callback_group=self.srv_cb)
        self.create_service(
            GetStatus, "/humanoid/get_status",
            self._handle_get_status, callback_group=self.srv_cb)

        self.get_logger().info("command server ready: 5 actions + 2 services")

    # ─────────────────────────────────────────────────────────
    #  Subscription callbacks (cheap — just cache last message)
    # ─────────────────────────────────────────────────────────
    def _on_joint_state(self, msg: JointState):
        self.last_joint_state = msg
        # Also refresh self.current_pose from /joint_states. This
        # makes trajectory restarts feel natural even when the
        # robot drifted from where we last commanded it.
        if msg.position and len(msg.name) == len(msg.position):
            for name, pos in zip(msg.name, msg.position):
                if name in self.current_pose:
                    self.current_pose[name] = float(pos)

    def _on_imu(self, msg: Imu):
        self.last_imu = msg

    def _imu_rpy_deg(self) -> tuple[float, float]:
        """Return (pitch_deg, roll_deg) from the IMU quaternion.
        Returns (NaN, NaN) if no IMU message has been received yet.
        """
        if self.last_imu is None:
            return float("nan"), float("nan")
        q = self.last_imu.orientation
        # Standard quaternion → roll/pitch (ZYX convention)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        return math.degrees(pitch), math.degrees(roll)

    # ─────────────────────────────────────────────────────────
    #  Goal-acceptance policy
    #
    #  Reject any new goal while another action is running.
    #  This is the mutual-exclusion guarantee mentioned in the
    #  design notes. The LLM layer can react to a rejected goal
    #  by calling Stop first, then retrying.
    # ─────────────────────────────────────────────────────────
    def _goal_generic(self, goal_request):
        if self.active_action != "idle":
            self.get_logger().warn(
                f"rejecting goal: '{self.active_action}' is already active")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _goal_walk(self, goal_request):
        if not (0.1 <= goal_request.distance_m <= 3.0):
            self.get_logger().warn(
                f"rejecting walk: distance_m={goal_request.distance_m} "
                f"outside [0.1, 3.0]")
            return GoalResponse.REJECT
        if goal_request.direction not in ("forward", "backward"):
            self.get_logger().warn(
                f"rejecting walk: direction={goal_request.direction!r} "
                f"not in ('forward', 'backward')")
            return GoalResponse.REJECT
        return self._goal_generic(goal_request)

    def _goal_wave(self, goal_request):
        if goal_request.which not in ("left", "right"):
            self.get_logger().warn(
                f"rejecting wave: which={goal_request.which!r} "
                f"not in ('left', 'right')")
            return GoalResponse.REJECT
        return self._goal_generic(goal_request)

    def _on_cancel(self, goal_handle):
        # We accept all cancellation requests. The execute loop
        # detects is_cancel_requested and exits cleanly.
        return CancelResponse.ACCEPT

    # ─────────────────────────────────────────────────────────
    #  The shared trajectory executor
    #
    #  Every action ends up calling this. It publishes one pose
    #  per tick at RATE_HZ, calls feedback_fn(progress) each tick,
    #  and bails out cleanly on cancel.
    #
    #  Returns (completed: bool, last_index: int).
    # ─────────────────────────────────────────────────────────
    def _execute_trajectory(self,
                            traj: list,
                            goal_handle,
                            feedback_fn: Callable[[float, int], None]) -> tuple[bool, int]:
        # Clear any lingering stop flag from a previous Stop service call
        # so the new action doesn't immediately cancel itself.
        self.stop_requested = False
        period = 1.0 / RATE_HZ
        n = len(traj)
        for i, pose in enumerate(traj):
            if goal_handle.is_cancel_requested or self.stop_requested:
                self.get_logger().info(
                    f"trajectory cancelled at index {i}/{n}")
                return False, i

            leg, arm, head = pose_to_arrays(pose)
            self.leg_pub.publish(Float64MultiArray(data=leg))
            self.arm_pub.publish(Float64MultiArray(data=arm))
            self.head_pub.publish(Float64MultiArray(data=head))
            self.current_pose = pose

            progress = (i + 1) / n
            feedback_fn(progress, i)

            time.sleep(period)

        return True, n

    # ─────────────────────────────────────────────────────────
    #  Action implementations — each just builds a trajectory
    #  and delegates to _execute_trajectory.
    # ─────────────────────────────────────────────────────────
    def _execute_walk(self, goal_handle):
        req = goal_handle.request
        self.active_goal_handle = goal_handle
        self.active_action = "walking"
        self.get_logger().info(
            f"walk: {req.distance_m}m {req.direction}")

        num_steps = max(1, int(round(req.distance_m / STEP_LENGTH_M)))
        traj = traj_march_in_place(num_steps=num_steps)

        def fb(progress, i):
            tilt_pitch, _ = self._imu_rpy_deg()
            feedback = Walk.Feedback()
            feedback.progress = float(progress)
            feedback.tilt_deg = float(tilt_pitch if not math.isnan(tilt_pitch) else 0.0)
            feedback.distance_traveled_m = float(progress * req.distance_m)
            goal_handle.publish_feedback(feedback)

        completed, _ = self._execute_trajectory(traj, goal_handle, fb)

        result = Walk.Result()
        if completed:
            goal_handle.succeed()
            result.success = True
            result.distance_actual_m = req.distance_m
            result.message = (f"Walked {req.distance_m:.2f}m {req.direction} "
                              f"(stub: marched in place {num_steps} steps).")
        else:
            goal_handle.canceled()
            result.success = False
            result.distance_actual_m = 0.0   # we can't easily estimate true distance from a march
            result.message = "Walk cancelled."

        self.active_action = "idle"
        self.active_goal_handle = None
        return result

    def _execute_stand_still(self, goal_handle):
        self.active_action = "standing"
        traj = traj_stand_still(self.current_pose)

        def fb(progress, i):
            feedback = StandStill.Feedback()
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)

        completed, _ = self._execute_trajectory(traj, goal_handle, fb)

        result = StandStill.Result()
        if completed:
            goal_handle.succeed()
            result.success = True
            result.message = "Returned to home pose."
        else:
            goal_handle.canceled()
            result.success = False
            result.message = "StandStill cancelled."
        self.active_action = "idle"
        return result

    def _execute_sit_down(self, goal_handle):
        self.active_action = "sitting"
        traj = traj_sit_down(self.current_pose)

        def fb(progress, i):
            feedback = SitDown.Feedback()
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)

        completed, _ = self._execute_trajectory(traj, goal_handle, fb)

        result = SitDown.Result()
        if completed:
            goal_handle.succeed()
            result.success = True
            result.message = "Sit-down complete."
        else:
            goal_handle.canceled()
            result.success = False
            result.message = "SitDown cancelled."
        self.active_action = "idle"
        return result

    def _execute_wave(self, goal_handle):
        which = goal_handle.request.which
        self.active_action = f"waving_{which}"
        traj = traj_wave(which=which, current_pose=self.current_pose)

        def fb(progress, i):
            feedback = WaveHand.Feedback()
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)

        completed, _ = self._execute_trajectory(traj, goal_handle, fb)

        result = WaveHand.Result()
        if completed:
            goal_handle.succeed()
            result.success = True
            result.message = f"Waved with {which} hand."
        else:
            goal_handle.canceled()
            result.success = False
            result.message = "WaveHand cancelled."
        self.active_action = "idle"
        return result

    def _execute_bow(self, goal_handle):
        self.active_action = "bowing"
        traj = traj_bow(self.current_pose)

        def fb(progress, i):
            feedback = Bow.Feedback()
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)

        completed, _ = self._execute_trajectory(traj, goal_handle, fb)

        result = Bow.Result()
        if completed:
            goal_handle.succeed()
            result.success = True
            result.message = "Bow complete."
        else:
            goal_handle.canceled()
            result.success = False
            result.message = "Bow cancelled."
        self.active_action = "idle"
        return result

    # ─────────────────────────────────────────────────────────
    #  Services
    # ─────────────────────────────────────────────────────────
    def _handle_stop(self, request, response):
            """Cancel any in-flight action, then snap to home.

            Sets self.stop_requested = True. The active execute loop
            checks this flag between every tick (~20 ms) and bails out.
            Once we've waited long enough for that to happen, we publish
            HOME so it stays in effect.

            If no action is running, this just snaps to HOME.
            """
            prev = self.active_action
            self.get_logger().warn(f"STOP: was '{prev}', flagging cancel")

            # Phase 1: signal the running action (if any) to stop.
            # The execute loop checks self.stop_requested before each
            # pose publish, so within one control period it will exit.
            self.stop_requested = True

            # Wait long enough for the loop to notice and stop publishing.
            # At RATE_HZ=50 the period is 20 ms; 100 ms gives ample margin.
            if prev != "idle":
                time.sleep(0.1)

            # Phase 2: publish HOME. The action loop is no longer
            # publishing, so this pose is the one that sticks.
            leg, arm, head = pose_to_arrays(HOME_POSE)
            self.leg_pub.publish(Float64MultiArray(data=leg))
            self.arm_pub.publish(Float64MultiArray(data=arm))
            self.head_pub.publish(Float64MultiArray(data=head))
            self.current_pose = dict(HOME_POSE)

            self.stop_requested = False
            self.active_action = "idle"
            response.success = True
            response.message = f"Stopped (was {prev!r}) and snapped to home."
            return response

    def _handle_get_status(self, request, response):
        pitch, roll = self._imu_rpy_deg()
        response.ok = True
        response.tilt_pitch_deg = 0.0 if math.isnan(pitch) else float(pitch)
        response.tilt_roll_deg  = 0.0 if math.isnan(roll)  else float(roll)
        response.current_action = self.active_action

        # Joint positions in ALL_JOINTS order. If joint_states hasn't
        # arrived yet, fall back to the commanded pose.
        response.joint_positions = [float(self.current_pose[j]) for j in ALL_JOINTS]
        response.message = (
            f"Action={self.active_action!r}, "
            f"tilt=(pitch={response.tilt_pitch_deg:.1f}°, "
            f"roll={response.tilt_roll_deg:.1f}°)"
        )
        return response


def main():
    rclpy.init()
    node = CommandServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
