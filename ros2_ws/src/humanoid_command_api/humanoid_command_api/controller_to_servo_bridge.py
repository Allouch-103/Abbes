"""
controller_to_servo_bridge.py
=============================

The REAL sim->hardware bridge: it turns the radians the command server
already streams to Gazebo into the servo-degree ``/joint_commands`` the
ESP32 firmware subscribes to.

Why this node exists
--------------------
The robot has two command "languages" that were developed separately:

  * ROS2 / Gazebo side  -> radians, URDF-zero convention (0 rad = the
    geometric reference pose: straight legs, arms down). The command
    server publishes this to three controllers:
        /leg_controller/commands   (10 joints)
        /arm_controller/commands   (6 joints)
        /head_controller/commands  (2 joints)

  * ESP32 firmware side -> servo degrees (0..180), ONE topic
        /joint_commands  (std_msgs/Float32MultiArray, 18 values)
    in the firmware's side-grouped order (see FIRMWARE_ORDER below).

This node listens to the three controller topics and republishes the
SAME motion as ``/joint_commands`` so the real robot mirrors the sim.
Gazebo keeps being driven directly by the controllers (unchanged) — this
bridge runs alongside, feeding only the ESP32.

The conversion (per joint)
--------------------------
    servo_deg = rest_deg + direction * degrees(angle_rad)
    servo_deg = clamp(servo_deg, 0, 180)

``rest_deg`` is the servo angle that corresponds to the URDF-zero pose of
that joint, and ``direction`` (+1 / -1) flips joints whose servo turns the
opposite way to the URDF axis (e.g. mirror-mounted left-side servos).

CALIBRATION (read this before driving a mounted robot!)
-------------------------------------------------------
Per the project convention, the radians->degrees offset lives HERE on the
ROS2 side, NOT in the firmware. The firmware just drives whatever absolute
degree it receives.

Until the robot is physically mounted we don't know the true offsets, so
the defaults below are the SAFE placeholder: every ``rest_deg = 90``
(the mechanical center of a 180 deg servo) and every ``direction = +1``.
With these defaults the HOME pose (all joints 0 rad) maps to "all servos
at 90 deg = centered", which is the safe pose to power on an unmounted
robot.

When you mount the robot, calibrate each joint here:
  1. Power on, send HOME. Each joint should sit at its mechanical neutral.
     If a joint's neutral is not 90 deg on the servo, set its ``rest`` to
     the value that IS neutral.
  2. Command a small positive angle on one joint (e.g. via Gazebo / a
     pose). If the real servo turns the WRONG way, flip that joint's
     ``direction`` to -1.
  3. The firmware still clamps to its own [min,max] in joint_definitions.h,
     so a mis-set value can't drive a servo past its mechanical limit.

This is a pure republisher — no actions, no blocking, no LLM. Safe to run
or not run independently of the rest of the stack.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32MultiArray, Float64MultiArray

from .pose_definitions import LEG_JOINTS, ARM_JOINTS, HEAD_JOINTS, ALL_JOINTS


# ─────────────────────────────────────────────────────────────
#  Firmware joint order (joint_definitions.h, side-grouped).
#  THIS is the index order of the published /joint_commands array.
#  Must stay in sync with firmware/include/joint_definitions.h.
# ─────────────────────────────────────────────────────────────
FIRMWARE_ORDER = [
    "r_shoulder_pitch",   # 0
    "r_shoulder_roll",    # 1
    "r_elbow_roll",       # 2
    "head_yaw",           # 3
    "camera_pitch",       # 4
    "r_hip_roll",         # 5
    "r_hip_pitch",        # 6
    "r_knee_pitch",       # 7
    "r_ankle_pitch",      # 8
    "r_ankle_roll",       # 9
    "l_shoulder_pitch",   # 10
    "l_shoulder_roll",    # 11
    "l_elbow_roll",       # 12
    "l_hip_roll",         # 13
    "l_hip_pitch",        # 14
    "l_knee_pitch",       # 15
    "l_ankle_pitch",      # 16
    "l_ankle_roll",       # 17
]

# Sanity: the bridge must know every joint the firmware expects, and no
# more. If these ever drift, fail loudly at import rather than mis-map.
assert sorted(FIRMWARE_ORDER) == sorted(ALL_JOINTS), \
    "FIRMWARE_ORDER and pose_definitions.ALL_JOINTS disagree on the joint set"


# ─────────────────────────────────────────────────────────────
#  Per-joint calibration table.  {joint_name: (rest_deg, direction)}
#  Defaults below = safe placeholder for an UNMOUNTED robot.
#  Edit per joint after mounting (see CALIBRATION note in the docstring).
# ─────────────────────────────────────────────────────────────
SERVO_CENTER = 90.0   # mechanical center of a 0..180 deg servo

CALIBRATION = {name: (SERVO_CENTER, +1) for name in FIRMWARE_ORDER}

# Hard servo travel limits (degrees). The firmware clamps too; this is a
# second guard so the bridge never emits a value outside a servo's range.
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 180.0


class ControllerToServoBridge(Node):

    def __init__(self):
        super().__init__("controller_to_servo_bridge")

        # Latest commanded angle (radians, URDF convention) per joint.
        # Init to 0.0 == HOME, so before any controller message arrives we
        # already publish a sane centered pose.
        self._angles_rad = {name: 0.0 for name in FIRMWARE_ORDER}

        # Publish rate for /joint_commands. The firmware interpolates
        # toward targets at 100 Hz, so 50 Hz of fresh targets is plenty
        # and matches the command server's trajectory RATE_HZ.
        self.declare_parameter("publish_rate_hz", 50.0)
        rate_hz = self.get_parameter("publish_rate_hz").value

        # ── Subscribers: match the command server's publisher QoS ──
        # command_server publishes the controller topics with the default
        # (RELIABLE) profile, so we subscribe RELIABLE to stay compatible.
        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            Float64MultiArray, "/leg_controller/commands",
            lambda m: self._on_group(m, LEG_JOINTS), sub_qos)
        self.create_subscription(
            Float64MultiArray, "/arm_controller/commands",
            lambda m: self._on_group(m, ARM_JOINTS), sub_qos)
        self.create_subscription(
            Float64MultiArray, "/head_controller/commands",
            lambda m: self._on_group(m, HEAD_JOINTS), sub_qos)

        # ── Publisher: /joint_commands ──
        # RELIABLE writer is compatible with the firmware's BEST_EFFORT
        # reader (and with a plain `ros2 topic echo`, which is reliable).
        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        self._pub = self.create_publisher(
            Float32MultiArray, "/joint_commands", pub_qos)

        self._timer = self.create_timer(1.0 / float(rate_hz), self._publish)

        self.get_logger().info(
            f"controller_to_servo_bridge up: "
            f"3 controller topics (rad) -> /joint_commands (servo deg, 18) "
            f"@ {rate_hz:.0f} Hz")

    # ── Cache one controller group's latest values ──
    def _on_group(self, msg: Float64MultiArray, joint_names: list):
        """Store the latest radians for the joints this controller owns.

        ``joint_names`` is the controller's joint ordering (from
        pose_definitions, which mirrors controllers.yaml), so msg.data[i]
        corresponds to joint_names[i].
        """
        if len(msg.data) != len(joint_names):
            self.get_logger().warn(
                f"ignoring command: got {len(msg.data)} values, "
                f"expected {len(joint_names)}", throttle_duration_sec=2.0)
            return
        for name, angle in zip(joint_names, msg.data):
            self._angles_rad[name] = float(angle)

    # ── Assemble + publish the 18-vector in firmware order ──
    def _publish(self):
        out = Float32MultiArray()
        data = []
        for name in FIRMWARE_ORDER:
            rest_deg, direction = CALIBRATION[name]
            servo_deg = rest_deg + direction * math.degrees(self._angles_rad[name])
            servo_deg = min(SERVO_MAX_DEG, max(SERVO_MIN_DEG, servo_deg))
            data.append(float(servo_deg))
        out.data = data
        self._pub.publish(out)


def main():
    rclpy.init()
    node = ControllerToServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
