"""
launch/walking.launch.py

Full walking stack:
  imu_filter          → /tilt_degrees
  balance_controller  → /joint_commands  (overridden by IK when walking)
  inverse_kinematics  → /joint_commands

When IK is enabled, it outputs the planned joint angles.
The balance controller still runs to provide IMU-based ankle corrections
during walking (both nodes publish to /joint_commands — the ESP32 takes
whichever arrives last, which is effectively the IK output + PID delta).

For Phase 4 testing without a gait generator:
  ros2 launch humanoid_robot walking.launch.py ik_enabled:=false
  → sends rest pose, use topic pub to test with static trajectories.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare("humanoid_robot")

    # ── Arguments ─────────────────────────────────────────────────────────────
    ik_enabled_arg = DeclareLaunchArgument(
        "ik_enabled",
        default_value="true",
        description="Set false to publish rest pose (safe testing mode)",
    )

    balance_enabled_arg = DeclareLaunchArgument(
        "balance_enabled",
        default_value="true",
        description="Enable/disable the PID balance controller",
    )

    # ── Config paths ──────────────────────────────────────────────────────────
    robot_params   = PathJoinSubstitution([pkg, "config", "robot_params.yaml"])
    imu_params     = PathJoinSubstitution([pkg, "config", "imu_params.yaml"])
    balance_params = PathJoinSubstitution([pkg, "config", "balance_params.yaml"])
    ik_params      = PathJoinSubstitution([pkg, "config", "ik_params.yaml"])

    # ── IMU filter node (Phase 2) ─────────────────────────────────────────────
    imu_filter_node = Node(
        package="humanoid_robot",
        executable="imu_filter",
        name="imu_filter",
        parameters=[robot_params, imu_params],
        output="screen",
    )

    # ── Balance controller node (Phase 3) ────────────────────────────────────
    balance_node = Node(
        package="humanoid_robot",
        executable="balance_controller",
        name="balance_controller",
        parameters=[
            robot_params,
            balance_params,
            {"enabled": LaunchConfiguration("balance_enabled")},
        ],
        output="screen",
    )

    # ── Inverse kinematics node (Phase 4) ────────────────────────────────────
    ik_node = Node(
        package="humanoid_robot",
        executable="inverse_kinematics",
        name="inverse_kinematics",
        parameters=[
            robot_params,
            ik_params,
            {"enabled": LaunchConfiguration("ik_enabled")},
        ],
        output="screen",
    )

    return LaunchDescription([
        ik_enabled_arg,
        balance_enabled_arg,
        imu_filter_node,
        balance_node,
        ik_node,
    ])