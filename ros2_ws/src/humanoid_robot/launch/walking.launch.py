"""
launch/walking.launch.py
Full walking stack:
  imu_filter          → /tilt_degrees
  balance_controller  → /joint_commands
  ik_vectors_DLS      → /joint_commands  (Wampler DLS method)
  joint_bridge        → /leg_controller/commands etc.
  zmp_trajectory      → /com_trajectory + /foot_trajectory
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = FindPackageShare("humanoid_robot")

    # ── Arguments ─────────────────────────────────────────
    ik_enabled_arg = DeclareLaunchArgument(
        "ik_enabled",
        default_value="true",
        description="Set false to publish rest pose (safe testing mode)",
    )
    balance_enabled_arg = DeclareLaunchArgument(
        "balance_enabled",
        default_value="true",   # balance now holds the crouch (ease-in + gain)
        description="Enable the LQR ankle balance correction",
    )
    balance_gain_arg = DeclareLaunchArgument(
        "balance_gain",
        default_value="1.0",
        description="balance correction_gain (1.0 holds the crouch)",
    )
    balance_hip_gain_arg = DeclareLaunchArgument(
        "balance_hip_gain",
        default_value="1.0",   # pitch hip-strategy on (recovers from step tips)
        description="hip-strategy pitch gain (0 = ankle-only)",
    )

    # ── Config paths ──────────────────────────────────────
    robot_params   = PathJoinSubstitution([pkg, "config", "robot_params.yaml"])
    imu_params     = PathJoinSubstitution([pkg, "config", "imu_params.yaml"])
    balance_params = PathJoinSubstitution([pkg, "config", "balance_params.yaml"])
    ik_params      = PathJoinSubstitution([pkg, "config", "ik_params.yaml"])
    gait_params    = PathJoinSubstitution([pkg, "config", "gait_params.yaml"])

    # ── IMU filter node ───────────────────────────────────
    imu_filter_node = Node(
        package="humanoid_robot",
        executable="imu_filter",
        name="imu_filter",
        parameters=[robot_params, imu_params],
        output="screen",
    )

    # ── Balance controller node (LQR, Python) ─────────────
    # Use the LQR balance_controller.py: subscribes /joint_commands (NOMINAL
    # from ik_vectors_DLS), publishes /joint_commands_corrected (nominal + Δθ).
    # NOT the old C++ PID binary (old side-grouped order, publishes raw
    # /joint_commands) — that one is retired from this path.
    balance_node = Node(
        package="humanoid_robot",
        executable="balance_controller.py",
        name="balance_controller",
        parameters=[
            robot_params,
            # balance_params.yaml is the OLD PID config — irrelevant to the LQR
            # node — so it's dropped. correction_gain is the live balance strength.
            {"enabled": ParameterValue(LaunchConfiguration("balance_enabled"), value_type=bool)},
            {"correction_gain": ParameterValue(LaunchConfiguration("balance_gain"), value_type=float)},
            {"hip_gain": ParameterValue(LaunchConfiguration("balance_hip_gain"), value_type=float)},
        ],
        output="screen",
    )

    # ── IK node (Wampler DLS) ─────────────────────────────
    ik_node = Node(
        package="humanoid_robot",
        executable="ik_vectors_DLS",
        name="ik_vectors_dls",
        parameters=[
            robot_params,
            ik_params,
            {"enabled": ParameterValue(LaunchConfiguration("ik_enabled"), value_type=bool)},
        ],
        output="screen",
    )

    # ── Joint bridge node ─────────────────────────────────
    bridge_node = Node(
        package="humanoid_robot",
        executable="joint_bridge",
        name="joint_bridge",
        output="screen",
    )

    # ── ZMP trajectory / pipeline node ───────────────────
    zmp_node = Node(
        package="humanoid_robot",
        executable="pipeline.py",
        name="zmp_trajectory_node",
        output="screen",
        parameters=[gait_params],
    )

    return LaunchDescription([
        ik_enabled_arg,
        balance_enabled_arg,
        balance_gain_arg,
        balance_hip_gain_arg,
        imu_filter_node,
        balance_node,
        ik_node,
        bridge_node,
        zmp_node,
    ])
