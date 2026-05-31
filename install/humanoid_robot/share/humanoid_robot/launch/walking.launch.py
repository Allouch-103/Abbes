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
        default_value="true",
        description="Enable/disable the PID balance controller",
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

    # ── Balance controller node ───────────────────────────
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

    # ── IK node (Wampler DLS) ─────────────────────────────
    ik_node = Node(
        package="humanoid_robot",
        executable="ik_vectors_DLS",
        name="ik_vectors_dls",
        parameters=[
            robot_params,
            ik_params,
            {"enabled": LaunchConfiguration("ik_enabled")},
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
        imu_filter_node,
        balance_node,
        ik_node,
        bridge_node,
        zmp_node,
    ])
