"""
launch/march.launch.py
March-in-place test stack:
  imu_filter        → /tilt_degrees
  balance_controller.py → /joint_commands_corrected  (LQR + arm swing)
  ik_vectors_DLS    → /joint_commands
  joint_bridge      → /leg_controller/commands etc.
  pipeline_march    → /com_trajectory + /foot_trajectory + /single_support
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare("humanoid_robot")

    balance_enabled_arg = DeclareLaunchArgument(
        "balance_enabled",
        default_value="true",
        description="Enable/disable the LQR balance controller",
    )
    ik_enabled_arg = DeclareLaunchArgument(
        "ik_enabled",
        default_value="true",
        description="Set false to publish rest pose",
    )

    robot_params   = PathJoinSubstitution([pkg, "config", "robot_params.yaml"])
    imu_params     = PathJoinSubstitution([pkg, "config", "imu_params.yaml"])
    balance_params = PathJoinSubstitution([pkg, "config", "balance_params.yaml"])
    ik_params      = PathJoinSubstitution([pkg, "config", "ik_params.yaml"])

    imu_filter_node = Node(
        package="humanoid_robot",
        executable="imu_filter",
        name="imu_filter",
        parameters=[robot_params, imu_params],
        output="screen",
    )

    balance_node = Node(
        package="humanoid_robot",
        executable="balance_controller.py",
        name="balance_controller",
        parameters=[
            robot_params,
            balance_params,
            {"enabled": LaunchConfiguration("balance_enabled")},
        ],
        output="screen",
    )

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

    bridge_node = Node(
        package="humanoid_robot",
        executable="joint_bridge",
        name="joint_bridge",
        output="screen",
    )

    march_node = Node(
        package="humanoid_robot",
        executable="pipeline_march.py",
        name="zmp_trajectory_node",
        output="screen",
    )

    return LaunchDescription([
        balance_enabled_arg,
        ik_enabled_arg,
        imu_filter_node,
        balance_node,
        ik_node,
        bridge_node,
        march_node,
    ])