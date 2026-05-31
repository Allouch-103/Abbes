"""
balance.launch.py — Launch Phase 3 standing balance stack

Starts:
  1. imu_filter       — complementary filter on /imu → /tilt_degrees
  2. balance_controller — PID on /tilt_degrees → /joint_commands

Usage:
  ros2 launch humanoid_robot balance.launch.py
  ros2 launch humanoid_robot balance.launch.py enabled:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('humanoid_robot')

    # Config file paths
    imu_config     = os.path.join(pkg, 'config', 'imu_params.yaml')
    balance_config = os.path.join(pkg, 'config', 'balance_params.yaml')

    # ── Launch arguments ─────────────────────────────────
    # These override YAML values at launch time:
    #   ros2 launch humanoid_robot balance.launch.py enabled:=false
    enabled_arg = DeclareLaunchArgument(
        'enabled', default_value='true',
        description='Enable active balance correction'
    )

    # ── Nodes ─────────────────────────────────────────────
    imu_filter_node = Node(
        package='humanoid_robot',
        executable='imu_filter',
        name='imu_filter',
        parameters=[imu_config],
        output='screen'
    )

    balance_node = Node(
        package='humanoid_robot',
        executable='balance_controller',
        name='balance_controller',
        parameters=[
            balance_config,
            # Override 'enabled' from launch argument
            {'enabled': LaunchConfiguration('enabled')}
        ],
        output='screen'
    )

    return LaunchDescription([
        enabled_arg,
        imu_filter_node,
        balance_node,
    ])