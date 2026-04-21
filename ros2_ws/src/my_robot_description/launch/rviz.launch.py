"""
rviz.launch.py — Visualize the humanoid robot in RViz2

What this launches:
  1. robot_state_publisher  — parses URDF, publishes TF transforms
  2. joint_state_publisher_gui — GUI sliders to manually move every joint
  3. rviz2                  — 3D visualizer with pre-loaded config

Usage:
  ros2 launch humanoid_robot rviz.launch.py

Verify:
  - Robot appears in RViz (not just axes)
  - All 18 joint sliders appear in the GUI window
  - Moving a slider moves the correct limb in RViz
  - No red "no transform" errors in RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('humanoid_robot')

    # ── URDF via xacro ────────────────────────────────────────
    # Command() runs xacro at launch time and feeds the output
    # directly to robot_state_publisher as the robot_description param.
    xacro_file = os.path.join(pkg, 'urdf', 'humanoid.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ── RViz config ───────────────────────────────────────────
    rviz_config = os.path.join(pkg, 'rviz', 'robot.rviz')

    # ── robot_state_publisher ─────────────────────────────────
    # Reads robot_description, subscribes to /joint_states,
    # and broadcasts TF2 transforms for every link.
    # Without this, RViz has no idea where anything is.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}],
        output='screen'
    )

    # ── joint_state_publisher_gui ─────────────────────────────
    # Opens a GUI with one slider per joint.
    # Publishes /joint_states so robot_state_publisher can
    # compute the TF tree.
    # Install if missing: sudo apt install ros-jazzy-joint-state-publisher-gui
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # ── RViz2 ─────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node,
    ])
