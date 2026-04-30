"""
gazebo.launch.py — part of my_robot_bringup

Reads URDF from:    my_robot_description  (separate package)
Reads controllers from: my_robot_bringup/config/controllers.yaml

Usage:
  ros2 launch my_robot_bringup gazebo.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Package directories ───────────────────────────────
    # URDF lives in my_robot_description
    desc_pkg = get_package_share_directory('my_robot_description')
    # Controllers config lives in my_robot_bringup
    bringup_pkg = get_package_share_directory('my_robot_bringup')
    # ros_gz_sim provides gz_sim.launch.py
    gz_pkg = get_package_share_directory('ros_gz_sim')

    # ── File paths ────────────────────────────────────────
    xacro_file       = os.path.join(desc_pkg,    'urdf',   'humanoid.urdf.xacro')
    controllers_file = os.path.join(bringup_pkg, 'config', 'controllers.yaml')

    # ── Robot description (URDF processed by xacro) ───────
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ── robot_state_publisher ─────────────────────────────
    # use_sim_time=True: node reads /clock from Gazebo bridge
    # so that dt calculations in imu_filter use sim time, not wall time
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # ── Gazebo Harmonic ───────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r',          # -r = run immediately (not paused)
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── Spawn robot ───────────────────────────────────────
    # TimerAction(2s): Gazebo needs time to start before it
    # can accept spawn service calls
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  'humanoid_robot',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.32',
        ],
        output='screen'
    )

    # ── ros_gz_bridge ─────────────────────────────────────
    # Bridges:
    #   /clock → all nodes with use_sim_time=True need this
    #   /imu   → imu_filter.cpp subscribes here (same as real ESP32)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/model/humanoid_robot/link/torso/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[(
            '/world/default/model/humanoid_robot/link/torso/sensor/imu_sensor/imu',
            '/imu'
        )],
        output='screen'
    )

    # ── Controller spawners ───────────────────────────────
    # Each spawner calls controller_manager services to load + activate
    # the controller. It exits with code 0 when the controller is ACTIVE.
    # We use this exit event to chain the next spawner.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # ── Sequencing ────────────────────────────────────────
    # Chain: spawn_robot exits → jsb → leg → arm → head
    # Reason: gz_ros2_control plugin only creates the hardware interfaces
    # AFTER the robot is spawned. Spawning a controller before that = error.
    load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
    load_leg = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[leg_controller_spawner]
        )
    )
    load_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=leg_controller_spawner,
            on_exit=[arm_controller_spawner]
        )
    )
    load_head = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[head_controller_spawner]
        )
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        bridge,
        TimerAction(period=2.0, actions=[spawn_robot]),
        load_jsb,
        load_leg,
        load_arm,
        load_head,
    ])