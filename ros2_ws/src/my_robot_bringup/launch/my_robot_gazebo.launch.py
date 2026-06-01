"""
my_robot_gazebo.launch.py

Start order:
  t=0s  : robot_state_publisher, Gazebo, ros_gz_bridge, RViz
  t=5s  : spawn_robot
  spawn exits → joint_state_broadcaster
  jsb exits   → leg_controller
  leg exits   → arm_controller
  arm exits   → head_controller
  head exits  → imu_filter, ik_vectors_DLS, joint_bridge,
                balance_controller.py, pipeline.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    desc_pkg    = get_package_share_directory('my_robot_description')
    bringup_pkg = get_package_share_directory('my_robot_bringup')
    gz_pkg      = get_package_share_directory('ros_gz_sim')

    xacro_file   = os.path.join(desc_pkg,    'urdf',   'humanoid.urdf.xacro')
    rviz_config  = os.path.join(desc_pkg,    'rviz',   'robot.rviz')
    world_file   = os.path.join(bringup_pkg, 'worlds', 'empty.sdf')

    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ── robot_state_publisher ─────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # ── Gazebo ────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # ── ros_gz_bridge ─────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/model/humanoid_robot/link/base_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[(
            '/world/default/model/humanoid_robot/link/base_link/sensor/imu_sensor/imu',
            '/imu'
        )],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── RViz ─────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    # ── Spawn robot ───────────────────────────────────────
    # FIX: z=0.30 — feet land on ground not inside it
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  'humanoid_robot',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.30',
            '-R', '0', '-P', '0', '-Y', '0',
        ],
        output='screen'
    )

    # ── Controller spawners ───────────────────────────────
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}], output='screen'
    )
    leg_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['leg_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}], output='screen'
    )
    arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}], output='screen'
    )
    head_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['head_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # ── Control nodes ─────────────────────────────────────
    # Started only after all controllers are active
    imu_filter_node = Node(
        package='humanoid_robot',
        executable='imu_filter',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    ik_node = Node(
        package='humanoid_robot',
        executable='ik_vectors_DLS',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    joint_bridge_node = Node(
        package='humanoid_robot',
        executable='joint_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    # Python LQR balance controller (not the removed C++ one)
    balance_node = Node(
        package='humanoid_robot',
        executable='balance_controller.py',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    pipeline_node = Node(
        package='humanoid_robot',
        executable='pipeline.py',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── Sequencing ────────────────────────────────────────
    load_jsb = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[jsb_spawner]
    ))
    load_leg = RegisterEventHandler(OnProcessExit(
        target_action=jsb_spawner,
        on_exit=[leg_spawner]
    ))
    load_arm = RegisterEventHandler(OnProcessExit(
        target_action=leg_spawner,
        on_exit=[arm_spawner]
    ))
    load_head = RegisterEventHandler(OnProcessExit(
        target_action=arm_spawner,
        on_exit=[head_spawner]
    ))
    # All control nodes start together after head controller is active
    load_control = RegisterEventHandler(OnProcessExit(
        target_action=head_spawner,
        on_exit=[
            imu_filter_node,
            ik_node,
            joint_bridge_node,
            balance_node,
            pipeline_node,
        ]
    ))

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
            description='Launch RViz2'),
        DeclareLaunchArgument('use_joint_gui', default_value='false',
            description='Launch joint_state_publisher_gui'),
        rsp_node,
        gazebo,
        bridge,
        rviz_node,
        TimerAction(period=5.0, actions=[spawn_robot]),
        load_jsb,
        load_leg,
        load_arm,
        load_head,
        load_control,
    ])
