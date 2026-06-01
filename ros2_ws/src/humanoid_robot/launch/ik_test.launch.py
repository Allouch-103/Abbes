"""
ik_test.launch.py — IK sign-verification harness (no gait, no balance).

Brings up just:
  ik_vectors_DLS   — CoM/foot pose -> joint angles (/joint_commands)
  joint_bridge     — /joint_commands_corrected (deg) -> controllers (rad)
  ik_pose_test     — cycles slow STATIC diagnostic poses

We feed joint_bridge from ik directly by remapping ik's /joint_commands onto
/joint_commands_corrected (no balance node in this harness).

Run alongside Gazebo:
  T1: ros2 launch my_robot_bringup my_robot_gazebo.launch.py
  T2: ros2 launch humanoid_robot ik_test.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("humanoid_robot")
    robot_params = PathJoinSubstitution([pkg, "config", "robot_params.yaml"])
    ik_params = PathJoinSubstitution([pkg, "config", "ik_params.yaml"])

    ik_node = Node(
        package="humanoid_robot", executable="ik_vectors_DLS", name="ik_vectors_dls",
        parameters=[robot_params, ik_params, {"enabled": True}],
        output="screen",
    )
    # No balance in this harness: feed joint_bridge straight from ik's output by
    # remapping /joint_commands -> /joint_commands_corrected.
    bridge_node = Node(
        package="humanoid_robot", executable="joint_bridge", name="joint_bridge",
        remappings=[("/joint_commands_corrected", "/joint_commands")],
        output="screen",
    )
    pose_node = Node(
        package="humanoid_robot", executable="ik_pose_test.py", name="ik_pose_test",
        output="screen",
    )
    return LaunchDescription([ik_node, bridge_node, pose_node])
