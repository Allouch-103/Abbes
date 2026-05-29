"""command_api.launch.py — launch the humanoid command server."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="humanoid_command_api",
            executable="command_server",
            name="humanoid_command_server",
            output="screen",
            emulate_tty=True,
        ),
    ])
