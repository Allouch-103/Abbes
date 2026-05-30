"""command_api.launch.py — bring up the humanoid command stack.

Launches, in one shot:
  - command_server : the action/service servers that drive the robot.
  - web_ui         : the browser control panel (AI Step 7). It embeds the
                     LLM dispatcher (Steps 4-6) in-process, so we do NOT also
                     launch the standalone `llm_dispatcher` REPL — that would
                     create a second dispatcher competing for the same robot.

Prereq for the LIVE path: Ollama running with the model pulled. Then:
    ros2 launch humanoid_command_api command_api.launch.py
and open http://127.0.0.1:8080 in a browser.

(To exercise just the servers + the terminal REPL instead of the web UI,
run command_server and llm_dispatcher by hand in two terminals as before.)
"""

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
        Node(
            package="humanoid_command_api",
            executable="web_ui",
            name="humanoid_web_ui",
            output="screen",
            emulate_tty=True,
        ),
    ])
