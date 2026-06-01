"""
balance_test.launch.py — methodically tune the balance controller.

Holds the robot in a STATIC mild-bent stance (BENT pose) and runs the balance
loop on top, so you can tune ONE knob (correction_gain) live and watch whether
balance holds the stance — isolated from the moving gait.

Chain:
  ik_pose_test (hold=BENT) -> /com_trajectory,/foot_trajectory
  ik_vectors_DLS           -> /joint_commands (nominal, bent stance)
  imu_filter (/imu)        -> /tilt_degrees
  balance_controller.py    -> /joint_commands_corrected (nominal + ankle dθ)
  joint_bridge             -> /leg|arm|head_controller/commands -> Gazebo

Run:
  T1: ros2 launch my_robot_bringup my_robot_gazebo.launch.py
  T2: ros2 launch humanoid_robot balance_test.launch.py
  T3: tune the knob live, watch the robot in the GUI:
      ros2 param set /balance_controller correction_gain 0.2
      ros2 param set /balance_controller correction_gain 0.4   ... etc.
  (Start 0 -> robot tips slowly forward. Raise gain until it holds steady;
   if it starts shaking/oscillating, back off ~30%.)
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("humanoid_robot")
    robot_params = PathJoinSubstitution([pkg, "config", "robot_params.yaml"])
    imu_params = PathJoinSubstitution([pkg, "config", "imu_params.yaml"])
    ik_params = PathJoinSubstitution([pkg, "config", "ik_params.yaml"])

    return LaunchDescription([
        Node(package="humanoid_robot", executable="imu_filter", name="imu_filter",
             parameters=[robot_params, imu_params], output="screen"),
        Node(package="humanoid_robot", executable="ik_vectors_DLS", name="ik_vectors_dls",
             parameters=[robot_params, ik_params, {"enabled": True}], output="screen"),
        Node(package="humanoid_robot", executable="balance_controller.py",
             name="balance_controller",
             parameters=[{"enabled": True}, {"correction_gain": 0.0}], output="screen"),
        Node(package="humanoid_robot", executable="joint_bridge", name="joint_bridge",
             output="screen"),
        Node(package="humanoid_robot", executable="ik_pose_test.py", name="ik_pose_test",
             parameters=[{"hold": "BENT"}], output="screen"),
    ])
