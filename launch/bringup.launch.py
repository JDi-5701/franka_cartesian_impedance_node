"""Franka robot-side bring-up: Cartesian impedance controller + Franka Hand SERVER.

One command for everything that talks to the robot (the NUC side):
  - cartesian_impedance_node (arm, franka::Robot)
  - franka_gripper server (franka::Gripper), via gripper_server.launch.py

The operator side (SpaceMouse arm teleop + gripper button "hand teleop") is launched
separately from spacemouse_teleop (so it can run on a different PC).

  ros2 launch franka_cartesian_impedance_node bringup.launch.py robot_ip:=192.168.3.100

Args:
  robot_ip : robot IP, passed to BOTH the controller and the gripper server.
  gripper  : include the franka_gripper server (default true).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('franka_cartesian_impedance_node'), 'launch')

    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.3.100',
                                         description='Robot IP (controller + gripper server)')
    gripper_arg = DeclareLaunchArgument('gripper', default_value='true',
                                        description='Include the franka_gripper server')

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cartesian_impedance.launch.py')),
        launch_arguments={'robot_ip': LaunchConfiguration('robot_ip')}.items())

    gripper_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gripper_server.launch.py')),
        launch_arguments={'robot_ip': LaunchConfiguration('robot_ip')}.items(),
        condition=IfCondition(LaunchConfiguration('gripper')))

    return LaunchDescription([robot_ip_arg, gripper_arg, controller, gripper_server])
