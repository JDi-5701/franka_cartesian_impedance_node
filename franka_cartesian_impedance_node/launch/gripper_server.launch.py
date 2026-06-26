"""Franka Hand SERVER bring-up (robot/hardware side only).

Starts just the franka_gripper action server (franka::Gripper, EXCLUSIVE libfranka
connection — only one may connect at a time). Run on the PC wired to the robot (NUC).
The operator-side button client ("hand teleop") lives in spacemouse_teleop.

  ros2 launch franka_cartesian_impedance_node gripper_server.launch.py robot_ip:=192.168.3.100
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.3.100',
                                         description='Robot IP for the franka_gripper server')

    gripper_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('franka_gripper'), 'launch', 'gripper.launch.py')),
        launch_arguments={'robot_ip': LaunchConfiguration('robot_ip'),
                          'namespace': '/'}.items())

    return LaunchDescription([robot_ip_arg, gripper_server])
