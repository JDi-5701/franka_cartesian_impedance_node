from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("franka_cartesian_impedance_node"),
        "config", "impedance_torque_params.yaml",
    )
    # robot_ip arg overrides the value in impedance_torque_params.yaml when given.
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.3.100",
        description="Robot IP (overrides impedance_torque_params.yaml)")
    return LaunchDescription([
        robot_ip_arg,
        Node(
            package="franka_cartesian_impedance_node",
            executable="cartesian_impedance_torque_node",
            name="cartesian_impedance_torque_node",
            output="screen",
            parameters=[params, {"robot_ip": LaunchConfiguration("robot_ip")}],
        ),
    ])
