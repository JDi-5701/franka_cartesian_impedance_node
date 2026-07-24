from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("franka_cartesian_impedance_node"),
        "config", "impedance_params.yaml",
    )
    # robot_ip arg overrides the value in impedance_params.yaml when given.
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.3.100",
        description="Robot IP (overrides impedance_params.yaml)")
    # Keep every process thread (DDS, executor, state publisher) off CPUs 14/15:
    # CPU15 handles the eno1 (FCI) IRQ + packet processing, CPU14 is where the
    # libfranka control thread pins itself (control_thread_cpu param). Pair with
    # scripts/nuc_rt_tune.sh which RT-boosts the packet processing on CPU15.
    # CPUs 6/7 are the SMT siblings of 14/15 on this NUC (lscpu -e: CPU6/14 =
    # core 6, CPU7/15 = core 7) — excluded too so the dedicated threads get the
    # whole physical core, not half of one.
    cpus_arg = DeclareLaunchArgument(
        "process_cpus", default_value="0-5,8-13",
        description="CPU list the node process is confined to (control thread escapes"
                    " to control_thread_cpu)")
    return LaunchDescription([
        robot_ip_arg,
        cpus_arg,
        Node(
            package="franka_cartesian_impedance_node",
            executable="cartesian_impedance_node",
            name="cartesian_impedance_node",
            output="screen",
            prefix=["taskset -c ", LaunchConfiguration("process_cpus"), " "],
            parameters=[params, {"robot_ip": LaunchConfiguration("robot_ip")}],
        ),
    ])
