import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch_ros.actions import Node


def _client_terminal_action(_context):
    client_cmd = "source /opt/ros/jazzy/setup.bash && ros2 run block1_dzvonar cartesian_control_client; exec bash"

    terminals = [
        ("gnome-terminal", ["gnome-terminal", "--", "bash", "-lc", client_cmd]),
        ("xterm", ["xterm", "-e", "bash", "-lc", client_cmd]),
        ("konsole", ["konsole", "-e", "bash", "-lc", client_cmd]),
        ("xfce4-terminal", ["xfce4-terminal", "--command", f"bash -lc '{client_cmd}'"]),
    ]

    for name, command in terminals:
        if shutil.which(name):
            return [
                LogInfo(msg=f"Opening interactive client in {name}..."),
                ExecuteProcess(cmd=command, output="screen"),
            ]

    return [
        LogInfo(
            msg=(
                "No supported terminal emulator found for auto client launch. "
                "Run manually: ros2 run block1_dzvonar cartesian_control_client"
            )
        )
    ]


def generate_launch_description():
    robot_model_share = get_package_share_directory("rrm_simple_robot_model")
    block1_share = get_package_share_directory("block1_dzvonar")

    robot_description_path = os.path.join(robot_model_share, "urdf", "arm.urdf")
    rviz_config_path = os.path.join(block1_share, "rviz", "task_1_6.rviz")

    with open(robot_description_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rrm_sim",
            executable="robot_sim_node",
            name="robot_sim_node",
            output="screen",
        ),
        Node(
            package="block1_dzvonar",
            executable="logger_node",
            name="joint_logger",
            output="screen",
        ),
        Node(
            package="block1_dzvonar",
            executable="ik_solver_node",
            name="ik_solver",
            output="screen",
        ),
        Node(
            package="block1_dzvonar",
            executable="motion_manager_node",
            name="motion_manager",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
        ),
        TimerAction(
            period=2.0,
            actions=[OpaqueFunction(function=_client_terminal_action)],
        ),
    ])
