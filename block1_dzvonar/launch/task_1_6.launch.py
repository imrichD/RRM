import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch_ros.actions import Node


def _clean_gui_env():
    env = os.environ.copy()
    for key in ["GTK_PATH", "GTK_EXE_PREFIX", "GTK_IM_MODULE_FILE", "GIO_MODULE_DIR", "GSETTINGS_SCHEMA_DIR", "LOCPATH", "LD_PRELOAD"]:
        env.pop(key, None)

    for key in ["PATH", "XDG_DATA_DIRS", "LD_LIBRARY_PATH"]:
        value = env.get(key, "")
        if value:
            env[key] = ":".join([p for p in value.split(":") if "/snap/" not in p])
    return env


def _client_terminal_action(_context):
    client_cmd = "source /opt/ros/jazzy/setup.bash && ros2 run block1_dzvonar cartesian_control_client; exec bash"
    gui_env = _clean_gui_env()

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
                ExecuteProcess(cmd=command, output="screen", env=gui_env),
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
    package_share = get_package_share_directory("block1_dzvonar")
    robot_description_path = os.path.join(package_share, "urdf", "arm6.urdf")
    rviz_config_path = os.path.join(package_share, "rviz", "task_1_6.rviz")

    with open(robot_description_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    gui_env = _clean_gui_env()

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
        ExecuteProcess(
            cmd=["rviz2", "-d", rviz_config_path],
            output="screen",
            env=gui_env,
        ),
        TimerAction(
            period=2.0,
            actions=[OpaqueFunction(function=_client_terminal_action)],
        ),
    ])