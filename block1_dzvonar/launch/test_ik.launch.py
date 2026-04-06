import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("block1_dzvonar")
    robot_description_path = os.path.join(package_share, "urdf", "arm6.urdf")

    with open(robot_description_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
            }],
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
    ])
