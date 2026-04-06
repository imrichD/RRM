import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robot_state_publisher_launch = os.path.join(
        get_package_share_directory("rrm_simple_robot_model"),
        "launch",
        "robot_state_publisher.launch.xml",
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(robot_state_publisher_launch),
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="block1_dzvonar",
            executable="logger_node",
            name="joint_logger",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])