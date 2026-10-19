from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    enable_safety_arg = DeclareLaunchArgument(
        "enable_safety",
        default_value="true",
        description="Whether the safety brake from the safety node is enabled",
    )

    return LaunchDescription(
        [
            enable_safety_arg,
            # Launch follow the gap
            Node(
                package="ftg",
                executable="ftg_node",
                name="ftg_node",
                output="screen",
                parameters=[{"enable_safety": LaunchConfiguration("enable_safety")}],
            ),
        ]
    )
