from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    deadman_button = DeclareLaunchArgument(
        "deadman_button",
        default_value="4",
        description="Which button should be considered the deadman switch",
    )

    return LaunchDescription(
        [
            deadman_button,
            Node(
                package="safety_node",
                executable="safety_node",
                name="safety_node",
                output="screen",
                parameters=[{"deadman_button": LaunchConfiguration("deadman_button")}],
            ),
        ]
    )
