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

    message_timeout_duration = DeclareLaunchArgument(
        "message_timeout_duration",
        default_value="50.0",
        description="How many milliseconds the node should wait for a /joy message before it assumes that no /joy is connected",
    )

    enable_ttc = DeclareLaunchArgument(
        "enable_ttc",
        default_value="true",
        description="Whether the ttc braking is enforced by safety_node",
    )

    return LaunchDescription(
        [
            deadman_button,
            message_timeout_duration,
            enable_ttc,
            Node(
                package="safety_node",
                executable="safety_node",
                name="safety_node",
                output="screen",
                parameters=[
                    {"deadman_button": LaunchConfiguration("deadman_button")},
                    {
                        "message_timeout_duration": LaunchConfiguration(
                            "message_timeout_duration"
                        ),
                        "enable_ttc": LaunchConfiguration("enable_ttc"),
                    },
                ],
            ),
        ]
    )
