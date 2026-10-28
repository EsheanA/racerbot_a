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

    track_arg = DeclareLaunchArgument(
        "track",
        default_value="levine",
        description="Name of the waypoint CSV (without .csv) in the waypoints folder",
    )

    return LaunchDescription(
        [
            enable_safety_arg,
            track_arg,
            # Launch pure pursuit
            Node(
                package="pure_pursuit_a",
                executable="pure_pursuit_a_node",
                name="pure_pursuit_a_node",
                output="screen",
                parameters=[
                    {
                        "enable_safety": LaunchConfiguration("enable_safety"),
                        "track": LaunchConfiguration("track"),
                    }
                ],
            ),
        ]
    )
