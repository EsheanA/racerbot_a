from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    enable_safety_arg = DeclareLaunchArgument(
        "enable_safety",
        default_value="true",
        description="Whether the safety brake from the safety node is enabled",
    )

    opponent_car = LaunchConfiguration("opponent_car")

    opponent_car_arg = DeclareLaunchArgument(
        "opponent_car",
        default_value="false",
        choices=["true", "false"],
        description="If true, remap /drive -> /opp_drive and /scan -> /opp_scan",
    )

    drive_topic = IfElseSubstitution(opponent_car, "/opp_drive", "/drive")
    scan_topic = IfElseSubstitution(opponent_car, "/opp_scan", "/scan")

    return LaunchDescription(
        [
            enable_safety_arg,
            opponent_car_arg,
            # Launch disparity_extender_node
            Node(
                package="disparity_extender",
                executable="disparity_extender_node",
                name="disparity_extender_node",
                output="screen",
                parameters=[{"enable_safety": LaunchConfiguration("enable_safety")}],
                remappings=[("/drive", drive_topic), ("/scan", scan_topic)],
            ),
        ]
    )
