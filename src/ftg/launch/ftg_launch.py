from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    enable_deadman_arg = DeclareLaunchArgument(
        "enable_deadman",
        default_value="true",
        description="Whether the deadman button gate is enforced by safety_node",
    )

    enable_ttc_arg = DeclareLaunchArgument(
        "enable_ttc",
        default_value="true",
        description="Whether the ttc braking is enforced by safety_node",
    )

    return LaunchDescription([
        enable_deadman_arg,
        enable_ttc_arg,

        # Launch follow the gap
        Node(
            package='ftg',
            executable='ftg_node',
            name='ftg_node',
            output='screen',
            remappings=[('drive', 'drive_raw')],
            parameters=[]
        ),

        # Launch safety_node
        Node(
            package='ftg',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[
                {
                    "enable_deadman": LaunchConfiguration("enable_deadman"),
                    "enable_ttc": LaunchConfiguration("enable_ttc"),
                }
            ],
        ),
    ])
