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

    use_fallback_follow_method = DeclareLaunchArgument(
        "use_fallback_follow_method",
        default_value="false",
        description="Whether the gap follow node should use the fallback point-following method",
    )

    least_squares_degree = DeclareLaunchArgument(
        "least_squares_degree",
        default_value="2",
        description="Polynomial degree used by the least-squares gap follower",
    )

    steering_gain = DeclareLaunchArgument(
        "steering_gain",
        default_value="1.0",
        description="Gain applied to steering toward the selected gap curve",
    )

    k_samples = DeclareLaunchArgument(
        "k_samples",
        default_value="200",
        description="Number of samples used to find the best steering angle",
    )

    max_steering_angle = DeclareLaunchArgument(
        "max_steering_angle",
        default_value="45.0",
        description="Maximum steering angle allowed for the follower in degrees",
    )

    max_speed = DeclareLaunchArgument(
        "max_speed",
        default_value="4.0",
        description="Maximum forward speed allowed for the gap follower in meters per second",
    )

    min_speed = DeclareLaunchArgument(
        "min_speed",
        default_value="0.5",
        description="Minimum forward speed allowed for the gap follower in meters per second",
    )

    hysteresis_alpha = DeclareLaunchArgument(
        "hysteresis_alpha",
        default_value="0.3",
        description="Smoothing factor for steering-angle hysteresis",
    )

    speed_curve_scale = DeclareLaunchArgument(
        "speed_curve_scale",
        default_value="1.0",
        description="Scale factor for the steering-to-speed curve",
    )
    max_slew_rate = DeclareLaunchArgument(
        "max_slew_rate",
        default_value="170.0",
        description="Maximum rate of change of commanded steering angle, in degrees/sec.",
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
            use_fallback_follow_method,
            least_squares_degree,
            steering_gain,
            k_samples,
            max_steering_angle,
            max_speed,
            min_speed,
            hysteresis_alpha,
            speed_curve_scale,
            max_slew_rate,
            opponent_car_arg,
            # Launch gap_finder_node
            Node(
                package="reactive",
                executable="gap_finder_node",
                name="gap_finder_node",
                output="screen",
                remappings=[("/drive", drive_topic), ("/scan", scan_topic)],
            ),
            # Launch gap_follower_node
            Node(
                package="reactive",
                executable="gap_follow_node",
                name="gap_follow_node",
                output="screen",
                parameters=[
                    {
                        "use_fallback_method": LaunchConfiguration(
                            "use_fallback_follow_method"
                        ),
                        "degree": LaunchConfiguration("least_squares_degree"),
                        "steering_gain": LaunchConfiguration("steering_gain"),
                        "k_samples": LaunchConfiguration("k_samples"),
                        "max_steering_angle": LaunchConfiguration("max_steering_angle"),
                        "max_speed": LaunchConfiguration("max_speed"),
                        "min_speed": LaunchConfiguration("min_speed"),
                        "hysteresis_alpha": LaunchConfiguration("hysteresis_alpha"),
                        "speed_curve_scale": LaunchConfiguration("speed_curve_scale"),
                        "max_slew_rate": LaunchConfiguration("max_slew_rate"),
                        "enable_safety": LaunchConfiguration("enable_safety"),
                    }
                ],
                remappings=[("/drive", drive_topic), ("/scan", scan_topic)],
            ),
        ]
    )
