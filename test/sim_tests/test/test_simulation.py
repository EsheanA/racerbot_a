import os
import time
import pytest
import launch_pytest
from launch_pytest.actions import ReadyToTest
from pathlib import Path
import launch
import launch_ros
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry


@launch_pytest.fixture
def simulation():
    sim_share = Path(
        get_package_share_directory("f1tenth_gym_ros")
    )


    # ALGORITHM_PACKAGE = os.environ["ALGORITHM_PACKAGE"] 
    ALGORITHM_PACKAGE = os.environ.get("ALGORITHM_PACKAGE", "ftg")
    # ALGORITHM_EXECUTABLE = os.environ["ALGORITHM_EXECUTABLE"]
    ALGORITHM_EXECUTABLE = os.environ.get("ALGORITHM_EXECUTABLE", "ftg_node")

    default_config = (Path(__file__).resolve().parents[1] / "config" / "levine.yaml")
    # config_path = os.environ["SIM_CONFIG"]
    config_path = os.environ.get("SIM_CONFIG", str(default_config))


    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sim_share / "launch" / "gym_bridge_launch.py")
        ),
        launch_arguments={
            "config": config_path,
            "open_foxglove": "false",
            "num_agent": "1",
        }.items(),
    )
    
    algorithm = launch_ros.actions.Node(
        package=ALGORITHM_PACKAGE,
        executable=ALGORITHM_EXECUTABLE,
        output="screen",
        parameters=[{"enable_safety": False}]
    )
    return launch.LaunchDescription([
        simulator,
        algorithm,
        ReadyToTest(),
    ])



@pytest.mark.launch(fixture=simulation)
def test_odometry_received():
    rclpy.init()

    node = rclpy.create_node("simulation_test_monitor")
    received_messages = []

    subscription = node.create_subscription(
        Odometry,
        "/ego_racecar/odom",
        lambda msg: received_messages.append(msg),
        10,
    )

    try:
        deadline = time.monotonic() + 15.0

        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received_messages, ( "Simulator did not publish odometry within 15 seconds" )
        

        node.get_logger().info("Observing the simulation for 15 seconds")

        observation_deadline = time.monotonic() + 15.0
        while time.monotonic() < observation_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    finally:
        node.destroy_node()
        rclpy.shutdown()
