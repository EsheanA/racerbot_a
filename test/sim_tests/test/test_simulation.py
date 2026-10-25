import os
import time
import unittest
from pathlib import Path
import launch
import launch_ros
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from nav_msgs.msg import Odometry



def generate_test_description():
    sim_share = Path(
        get_package_share_directory("f1tenth_gym_ros")
    )


    ALGORITHM_PACKAGE = os.environ["ALGORITHM_PACKAGE"]
    ALGORITHM_EXECUTABLE = os.environ["ALGORITHM_EXECUTABLE"]
    config_path = os.environ["SIM_CONFIG"]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sim_share / "launch" / "gym_bridge_launch.py")
        ),
        launch_arguments={
            "config": config_path,
            "open_foxglove": "true",
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





class TestSimulation(unittest.TestCase):

    def test_odometry_received(self):
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

            self.assertTrue(
                received_messages,
                "Simulator did not publish odometry within 15 seconds"
            )

            node.get_logger().info("Observing the simulation for 15 seconds")

            observation_deadline = time.monotonic() + 15.0
            while time.monotonic() < observation_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)

        finally:
            node.destroy_node()
            rclpy.shutdown()
