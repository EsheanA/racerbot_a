import os
import time
## for loop detection implementation
import math
from std_msgs.msg import String
##----------------------------------
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

##--- Converts car's orientation/3d rotation to angle range of initial position
def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
##------------------------------------------------------------
##--- takes in cars position and heading, notifies of loop is complete
class LoopDetector:
    # sets up values and memory
    def __init__(self, radius=1.0, leave_dist=3.0, max_heading_err=math.radians(45)):
        self.radius = radius
        self.leave_dist = leave_dist
        self.max_heading_err = max_heading_err
        self.start = None          # (x, y, yaw)
        self.armed = False
        self.laps = 0

        self.lap_times = []        # seconds per completed lap (sim time)
        self.travelled = 0.0       # total distance driven (m)
        self.last_xy = None
        self.t_first = None
        self.t_last = None
        self.t_lap_start = None

    # called on every odom msg, loops through values to check if lap occurs
    def update(self, x, y, yaw, t):
        self.t_last = t
        if self.last_xy is not None:
            self.travelled += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

        if self.start is None:
            self.start = (x, y, yaw)
            self.t_first = self.t_lap_start = t
            return False

        sx, sy, syaw = self.start
        dist = math.hypot(x - sx, y - sy)

        if not self.armed:
            if dist > self.leave_dist:
                self.armed = True
            return False

        heading_err = abs(math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw)))
        if dist < self.radius and heading_err < self.max_heading_err:
            self.armed = False
            self.laps += 1
            self.lap_times.append(t - self.t_lap_start)
            self.t_lap_start = t
            return True
        return False

    def summary(self):
        if self.t_first is None:
            return "Loop summary: no odometry received."
        elapsed = self.t_last - self.t_first
        lines = [
            f"Laps completed : {self.laps}",
            f"Sim time       : {elapsed:.1f} s",
            f"Distance driven: {self.travelled:.1f} m",
        ]
        if self.lap_times:
            lines.append("Lap times      : " + ", ".join(f"{t:.2f}s" for t in self.lap_times))
            lines.append(f"Best / average : {min(self.lap_times):.2f}s / "
                         f"{sum(self.lap_times) / len(self.lap_times):.2f}s")
        return "\n".join(lines)
###----------------------------------------------------------------------------------------


@launch_pytest.fixture
def simulation():
    sim_share = Path(
        get_package_share_directory("f1tenth_gym_ros")
    )


    ALGORITHM_PACKAGE = os.environ.get("ALGORITHM_PACKAGE", "ftg")
    ALGORITHM_EXECUTABLE = os.environ.get("ALGORITHM_EXECUTABLE", "ftg_node")

    default_config = (Path(__file__).resolve().parents[1] / "config" / "levine.yaml")
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

    node.get_logger
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

        assert received_messages, ( "Simulator did not publish odometry within 30 seconds" )
        

        node.get_logger().info("Observing the simulation for 30 seconds")

        observation_deadline = time.monotonic() + 15.0
        while time.monotonic() < observation_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    finally:
        node.destroy_node()
        rclpy.shutdown()

##--- watches car's odometry, passes if loop is detected
@pytest.mark.launch(fixture=simulation)
def test_loop_detected(request):
    rclpy.init()
    node = rclpy.create_node("loop_detector")

    detector = LoopDetector(radius=1.0, leave_dist=3.0)
    lap_pub = node.create_publisher(String, "/lap_detected", 10)
    received = []

    def odom_cb(msg):
        received.append(msg)
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if detector.update(p.position.x, p.position.y, yaw, t):
            text = f"Lap {detector.laps} completed"
            lap_pub.publish(String(data=text))
            node.get_logger().info(text)

    node.create_subscription(Odometry, "/ego_racecar/odom", odom_cb, 10)

    try:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        request.config.loop_summary = detector.summary()   # saved for the report
        node.destroy_node()
        rclpy.shutdown()

    assert received, "Simulator did not publish odometry"
    assert detector.laps >= 1, "Car did not complete a lap within 30 seconds"
##-------------------------------
