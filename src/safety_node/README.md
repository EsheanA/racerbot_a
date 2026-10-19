# Safety Node
A ROS2 node that adds a deadman switch (based on input from the `/joy` topic) to ensure code only runs when operators want it to.

## Usage
```bash
cd /racerbot-ws
colcon build --packages-select safety_node
ros2 launch safety_node launch_safety.py
```

## Topics Published
- `/safety/brake` (`std_msgs::msg::Bool`) Whether or not the car should brake due to the deadman switch not being held/connected