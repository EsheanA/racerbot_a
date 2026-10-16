# Disparity Extender

The Disparity Extender Node processes LiDAR disparities to safely expand obstacles and select the furthest reachable point, allowing the vehicle to drive faster while reducing the risk of clipping obstacles.

## Building
```bash
cd /racerbot_ws
colcon build --packages-select disparity_extender
```

## Launching in real life

```bash
ros2 launch disparity_extender disparity_extender_launch.py enable_ttc:=false
```

## Launching in sim

```bash
ros2 launch disparity_extender disparity_extender_launch.py enable_deadman:=false enable_ttc:=false
```

## Safety Node

Disable the deadman gate with `enable_deadman:=false`.

Disable TTC braking with `enable_ttc:=false`.
