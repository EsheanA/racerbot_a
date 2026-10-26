# Pure Pursuit

Description TODO

## Building
```bash
cd /racerbot_ws
colcon build --packages-select pure_pursuit_a
```

## Launching in sim

```bash
ros2 run pure_pursuit_a pure_pursuit_a_node --track:=levine2
```