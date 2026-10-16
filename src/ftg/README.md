# Follow The Gap

Description TODO

## Building
```bash
cd /racerbot_ws
colcon build --packages-select ftg
```

## Launching in real life

```bash
ros2 launch ftg ftg_launch.py
```

## Launching in sim

```bash
ros2 launch ftg ftg_launch.py enable_deadman:=false enable_ttc:=false
```

## Safety Node

Disable the deadman gate with `enable_deadman:=false`.

Disable TTC braking with `enable_ttc:=false`.
