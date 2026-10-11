# Research

Contains notes and learnings from testing in simulation, on the car, and from lectures, so we can track what's worked, what hasn't, and why

## Simulation

- If our turning in simulation is not consistent with our turning in real life testing, the likely culprit to consider would be the simulator's vehicle dynamics model.
  - If nonlinear: We may be cornering too fast, causing saturation/sliding that wouldn't be picked up with a kinematic or linear model.

## Safety

- TTC needs reliable odometry.
- Safety should be built into the driving controller, not a separate process, otherwise they compete with each other.

## Reactive Driving

### Follow the Gap (FTG)

- Preprocess lidar scan
  - Restrict scan to 180 degrees: Good since it removes lidar points behind and to the sides of the car that aren't relevant to forward driving.
  - Smooth by taking mean every 3 scans: Don't know if it makes a difference.
  - Remove outlier scans: To be tested.

- Safety bubble
  - Using exact value instead of linear approximation is good.
  - Overall, good to avoid crashing into corners.

- Finding Gap
  - We choose widest gap: Backfires when a wide but shallow gap wins over a narrower, deeper one, steering us toward a dead end.
  - We choose width\*depth gap: Solid.
  - We account for how much we must turn: To be tested.
  - We choose center of widest gap: Solid.

- Steering
  - Go to best point: Car oversteers, as it chooses a new point each time.

- Speed
  - Slower speeds generally crash less and are closer to matching real life vs. simulation.
  - No solid speed function yet.

### Disparity Extender (DE)

- Decreasing the speed caused the car to crash into more corners. Added saftey bubble to try and counteract this. Good
- Car seemed to drive in "S-Curves" on straightaways
- Hypothesis was that the car would come out of turns slightly turned, drive towards a wall and then compensate, causing it to drive to the other wall. This process compounded over time and resulted in the "S-Curves"
- Deviation Penalty
  - Apply a penalty that influences the car to drive straight based on a penalty factor
  - No matter what values were tried for the penalty factor, the car either drove straight into a wall or the penalty had 0 effect
- Limiting the angle returned from the function that determined steering angle to the car's turning range also had no effect
- Slew rate limiter
  - Capped how fast the turning angle could change per callback tick, rather than how far obstacles could be.
  - This allowed for more smoother turns and helped the car stay straight coming out of turns
  - This fixed the "S-Curves" issue

## Mapping Driving

### TF Systems

**Issue:** the car tracks its own motion by counting wheel turns and steering angle (dead reckoning). This is fast and smooth, but drifts over time from wheel slip, tire wear, and other real-world imperfections, so the tracked position slowly stops matching the car's true position. TF systems manage this offset, instead of one single number for the car's location. ROS keeps several different position estimates as separately named coordinate frames, so the one that always drifts (dead reckoning estimate or `odom` frame) can be corrected later without being thrown out or restarted.

**Frames and the tree**

- Every coordinate frame (`map`, `odom`, `base_link`, sensor frames) sits in one tree, and each frame has exactly one parent. (Set by [REP 105](https://www.ros.org/reps/rep-0105.html), the official ROS spec for robot frames.)
- Fixed parts on the car (sensor mounts) publish a static transform once, onto a topic called `/tf_static`. Dynamic things, like `odom → base_link`, publish continuously, onto a topic called `/tf`. Every message on either topic has the same shape: a parent frame, any number of child coordinate frames, and the rotation plus translation between them, sent as a `geometry_msgs/msg/transform_stamped` ([ROS 2 tf2 tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)).
- Any node can ask "where is X relative to Y" and tf2 chains the pieces together, using [tf2 tutorials, ROS 2 docs]

**Order**
`map` -> `odom` -> `base_link` -> sensors

- `base_link`: bolted to the chassis, doesn't move relative to the car (rear axle center).
- `odom`: a world-fixed frame used for dead reckoning. Drifts over time, but continuous ([REP 105](https://www.ros.org/reps/rep-0105.html)).
- `map`: another world-fixed frame tied to the saved track map. Accurate long term, but can jump when localization corrects it.
- Map and odom can't both attach straight to base_link. A frame can only have one parent, so they have to be stacked.

![frame order](assets/frame%20order.png)

**Sensor frames (example: `laser_frame`)**

- Sensors only know the world in their own frame of reference. A LiDAR point returns a distance from a wall and its angle, which is measured from the sensor itself, in `laser_frame`. On its own, we cannot tell exactly where the point is relative to the car.
- When LiDAR is mounted, we measure exactly how close `laser_frame` sits relative to `base_link` (for example, 15cm forward of the axle, facing straight ahead), and publish that as a static transform. (This never changes since the components are bolted down.)
- Applying that transform is what allows us to make sense of the LiDAR readings, using the static transform as a frame of reference (ex. LiDAR scans 2m ahead of sensor -> 2.15m ahead of `base_link`). Specifically, it is a rigid body transform, a rotation and a translation, no stretching, since the sensor and chassis don't flex relative to each other ([ROS 2 tf2 tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)).
- Because `base_link` already chains up to `odom` and `map`, that same LiDAR point can then be expressed relative to either of those too. The LiDAR node never needs to know odometry or localization exist.
- `imu_frame` works the same way, a fixed offset to `base_link`, so IMU readings can be used in the car's own frame.

**Odometry (`odom` frame and `odom → base_link` transform)**

- Whatever node reads the wheels/VESC (plus IMU if fused) does two separate things with the result:
  1. Publishes it as data, a `nav_msgs/Odometry` message on a topic (`/odom`), carrying the robots pose and its velocity, for anything that wants the actual numbers, like an EKF or a logger ([nav_msgs/Odometry message](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html)).
  2. Publishes the matching `odom → base_link` transform onto `/tf`, at the same rate. This transform represents the car's smooth, continuous trajectory based purely on dead reckoning, measuring how far it has traveled relative to its starting point (the fixed `odom` frame).

**Correcting Odom Drift (`map → odom`)**

- Because dead reckoning accumulates error over time, the `odom → base_link` position will eventually drift away from reality.
- To prevent the car from violently jerking to correct this error (would break continuous control loops), the `odom → base_link` transform is never snapped back to reality.
- Instead, a localization node (SLAM) calculates the exact amount of accumulated drift and publishes a `map → odom` transform. This acts as an offset, shifting the entire `odom` frame back to align with the real-world `map`, ensuring the final `map → base_link` chain is accurate while keeping local movement perfectly smooth.

- These two are easy to mix up, they usually come from the same node and the same numbers, but they do different jobs. The topic is data to read, the transform is for chaining coordinate frames. While purely reactive nodes (ftg) only need static sensor transforms, trajectory trackers that need to track the car's continuous movement in space rely on this odom transform.
- Reliable odometry is also what our TTC safety logic needs (see Safety above).

**Splitting up `map` and `odom`**

- Speed: localization updates slower (10 to 40Hz) than control needs (50 to 100+Hz). Splitting lets fast nodes work asynchronously with slow nodes.
- Safety: jumps only ever happen in map to odom. Odom to base_link never jumps, so PID and follow the gap never get destabilized mid turn.
- Simplicity: anything that wants the car's map position (costmap, raceline follower, RViz) just asks for it, and tf2 chains map, odom, and base_link together automatically.

**Debugging:** use `ros2 run tf2_ros tf2_echo` to show a live transform, and use `ros2 run rqt_tf_tree rqt_tf_tree` to see the whole tree for debugging purposes.

## AI Driving

(No notes yet.)

---

# Research

Contains notes and learnings from testing in simulation, on the car, and from lectures, so we can track what's worked, what hasn't, and why

## Simulation

- If our turning in simulation is not consistent with our turning in real life testing, the likely culprit to consider would be the simulator's vehicle dynamics model.
  - If nonlinear: We may be cornering too fast, causing saturation/sliding that wouldn't be picked up with a kinematic or linear model.

## Safety

- TTC needs reliable odometry.
- Safety should be built into the driving controller, not a separate process, otherwise they compete with each other.

## Reactive Driving

### Follow the Gap (FTG)

- Preprocess lidar scan
  - Restrict scan to 180 degrees: Good since it removes lidar points behind and to the sides of the car that aren't relevant to forward driving.
  - Smooth by taking mean every 3 scans: Don't know if it makes a difference.
  - Remove outlier scans: To be tested.

- Safety bubble
  - Using exact value instead of linear approximation is good.
  - Overall, good to avoid crashing into corners.

- Finding Gap
  - We choose widest gap: Backfires when a wide but shallow gap wins over a narrower, deeper one, steering us toward a dead end.
  - We choose width\*depth gap: Solid.
  - We account for how much we must turn: To be tested.
  - We choose center of widest gap: Solid.

- Steering
  - Go to best point: Car oversteers, as it chooses a new point each time.

- Speed
  - Slower speeds generally crash less and are closer to matching real life vs. simulation.
  - No solid speed function yet.

### Disparity Extender (DE)

- Decreasing the speed caused the car to crash into more corners. Added saftey bubble to try and counteract this. Good
- Car seemed to drive in "S-Curves" on straightaways
- Hypothesis was that the car would come out of turns slightly turned, drive towards a wall and then compensate, causing it to drive to the other wall. This process compounded over time and resulted in the "S-Curves"
- Deviation Penalty
  - Apply a penalty that influences the car to drive straight based on a penalty factor
  - No matter what values were tried for the penalty factor, the car either drove straight into a wall or the penalty had 0 effect
- Limiting the angle returned from the function that determined steering angle to the car's turning range also had no effect
- Slew rate limiter
  - Capped how fast the turning angle could change per callback tick, rather than how far obstacles could be.
  - This allowed for more smoother turns and helped the car stay straight coming out of turns
  - This fixed the "S-Curves" issue

## Mapping Driving

### TF Systems

**Issue:** the car tracks its own motion by counting wheel turns and steering angle (dead reckoning). This is fast and smooth, but drifts over time from wheel slip, tire wear, and other real-world imperfections, so the tracked position slowly stops matching the car's true position. TF systems manage this offset, instead of one single number for the car's location. ROS keeps several different position estimates as separately named coordinate frames, so the one that always drifts (dead reckoning estimate or `odom` frame) can be corrected later without being thrown out or restarted.

**Frames and the tree**

- Every coordinate frame (`map`, `odom`, `base_link`, sensor frames) sits in one tree, and each frame has exactly one parent. (Set by [REP 105](https://www.ros.org/reps/rep-0105.html), the official ROS spec for robot frames.)
- Fixed parts on the car (sensor mounts) publish a static transform once, onto a topic called `/tf_static`. Dynamic things, like `odom → base_link`, publish continuously, onto a topic called `/tf`. Every message on either topic has the same shape: a parent frame, any number of child coordinate frames, and the rotation plus translation between them, sent as a `geometry_msgs/msg/transform_stamped` ([ROS 2 tf2 tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)).
- Any node can ask "where is X relative to Y" and tf2 chains the pieces together, using [tf2 tutorials, ROS 2 docs]

**Order**
`map` -> `odom` -> `base_link` -> sensors

- `base_link`: bolted to the chassis, doesn't move relative to the car (rear axle center).
- `odom`: a world-fixed frame used for dead reckoning. Drifts over time, but continuous ([REP 105](https://www.ros.org/reps/rep-0105.html)).
- `map`: another world-fixed frame tied to the saved track map. Accurate long term, but can jump when localization corrects it.
- Map and odom can't both attach straight to base_link. A frame can only have one parent, so they have to be stacked.

![frame order](assets/frame%20order.png)

**Sensor frames (example: `laser_frame`)**

- Sensors only know the world in their own frame of reference. A LiDAR point returns a distance from a wall and its angle, which is measured from the sensor itself, in `laser_frame`. On its own, we cannot tell exactly where the point is relative to the car.
- When LiDAR is mounted, we measure exactly how close `laser_frame` sits relative to `base_link` (for example, 15cm forward of the axle, facing straight ahead), and publish that as a static transform. (This never changes since the components are bolted down.)
- Applying that transform is what allows us to make sense of the LiDAR readings, using the static transform as a frame of reference (ex. LiDAR scans 2m ahead of sensor -> 2.15m ahead of `base_link`). Specifically, it is a rigid body transform, a rotation and a translation, no stretching, since the sensor and chassis don't flex relative to each other ([ROS 2 tf2 tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)).
- Because `base_link` already chains up to `odom` and `map`, that same LiDAR point can then be expressed relative to either of those too. The LiDAR node never needs to know odometry or localization exist.
- `imu_frame` works the same way, a fixed offset to `base_link`, so IMU readings can be used in the car's own frame.

**Odometry (`odom` frame and `odom → base_link` transform)**

- Whatever node reads the wheels/VESC (plus IMU if fused) does two separate things with the result:
  1. Publishes it as data, a `nav_msgs/Odometry` message on a topic (`/odom`), carrying the robots pose and its velocity, for anything that wants the actual numbers, like an EKF or a logger ([nav_msgs/Odometry message](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html)).
  2. Publishes the matching `odom → base_link` transform onto `/tf`, at the same rate. This transform represents the car's smooth, continuous trajectory based purely on dead reckoning, measuring how far it has traveled relative to its starting point (the fixed `odom` frame).

**Correcting Odom Drift (`map → odom`)**

- Because dead reckoning accumulates error over time, the `odom → base_link` position will eventually drift away from reality.
- To prevent the car from violently jerking to correct this error (would break continuous control loops), the `odom → base_link` transform is never snapped back to reality.
- Instead, a localization node (SLAM) calculates the exact amount of accumulated drift and publishes a `map → odom` transform. This acts as an offset, shifting the entire `odom` frame back to align with the real-world `map`, ensuring the final `map → base_link` chain is accurate while keeping local movement perfectly smooth.

- These two are easy to mix up, they usually come from the same node and the same numbers, but they do different jobs. The topic is data to read, the transform is for chaining coordinate frames. While purely reactive nodes (ftg) only need static sensor transforms, trajectory trackers that need to track the car's continuous movement in space rely on this odom transform.
- Reliable odometry is also what our TTC safety logic needs (see Safety above).

**Splitting up `map` and `odom`**

- Speed: localization updates slower (10 to 40Hz) than control needs (50 to 100+Hz). Splitting lets fast nodes work asynchronously with slow nodes.
- Safety: jumps only ever happen in map to odom. Odom to base_link never jumps, so PID and follow the gap never get destabilized mid turn.
- Simplicity: anything that wants the car's map position (costmap, raceline follower, RViz) just asks for it, and tf2 chains map, odom, and base_link together automatically.

**Debugging:** use `ros2 run tf2_ros tf2_echo` to show a live transform, and use `ros2 run rqt_tf_tree rqt_tf_tree` to see the whole tree for debugging purposes.

## AI Driving

(No notes yet.)

---



## Setting up SLAM & PF

**SLAM (Simultaneous Localization and Mapping)** uses LiDAR scans and TF transforms to build a 2D map while estimating the car's position. **Particle Filter (PF)** uses an existing map, LiDAR, and odometry to estimate the car's position.

**References:**

- [SFU Racerbot Tutorial 5](https://github.com/sfu-racerbot/racerbot-docs/blob/main/notes/TUTORIAL-5.md)
- [SFU Racerbot Workspace](https://github.com/sfu-racerbot/racerbot_ws)
- [F1TENTH Simulator](https://github.com/f1tenth/f1tenth_gym_ros/tree/dev-humble)
- [SFU Racerbot Map Configuration](https://github.com/sfu-racerbot/racerbot_a/blob/main/src/maps/README.md)

### First-Time Setup (If Never Used SLAM/PF)

These instructions assume the Racerbot workspace, Docker container, and F1TENTH simulator are already installed.

#### Installing SLAM Toolbox

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-slam-toolbox
```

#### Installing Particle Filter

Clone the dependencies into your workspace (**skip repositories already cloned**):

```bash
cd /racerbot_ws/src

git clone https://github.com/f1tenth/range_libc.git -b humble-devel
git clone https://github.com/f1tenth/particle_filter.git -b humble-devel
```

Install dependencies and build without CUDA:

```bash
cd /racerbot_ws
python3 -m pip install Cython
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-select range_lib --cmake-args -DWITH_CUDA=OFF
colcon build
source install/setup.bash
```

Edit `/racerbot_ws/src/particle_filter/config/localize.yaml` and make sure these parameters are set **under `particle_filter: ros__parameters:`**:

```yaml
scan_topic: "/scan"
odometry_topic: "/ego_racecar/odom"
range_method: "rm"
viz: 1
```

These settings use the simulator's LiDAR and odometry topics, CPU raycasting, and PF visualization.

---

### Starting a Session

Start your existing Docker container:

```bash
docker start -ai racerbot
```

Inside Docker, start tmux and source the ROS 2 environment:

```bash
cd /racerbot_ws
tmux
source /racerbot_ws/.venv/bin/activate
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

Launch the F1TENTH simulator and connect Foxglove as normally done.

Launch your movement/path-detection node (FTG, Disparity Extender, etc.) in another tmux window. Keep it running during SLAM or PF testing.

**Reminder:** Use `Ctrl+B`, then `C` to create another tmux window. Source the ROS 2 environment again in each new window.

---

### Running SLAM

#### Launch SLAM

With the simulator and movement node running, open another sourced tmux window:

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -r __node:=slam_toolbox \
  --params-file /racerbot_ws/src/f1tenth_gym_ros/launch/slam/f1tenth_gym_online_async.yaml \
  -r map:=/slam_map \
  -p use_sim_time:=false
```

This uses the simulator-specific TF configuration and publishes the generated map to `/slam_map`, separate from the simulator's original `/map`.

#### Foxglove Configuration

In Foxglove's **3D panel**:

1. Set both **Fixed frame** and **Display frame** to `slam_map`.

![Selecting slam_map from the frame options](docs/assets/foxglove-slam-frame-menu.png)

![slam_map selected for both Fixed frame and Display frame](docs/assets/foxglove-slam-frame-settings.png)

2. Enable `/slam_map` and `/scan`.
3. Disable the simulator's original `/map` topic to avoid overlapping maps.

![Foxglove topic visibility for SLAM mapping](docs/assets/foxglove-slam-topic-visibility.png)

The occupancy grid should fill in as the car explores the environment.

#### Saving the Map

Once mapping is complete, open another **sourced** tmux window:

```bash
mkdir -p /racerbot_ws/src/mapping_output

ros2 run nav2_map_server map_saver_cli \
  -t /slam_map \
  -f /racerbot_ws/src/mapping_output/track_slam \
  --ros-args -p map_subscribe_transient_local:=true
```

This creates

- `track_slam.yaml` -> map pixel resolution and origin.
- `track_slam.pgm` ->  occupancy grid image (Actual map).

These files can be reused for localization.

#### Using the SLAM-Generated Map in the Simulator

To make the simulator use the map created by SLAM, follow the map changing inctructions in the [SFU Racerbot Maps README](https://github.com/sfu-racerbot/racerbot_a/blob/main/src/maps/README.md).

**1. Copy the generated map files** into the simulator's maps directory:

```bash
cp /racerbot_ws/src/mapping_output/track_slam.{yaml,pgm} \
   /racerbot_ws/src/f1tenth_gym_ros/maps/
```

Keep both files together. The YAML file's `image` field should reference `track_slam.pgm`:

```yaml
image: track_slam.pgm
```

Preserve the saved resolution, origin, and occupancy parameters generated by SLAM rather than replacing them with example values.

**2. Change the simulator's map configuration.**

Open:

```text
/racerbot_ws/src/f1tenth_gym_ros/config/sim.yaml
```

Find the `map_path` parameter and change it to:

```yaml
map_path: 'maps/track_slam'
```

The simulator loads the corresponding YAML file and its referenced map image. The path should not include the file extension.

**3. Rebuild and source the workspace:**

```bash
cd /racerbot_ws
colcon build --packages-select f1tenth_gym_ros
source install/local_setup.bash
```

**4. Restart the simulator** to load the new map.

Verify that `/map` now displays the saved SLAM map. If necessary, adjust the car's initial position in `sim.yaml`:

```yaml
sx: 0.0
sy: 0.0
stheta: 0.0
```

These are example values only. Set these to an unobstructed starting postiion for the car depending on the track.

The simulator's existing map server will now provide this map to PF. 

**Note:** A SLAM-generated occupancy grid may contain unexplored areas, ensure that the track is completely mapped out before using this map.

---

### Running Particle Filter

#### Launch PF

Keep the simulator and movement node running. Stop SLAM if mapping is complete, then run PF in a sourced tmux window:

```bash
ros2 run particle_filter particle_filter --ros-args \
  --params-file /racerbot_ws/src/particle_filter/config/localize.yaml
```

PF obtains its map through the simulator's existing `/map_server/map` service, avoiding a duplicate map server.

If the simulator's `map_path` was updated to the SLAM-generated map, PF will use that map automatically.

#### PF Visualization in Foxglove

In Foxglove's **3D panel**:

1. Set both **Fixed frame** and **Display frame** to `map`.
2. Enable these topics:

| Topic |
| --- |
| `/map` |
| `/pf/viz/inferred_pose` |
| `/pf/viz/particles` |
| `/scan` |

3. Use the **2D Pose Estimate** tool to publish to `/initialpose` and initialize the car's position and orientation.

**Expected Result:** Particles should concentrate around the estimated position and follow the car as it moves.

**Note:** In simulation, publishing to `/initialpose` can also reset the simulator's car position.

#### Checking PF Frequency

```bash
ros2 topic hz /pf/viz/inferred_pose
```

At least 30 Hz is a good target.

#### Using a SLAM-Generated Map

By default, PF uses the map provided by the simulator's existing map server.

To use a SLAM-generated map, follow the steps in **Using the SLAM-Generated Map in the Simulator** above to update `map_path` and restart the simulator.

No changes to the PF launch command are required.

---

### Quick Setup (If Packages Are Already Installed)

1. Start Docker, open tmux, and source the ROS 2 environment.
2. Launch the F1TENTH simulator and your movement node.
3. Connect Foxglove as usual.
4. **SLAM:** Run the SLAM command above. Set the Foxglove frame to `slam_map` and display `/slam_map` and `/scan`.
5. **Save map:** Use `map_saver_cli` to save the generated map.
6. **Change map (optional):** Copy the generated map files into the simulator's `maps` directory, update `map_path` in `sim.yaml`, rebuild, and restart the simulator.
7. **PF:** Run the PF command above. Set the Foxglove frame to `map` and enable `/map`, `/pf/viz/inferred_pose`, `/pf/viz/particles`, and `/scan`.
8. Initialize PF with **2D Pose Estimate** and verify that the particles converge around the car.
