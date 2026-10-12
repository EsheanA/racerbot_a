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
    - We choose width*depth gap: Solid.
    - We account for how much we must turn: To be tested.
    - We choose center of widest gap: Solid.

- Steering
    - Go to best point: Car oversteers, as it chooses a new point each time.

- Speed
    - Slower speeds generally crash less and are closer to matching real life vs. simulation.
    - No solid speed function yet.

### Disparity Extender (DE)

- Speed
    - Decreasing the speed caused the car to crash into more corners. Added saftey bubble to try and counteract this. Good
    - should be the lower of two limits: slow down for sharp turns, and slow down when the target point is close. Okay but sometimes can be too slow, might or might not be a tuning problem.
- Drunk (snaking left and right)
    - The car is drunk because it steers toward the furthest point, which constantly shifts each time
    - Aim for middle gap: Basically FTG if we do that, somewhat defeats the idea of DE. To be looked into
    - Steering is smoothed with a low-pass filter: each new angle is blended with the previous one (`alpha`), so a single jumpy scan only nudges the car instead of jerking it. Lower `alpha` = smoother but slower to react.
- Obstacle avoidance
    - Too much disparity extending and we slowly go to walls, too little and we crash corners
    - Tried using safety bubble to avoid crashing into corners but this isn't good because it doesn't work well and is taking the role of disparity extender.

### Least Squares
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
- Sensors only know the world in their own frame of reference. A LiDAR point returns a distance from a wall and its angle, which is  measured from the sensor itself, in `laser_frame`. On its own, we cannot tell exactly where the point is relative to the car.
- When LiDAR is mounted, we measure exactly how close `laser_frame` sits relative to `base_link` (for example, 15cm forward of the axle, facing straight ahead), and publish that as a static transform. (This never changes since the components are bolted down.)
- Applying that transform is what allows us to make sense of the LiDAR readings, using the static transform as a frame of reference (ex. LiDAR scans 2m ahead of sensor -> 2.15m ahead of `base_link`). Specifically, it is a rigid body transform, a rotation and a translation, no stretching, since the sensor and chassis don't flex relative to each other ([ROS 2 tf2 tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)).
- Because `base_link` already chains up to `odom` and `map`, that same LiDAR point can then be expressed relative to either of those too. The LiDAR node never needs to know odometry or localization exist.
- `imu_frame` works the same way, a fixed offset to `base_link`, so IMU readings can be used in the car's own frame.

**Odometry (`odom` frame and `odom → base_link` transform)**
- Whatever node reads the wheels/VESC (plus IMU if fused) does two separate things with the result:
  1. Publishes it as data, a `nav_msgs/Odometry` message on a topic (`/odom`), carrying the robots pose and its velocity, for anything that wants the actual numbers, like an EKF or a logger ([nav_msgs/Odometry message](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html)).
  2.  Publishes the matching `odom → base_link` transform onto `/tf`, at the same rate. This transform represents the car's smooth, continuous trajectory based purely on dead reckoning, measuring how far it has traveled relative to its starting point (the fixed `odom` frame).

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
