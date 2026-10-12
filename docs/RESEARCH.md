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

- Speed
  - Decreasing the speed caused the car to crash into more corners. Added safety bubble to try and counteract this.
  - Speed should be the lower of two limits: slow down for sharp turns, and slow down when the target point is close. This can sometimes be too slow and may need tuning.
- Drunk (snaking left and right)
  - The car steers toward the furthest point, which constantly shifts each time.
  - Aiming for the middle gap is basically FTG and somewhat defeats the idea of DE. To be investigated.
  - Steering is smoothed with a low-pass filter: each new angle is blended with the previous one (`alpha`), so a single jumpy scan only nudges the car instead of jerking it. Lower `alpha` means smoother but slower reactions.
- Obstacle avoidance
  - Too much disparity extending slowly sends the car toward walls; too little causes crashes in corners.
  - A safety bubble did not work well for avoiding corners and takes over the role of the disparity extender.
- Least-squares steering
  - The car seemed to drive in S-curves on straightaways.
  - The hypothesis was that the car exited turns slightly turned, drove toward a wall, and then over-corrected toward the other wall. This process compounded over time.
  - Applying a deviation penalty either drove the car straight into a wall or had no effect, regardless of the penalty factor.
  - Limiting the returned steering angle to the car's turning range also had no effect.
  - A slew-rate limiter capped how fast the turning angle could change per callback tick. This produced smoother turns, helped the car stay straight after turns, and fixed the S-curves issue.

## AI Driving

(No notes yet.)

---
