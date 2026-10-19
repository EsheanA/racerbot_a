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

    - Can cause the car to snake much more when driving down wider corridors.
        - Current Solution: Decrease safety bubble size on obstacles the car is unlikely to hit.
        - Current Implementation:
            - Decrease the size of the bubble if the obstacle is away from the car's facing direction and not very close to the car.
            - Parameters have not been tested thoroughly yet.
            - Snaking still occurs without the safety bubble

- Finding Gap
    - We choose widest gap: Backfires when a wide but shallow gap wins over a narrower, deeper one, steering us toward a dead end.
    - We choose width*depth gap: Solid.
    - We account for how much we must turn: To be tested.
    - We choose center of widest gap: Solid.

- Steering
    - Go to best point: Car oversteers, as it chooses a new point each time.
    - Smoothing the steering with a low-pass filter seems to have very little effect.

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

## AI Driving

(No notes yet.)