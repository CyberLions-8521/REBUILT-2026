# Branch for my version of the robot code

May be possibly used for the SoCal competition

Anything below with a checkmark [✅] has been tested on the robot and confirmed to work.

Added:
* Odometry [✅]
* Simulation [✅]
* Pathplanner [✅]
* Odometry-based auto-align [✅]
* Odometry-based auto-distance [✅]
* Elastic dashboard 
* Dynamic PID tuning [✅]
* Limelight visionary pose estimation

The odometry based auto-align and auto-distancing can take in any Translation2d object (any point on the field) and can target it. So anything is possible including using it for slightly easier passing, etc.

(The branch Ethan-Drivebase is the original branch and the branch before I re-added the subsystems back onto the drivebase.)

* Includes 2 Translation2d objects that allow the odometry-alignment commands to point to either hub depending on the alliance
* Includes 9 autos designed in PathPlanner
    * No movement but still updates the starting pose for odometry [L/M/R] (Emergency stop equivalent)
    * Only shoot the preloaded fuel [L/M/R]
    * Shoot the preloaded fuel and collect from the neutral zone [L/R]
    * Collect fuel from the outpost and shoot [R]

## Controls

<div align="center">
    <img src="/image-assets/bindings.png" width="800">
</div>