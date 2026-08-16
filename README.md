# Branch for my version of the robot code

A little thing that I've been working on during my free-time and for the better of the robotics team. It might be useful for any competitions of REBUILT assuming that everything works. It is mostly the same code, but with the drivetrain code under improvements by me. Below:

Anything below with a checkmark [✅] has been tested on the robot and confirmed to work.

Added:
* Odometry [✅]
* Simulation [✅]
* Pathplanner [✅]
* Odometry-based auto-align [✅]
* Odometry-based auto-distance [✅]
* Elastic dashboard 
* Dynamic PID tuning [✅]
* Limelight visionary pose estimation (likely needs to be configured correctly in RobotContainer)

The odometry based auto-align and auto-distancing can take in any Translation2d object (any point on the field) and can target it. So anything is possible including using it for slightly easier passing, etc.

(The branch Ethan-Drivebase is the original branch and the branch before I re-added the subsystems back onto the drivebase.)

## Bonus

* Already includes 2 Translation2d objects that allow the odometry-alignment commands to point to either hub depending on the alliance

## Controls

<img src="/image-assets/bindings.png" width="700">
