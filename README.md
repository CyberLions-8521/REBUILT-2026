# Branch for my version of the robot code

May be possibly used for the SoCal competition

**Currently based on WPILIB 2027 Alpha 7**

**Vendor Dependencies:**
* **Commands v2**
* **CTRE-Phoenix (v6)**
* **LimelightLib 2**
* **PathplannerLib (currently does not work in alpha 7 YET)**
* **REVLib**

Anything below with a checkmark [✅] has been tested on the robot and confirmed to work.

Added:
* Odometry [✅]
* Simulation [✅]
* Pathplanner [✅]
* Odometry-based auto-align [✅]
* Odometry-based auto-distance [✅]
* Elastic dashboard [✅]
* Dynamic PID tuning [✅]
* Limelight visionary pose estimation [✅]

The odometry based auto-align and auto-distancing can take in any Translation2d object (any point on the field) and can target it. So anything is possible including using it for slightly easier passing, etc.

(The branch Ethan-Drivebase is the original branch and the branch before I re-added the subsystems back onto the drivebase.)

* Includes 2 Translation2d objects that allow the odometry-alignment commands to point to either hub depending on the alliance
* Includes 7 autos designed in PathPlanner + 3 in code
    * [List of autos here](/image-assets/autos/README.md)

## Controls

<div align="center">
    <img src="/image-assets/bindings.png" width="800">
</div>