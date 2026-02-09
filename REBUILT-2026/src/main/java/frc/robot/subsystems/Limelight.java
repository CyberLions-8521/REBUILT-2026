package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

    public Pose3d TargetPose;
    public Pose3d RobotPose;
    public final int[] hubIDs = new int[]{9,10,8,5,11,2,18,27,21,24,26,25};


    public Limelight() {

        //REQUIREMENTS FOR MEGATAG 2:
        // Your Limelight's robot-space pose has been configured in the webUI or via the API
        // A field map (.fmap) has been uploaded
        // LimelightHelpers.SetRobotOrientation(robotYawInDegrees,0,0,0,0,0) is called every frame in robot-side code
        // SetRobotOrientation assumes a centered (see the map generator) or blue-corner origin. CCW-positive, 0 degrees -> facing red alliance wall in FRC.

        LimelightHelpers.setPipelineIndex("", 0);

        LimelightHelpers.setCameraPose_RobotSpace("", 
            //turn these into constants later
            0.0,    // Forward offset (meters)
            0.0,    // Side offset (meters)
            0.0,    // Height offset (meters)
            0.0,    // Roll (degrees)
            0,   // Pitch (degrees)
            0.0     // Yaw (degrees)
        );

        LimelightHelpers.setFiducial3DOffset("", 
            //to top of hub opening from center apriltag
            0.0,    // Forward offset
            0.0,    // Side offset  
            0.0     // Height offset
        );

        LimelightHelpers.SetFiducialIDFiltersOverride("", hubIDs); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride("", 1.0f); //downscale by 1x (no effect)
    }

    @Override
    public void periodic() {


        //https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
        //TargetPose = Gets the target's 3D pose with respect to the robot's coordinate system.
        //RobotPose =  Gets the robot's 3D pose in the WPILib Blue Alliance Coordinate System. (wpiRed is not recommended)
        //To do:
        //- Check if the methods correspond and are proportional with real world measurements
        //- Figure out when to use TargetPose and RobotPose (or just one)

        TargetPose = LimelightHelpers.getTargetPose3d_RobotSpace("");   
        // RobotPose = LimelightHelpers.getBotPose3d_wpiBlue("");

        SmartDashboard.putNumber("TargetPose X", TargetPose.getX());
        SmartDashboard.putNumber("TargetPose Y", TargetPose.getY());
        SmartDashboard.putNumber("TargetPose Z", TargetPose.getZ());
    }
}   