package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    public Limelight() {

        //REQUIREMENTS FOR MEGATAG 2:
        // Your Limelight's robot-space pose has been configured in the webUI or via the API
        // A field map (.fmap) has been uploaded
        // LimelightHelpers.SetRobotOrientation(robotYawInDegrees,0,0,0,0,0) is called every frame in robot-side code
        // SetRobotOrientation assumes a centered (see the map generator) or blue-corner origin. CCW-positive, 0 degrees -> facing red alliance wall in FRC.

        LimelightHelpers.setPipelineIndex("", 0);

        LimelightHelpers.setCameraPose_RobotSpace("", 
            //turn these into constants later
            0.5,    // Forward offset (meters)
            0.0,    // Side offset (meters)
            0.5,    // Height offset (meters)
            0.0,    // Roll (degrees)
            30.0,   // Pitch (degrees)
            0.0     // Yaw (degrees)
        );

        LimelightHelpers.setFiducial3DOffset("", 
            //to top of hub opening from center apriltag
            0.5969,    // Forward offset
            0.0,    // Side offset  
            0.70485     // Height offset
        );

        final int[] hubIDs = new int[]{9,10,8,5,11,2,18,27,21,24,26,25};
        LimelightHelpers.SetFiducialIDFiltersOverride("", hubIDs); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride("", 1.0f); //downscale by 1x (no effect)

        if(LimelightHelpers.getTV("")) {
                SmartDashboard.putBoolean("TV", LimelightHelpers.getTV(""));
        }
    }

    @Override
    public void periodic() {
        PoseEstimate PoseEst = new PoseEstimate();

        // DEFAULT CONSTRUCTOR
        // public PoseEstimate(Pose2d pose,
        // double timestampSeconds,
        // double latency,
        // int tagCount,
        // double tagSpan,
        // double avgTagDist,
        // double avgTagArea,
        // LimelightHelpers.RawFiducial[] rawFiducials,
        // boolean isMegaTag2)
         
        // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_orb").getDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

    }
}   