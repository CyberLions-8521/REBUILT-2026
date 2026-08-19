// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class SwerveConstants {

    //======== Motor IDs ================================================================

        public static final int kFrontLeftDriveID  = 1; 
        public static final int kFrontLeftTurnID   = 2; 
        public static final int kFrontRightDriveID = 3; 
        public static final int kFrontRightTurnID  = 4; 
        public static final int kBackLeftDriveID   = 7; 
        public static final int kBackLeftTurnID    = 8; 
        public static final int kBackRightDriveID  = 5; 
        public static final int kBackRightTurnID   = 6; 

        public static final int kFrontLeftCANCoderID  = 11;
        public static final int kFrontRightCANCoderID = 12;
        public static final int kBackLeftCANCoderID   = 13;
        public static final int kBackRightCANCoderID  = 14;

    //======== Motor Configurations =====================================================

        public static final int kDriveCurrentLimit = 85;
        public static final int kTurnCurrentLimit = 75;

        public static final double kFrontLeftCANCoderMagnetOffset  = -0.10620;    
        public static final double kFrontRightCANCoderMagnetOffset = 0.217285;     
        public static final double kBackLeftCANCoderMagnetOffset   = -0.06372;     
        public static final double kBackRightCANCoderMagnetOffset  = -0.17163;   

        public static final double kCANcoderAbsDiscontPoint = 0.5;

    //======== Swerve Drivebase ========================================================

        private static final double kWheelDiameter = Units.inchesToMeters(4);
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;

        private static final double kDriveGearRatio = 6.75;     // found on SDS page for MK4i
        public static final double kTurnGearRatio = (150.0 / 7.0);     // found on SDS page for MK4i

        public static final double kDriveConversionFactor = kDriveGearRatio / kWheelCircumference;  // meters (of robot travel)

        public static final double kWheelBase = Units.inchesToMeters(22.5625);    // x-direction of robot, set later
        public static final double kTrackWidth = Units.inchesToMeters(22.5625);   // y-direction of robot, set later

        public static final double kMaxMetersPerSecond = 3.3; //oriignal is 4.4 
        public static final double kMaxAngularSpeed = 2 * Math.PI;  // radians

        public static final String kCANBus = "Circus Circle";   // name assigned in Phoenix Tuner X

        public static final double kSlewRateLimiter = 3.0;

    //======== Tuning ==================================================================

        public static final double kDriveP = 3.5; //set later 3.5 (oriingal)
        public static final double kDriveV = 2.4; //used for feed forward 2.4 (original)

        public static final double kTurnP = 120; //set later
        public static final double kTurnD = 1.5; //set later

    //======== Auto-align ===================================================================

        public static final double kMaxAcceleration = kMaxMetersPerSecond * 2; // meters (takes half a second to get to max accel)
        public static final double kMaxAngularAcceleration = kMaxAngularSpeed * 2; // radians/sec^2 (takes half a second to get to max accel)

        public static final double kAutoAlignTolerance = Units.degreesToRadians(2.0);
        public static final double kAutoDistanceTolerance = 0.05; // meters

        public static final double kAutoAlignP = 25.0;
        public static final double kAutoAlignI = 0.0;
        public static final double kAutoAlignD = 0.5;

        public static final double kAutoDistanceP = 35.0;
        public static final double kAutoDistanceI = 0.0;
        public static final double kAutoDistanceD = 0.5;
        public static final double kAutoDistanceTarget = 2.5; // meters

    //======== Pathplanner ==================================================================
        public static final double kTranslationP = 5.0;
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;

        public static final double kRotationP = 5.0;
        public static final double kRotationI = 0.0;
        public static final double kRotationD = 0.0;

    }

    public static final class LimelightConstants {
        public static final String limelightName = "limelight";
        public static final double kMaxViableGyroRate = 720.0; // degrees 
    }

    public static final class ShooterConstants {

    //======== Motor IDs ================================================================


        public static final int kHoodID = 20;


        public static final int kShooterTopLeftID = 21;
        public static final int kShooterTopRightID = 22;
        public static final int kShooterBottomRightID = 23;

    //======== Shooter Subsystem ========================================================




        public static final String kCanbusName = "rio";
        
        public static double kFlywheelVelocityInput = 0.0;
        public static final double kBottomMotorRatio = 0.8; // run the bottom hood slower for backspin (stabler shot)
        
        public static final double kMaxRot = 8.8;
        public static final double kHoodDegsToRot = 0.296;

        public static final double kShooterHeight = 0.5842; // 23 in to m
        public static final double kDrivebaseHeight = 0.1778; // 7 in
        public static final double kStartingHeight = kShooterHeight + kDrivebaseHeight;
        public static final double kHubHeight = 1.8288; // 72 in (per game manual)
        public static final double kDeltaHeight = kHubHeight - kStartingHeight; 

    //======== Tuning ==================================================================

        public static final double kHoodP = 0.2; //original: 0.5
        public static final double kHoodI = 0.0;
        public static final double kHoodD = 0.0;
        public static final double kHoodFeedForward = 0.229; // final later
        public static final double kHoodLowDegFromHorizontal = 40;

        public static final double kShooterP = 0.2;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterS = 0.25;
        public static final double kShooterV = 0.1185;
        // adjust these later
        public static final double kMinShooterRange = 1.5; // minimum distance the robot can shoot from
        public static final double kMaxShooterRange = 5.8; // maximum distance the robot can shoot from OR that the limelight can see
        public static final double kMinShooterVelocity = 0.0; // minimum velocity the robot must be
        public static final double kMaxShooterVelocity = 80.0;

        public static final double kGravity = 9.807;
        
        public static final double kA = 36.74;
        public static final double kB = 4.59;

        // in meters
        // field measurements from field dimension drawings pdf (public)
        public static final double aprilTagToHub = 0.597;
        public static final double limelightToRobotCenter = 0.305; // 12 in to m

    }

    public static final class LimitSwitchConstants {
        public static final int kLeftLimitSwitchID = 0; 
        public static final int kRightLimitSwitchID = 4; 
    }

    public static final class CANdleConstants {
        public static final String kCanbusName = "Ryan";

        public static final int kLedCount = 256;
        public static final int kCANdleID = 15;
    }

    public static class IntakeConstants {

    //======== Motor IDs ================================================================
        
        public static final int kPivotID = 12;    
        public static final int kIntakeID = 10;   

    //======== Motor Configurations ================================================================

        public static final int pivotCurrentLimit = 20;
        public static final int intakeCurrentLimit = 20;

    //======== Intake Subsystem ================================================================

        public static final double kExtendedEncoderPosition = -4.5; //set later
        public static final double kRetractedEncoderPosition = -2; //set later
        public static final double kRetractedMiddleEncoderPosition = -2.5;
        
        public static final String kCanbusName = "rio";
        public static final double kGearRatio = 14.5;              
        public static final double kGearCircumference = 0; 
        public static final double kMiddleEncoderPosition = -2.56; //for after ventura

        public static  double rollerP = 0;
        public static  double rollerV = 0;

        public static  double pivotP = 0;
        public static  double pivotD = 0;
        public static  double pivotG = 0;

        public static final double extendedEncoderPosition = -4.5;
        public static final double retractedEncoderPosition = -2;
        public static final double middleEncoderPosition = -2.56; 
        
    }

    public static class IndexerConstants {
    //======== Motor IDs ================================================================
        public static final int kIndexerID = 0;     

    //======== Motor Configurations ================================================================

        public static final int kRollerCurrentLimit = 20;


    //======== Indexer Subsystem ================================================================

        public static final String kCanbusName = "rio";
        public static final int kGearRatio = 0;               //NEED TO CHANGE
        public static final int kGearCircumference = 0;       //NEED TO CHANGE
        
    }

}
