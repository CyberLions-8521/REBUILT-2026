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

        public static final double kAutoAlignP = 5.0;
        public static final double kAutoAlignI = 0.0;
        public static final double kAutoAlignD = 0.5;

        public static final double kAutoDistanceP = 3.0;
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

}
