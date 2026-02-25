package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class ControllerConstants {
        public static final double kDeadband = 0.2;
    }
    
    public static final class SwerveConstants {
        public static final int driveMotorCurrentLimit = 85;
        public static final int turnMotorCurrentLimit = 75;

        private static final double kWheelDiameter = Units.inchesToMeters(4);
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;
        private static final double kDriveGearRatio = 6.75;     // found on SDS page for MK4i
        public static final double kDriveConversionFactor = kWheelCircumference / kDriveGearRatio;  // meters (of robot travel)

        private static final double kTurnGearRatio = (150.0 / 7.0);     // found on SDS page for MK4i
        public static final double kAngleConversion = 360;              // degrees
        public static final double kTurnConversionFactor = kAngleConversion / kTurnGearRatio;       // degrees (of output shaft)

        private static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0;      // neo free rpm = 5676 rpm
        private static final double kDriveWheelFreeSpeedRps = kDrivingMotorFreeSpeedRps * kDriveConversionFactor;
        public static final double driveP = 0; //set later
        public static final double driveD = 0; //set later
        public static final double driveV = 0; //used for feedforward
        public static final double driveS = 0; //set later
        public static final double turnP = 0; //set later
        public static final double turnD = 0; //set ;ater
        public static final double turnS = 0; //set later
        public static final String kCANCoderBus = "Ryan";   // name assigned in Phoenix Tuner X

        public static final double kSlewRateLimiter = 3.0;
        public static final int kFrontLeftDriveID  = 1; 
        public static final int kFrontLeftTurnID   = 2; 
        public static final int kFrontRightDriveID = 3; 
        public static final int kFrontRightTurnID  = 4; 
        public static final int kBackLeftDriveID   = 5; 
        public static final int kBackLeftTurnID    = 6; 
        public static final int kBackRightDriveID  = 7; 
        public static final int kBackRightTurnID   = 8; 

        public static final int kFrontLeftAbsEncoderID  = 2;
        public static final int kFrontRightAbsEncoderID = 3;
        public static final int kBackLeftAbsEncoderID   = 0;
        public static final int kBackRightAbsEncoderID  = 1;

        public static final double kWheelBase = Units.inchesToMeters(22.5625);    // x-direction of robot, set later
        public static final double kTrackWidth = Units.inchesToMeters(22.5625);   // y-direction of robot, set later

        public static final double kMaxMetersPerSecond = 4.0; //tune later
        public static final double kMaxAngularSpeed = 2 * Math.PI;  // radians

        public static final double kFrontLeftAbsEncoderMagnetOffset  = 0.141319;     // set later
        public static final double kFrontRightAbsEncoderMagnetOffset = 0.288254;     // set later
        public static final double kBackLeftAbsEncoderMagnetOffset   = 0.190155;     // set later
        public static final double kBackRightAbsEncoderMagnetOffset  = 0.094242;     // set later

        //Dimensions for PathPlanner
        public static final double kDriveBaseRadius = 0.0; // need to measure
    }

    public static final class PathPlannerConstants {
        // PID for PathPlanner position correction
        public static final double translationP = 0;
        public static final double translationI = 0;
        public static final double translationD = 0;

        public static final double rotationP = 0;
        public static final double rotationI = 0;
        public static final double rotationD = 0;
    }


    public static class OperatorConstants {
        public static final int kDriveControllerPort = 0;
        public static final int kCommandControllerPort = 1;
    }

    public static class LimelightConstants {
        public static final String kName = "limelight-twoplus";

        //tuning auto align
        public static final double kAimP = 0.035;
        public static final double kRangeP = 0.1;
    }
}