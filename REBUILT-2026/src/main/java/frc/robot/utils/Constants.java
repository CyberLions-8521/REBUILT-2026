// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final class SwerveConstants {
        public static final int driveMotorCurrentLimit = 85;
        public static final int turnMotorCurrentLimit = 75;

        private static final double kWheelDiameter = Units.inchesToMeters(4);
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;
        private static final double kDriveGearRatio = 6.75;     // found on SDS page for MK4i
        public static final double kDriveConversionFactor = kDriveGearRatio / kWheelCircumference;  // meters (of robot travel)

        private static final double kTurnGearRatio = (150.0 / 7.0);     // found on SDS page for MK4i
        public static final double kAngleConversion = 360;              // degrees
        public static final double kTurnConversionFactor = 1;     // degrees (of output shaft)

        public static final double driveP = 3.5; //set later
        public static final double driveV = 2.4; //used for feedforward
        public static  double turnP = 0; //set later
        public static  double turnD = 0; //set ;ater
        public static final String kCANCoderBus = "rio";   // name assigned in Phoenix Tuner X

        public static final double kSlewRateLimiter = 3.0;
        public static final int kFrontLeftDriveID  = 1; 
        public static final int kFrontLeftTurnID   = 2; 
        public static final int kFrontRightDriveID = 3; 
        public static final int kFrontRightTurnID  = 4; 
        public static final int kBackLeftDriveID   = 5; 
        public static final int kBackLeftTurnID    = 6; 
        public static final int kBackRightDriveID  = 7; 
        public static final int kBackRightTurnID   = 8; 

        public static final int kFrontLeftCANCoderID  = 11;
        public static final int kFrontRightCANCoderID = 12;
        public static final int kBackLeftCANCoderID   = 13;
        public static final int kBackRightCANCoderID  = 14;

        public static final double kWheelBase = Units.inchesToMeters(22.5625);    // x-direction of robot, set later
        public static final double kTrackWidth = Units.inchesToMeters(22.5625);   // y-direction of robot, set later

        public static final double kMaxMetersPerSecond = 4.0; //tune later
        public static final double kMaxAngularSpeed = 2 * Math.PI;  // radians

        public static final double kCANcoderAbsDiscontPoint = 0.5;
        public static final double kFrontLeftCANCoderMagnetOffset  = 0;     // set later
        public static final double kFrontRightCANCoderMagnetOffset = 0;     // set later
        public static final double kBackLeftCANCoderMagnetOffset   = 0;     // set later
        public static final double kBackRightCANCoderMagnetOffset  = 0;     // set later
    }

    public static final class LimelightConstants {
        public static final String limelightName = "limelight";
        public static final double TXControllerP = 0;
        public static final double TXControllerD = 0;
    }
}
