// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;


/** Add your docs here. */
public class Constants {

    public static final class SwerveConstants {
        public static final int turnMotorFreeLimit = 20;    // current limits in amps
        public static final int turnMotorStallLimit = 20;   // current limits in amps
        public static final int driveMotorFreeLimit = 40;   // current limits in amps
        public static final int driveMotorStallLimit = 40;  // current limits in amps

        private static final double kWheelDiameter = Units.inchesToMeters(4);
        private static final double kWheelCircumference = Math.PI * kWheelDiameter;
        private static final double kDriveGearRatio = 6.75;     // found on SDS page for MK4
        public static final double kDriveConversionFactor = kWheelCircumference / kDriveGearRatio;  // meters (of robot travel)

        private static final double kTurnGearRatio = 12.8;     // found on SDS page for MK4
        public static final double kAngleConversion = 360;              // degrees
        public static final double kTurnConversionFactor = kAngleConversion / kTurnGearRatio;       // degrees (of output shaft)

        private static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0;      // neo free rpm = 5676 rpm
        private static final double kDriveWheelFreeSpeedRps = kDrivingMotorFreeSpeedRps * kDriveConversionFactor;
        public static final double driveFF = 1.0 / kDriveWheelFreeSpeedRps;
        public static final double driveP = 0.025;
        public static final double driveI = 0;
        public static final double driveD = 0;
        public static final double turnP = 0.04;
        public static final double turnI = 0;
        public static final double turnD = 0.01;        
        public static final String kCANCoderBus = "Ryan";   // name assigned in Phoenix Tuner X

        public static final double kSlewRateLimiter = 3.0;
        public static final int kFrontLeftDriveID  = 7;
        public static final int kFrontLeftTurnID   = 13;
        public static final int kFrontRightDriveID = 21;
        public static final int kFrontRightTurnID  = 25;
        public static final int kBackLeftDriveID   = 14;
        public static final int kBackLeftTurnID    = 10;
        public static final int kBackRightDriveID  = 3;
        public static final int kBackRightTurnID   = 1;

        // Note: CANcoder CAN IDs are on a separate CAN bus than SparkMAXs
        // allowing for duplicates between CANcoders and SparkMAXs
        public static final int kFrontLeftMagEncoderID  = 1; //set later
        public static final int kFrontRightMagEncoderID = 0; //set later
        public static final int kBackLeftMagEncoderID   = 3; //set later
        public static final int kBackRightMagEncoderID  = 2; //set later

        public static final double kWheelBase = Units.inchesToMeters(19.5);    // x-direction of robot
        public static final double kTrackWidth = Units.inchesToMeters(19.5);   // y-direction of robot

        public static final double kMaxMetersPerSecond = 3.0;
        public static final double kMaxAngularSpeed = 2 * Math.PI;  // radians

        public static final double kSRXEncoderMaxValue = 1;
        public static final double kFrontLeftMagEncoderMagnetOffset  = 0.782056;     // set later, measure in Phoenix tuner x
        public static final double kFrontRightMagEncoderMagnetOffset = 0.387926;     // set later, measure in Phoenix tuner x
        public static final double kBackLeftMagEncoderMagnetOffset   = 0.015398;     // set later, measure in Phoenix tuner x
        public static final double kBackRightMagEncoderMagnetOffset  = 0.833306;     // set later, measure in Phoenix tuner x

        public static final double kStrafeP = 0.0;
        public static final double kStrafeI = 0.0;
        public static final double kStrafeD = 0.0;

        public static final double kAutoAlignSpeed = 0.5;
    }

    public static final class LimelightConstants {
        public static final String limelightName = "limelight3.0";
        public static final double outpostShootingRadius = 0; //set later
        public static final double outpostShootingRadiusTolerance = 0; //set later
        
    }
}
