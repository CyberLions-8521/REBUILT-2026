// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
        public static final int kFrontLeftDriveID  = 3;
        public static final int kFrontLeftTurnID   = 1;
        public static final int kFrontRightDriveID = 14;
        public static final int kFrontRightTurnID  = 10;
        public static final int kBackLeftDriveID   = 21;
        public static final int kBackLeftTurnID    = 25;
        public static final int kBackRightDriveID  = 7;
        public static final int kBackRightTurnID   = 13;

        // Note: CANcoder CAN IDs are on a separate CAN bus than SparkMAXs
        // allowing for duplicates between CANcoders and SparkMAXs
        public static final int kFrontLeftMagEncoderID  = 2; //set later
        public static final int kFrontRightMagEncoderID = 3; //set later
        public static final int kBackLeftMagEncoderID   = 0; //set later
        public static final int kBackRightMagEncoderID  = 1; //set later

        public static final double kWheelBase = Units.inchesToMeters(19.5);    // x-direction of robot
        public static final double kTrackWidth = Units.inchesToMeters(19.5);   // y-direction of robot

        public static final double kMaxMetersPerSecond = 3.0;
        public static final double kMaxAngularSpeed = 2 * Math.PI;  // radians

        public static final double kSRXEncoderMaxValue = 1;
        public static final double kFrontLeftMagEncoderMagnetOffset  = 0.336;     // set later, measure in Phoenix tuner x
        public static final double kFrontRightMagEncoderMagnetOffset = 0.467;     // set later, measure in Phoenix tuner x
        public static final double kBackLeftMagEncoderMagnetOffset   = 0.874;     // set later, measure in Phoenix tuner x
        public static final double kBackRightMagEncoderMagnetOffset  = 0.302;     // set later, measure in Phoenix tuner x

        public static final double kStrafeP = 0.0;
        public static final double kStrafeI = 0.0;
        public static final double kStrafeD = 0.0;

        public static final double kAutoAlignSpeed = 0.5;
    }

    public static final class LimelightConstants {
        public static final String limelightName = "limelight";
        public static final double outpostShootingRadius = 0; //set later
        public static final double outpostShootingRadiusTolerance = 0; //set later
        
    }

    public static final class LimitSwitchConstants {
        public static final int kLeftLimitSwitchID = 0; 
        public static final int kRightLimitSwitchID = 4; 
    }

    public static final class CANdleConstants {
        public static final int ledCount = 240;
        public static final int CANdleID = 4;
    }
}
