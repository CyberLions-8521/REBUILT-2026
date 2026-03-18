// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class Configs {
    public final static class SwerveConfigs {
        public static final SparkMaxConfig m_configDrive = new SparkMaxConfig();
        public static final SparkMaxConfig m_configTurn = new SparkMaxConfig();

        static {
            m_configDrive
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(SwerveConstants.driveMotorStallLimit, SwerveConstants.driveMotorFreeLimit);

            m_configTurn
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(SwerveConstants.turnMotorStallLimit, SwerveConstants.turnMotorFreeLimit);

            m_configDrive.encoder
                .positionConversionFactor(SwerveConstants.kDriveConversionFactor)           // meters
                .velocityConversionFactor(SwerveConstants.kDriveConversionFactor / 60.0);   // meters per second

            m_configTurn.encoder
                .positionConversionFactor(SwerveConstants.kTurnConversionFactor)            // degrees
                .velocityConversionFactor(SwerveConstants.kTurnConversionFactor / 60.0);    // degrees per second

            m_configDrive.closedLoop
                .pidf(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD, SwerveConstants.driveFF)
                .outputRange(-1, 1)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(false);

            m_configTurn.closedLoop
                .pid(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD)
                .outputRange(-1, 1)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, SwerveConstants.kAngleConversion); 
        }
    }

    public final static class CANdleConfigs {
        public static final CANdleConfiguration CANdleConfig = new CANdleConfiguration();

        static {
            CANdleConfig.LED
                .withStripType(StripTypeValue.GRB)
                .withBrightnessScalar(0.3);
        }
    }
}
