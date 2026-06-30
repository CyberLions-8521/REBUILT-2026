// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.utils.Constants.SwerveConstants;

/** Add your docs here. */
public class Configs {

    public static final class SwerveConfigs {
        public static final TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration turnConfigs = new TalonFXConfiguration();

        public static final MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();

        static {
            driveConfigs.Slot0
                .withKP(SwerveConstants.kDriveP)
                .withKV(SwerveConstants.kDriveV);
            driveConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SwerveConstants.kDriveCurrentLimit);
            driveConfigs.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
            driveConfigs.Feedback
                .withSensorToMechanismRatio(SwerveConstants.kDriveConversionFactor);

            turnConfigs.Slot0
                .withKP(SwerveConstants.kTurnP)
                .withKD(SwerveConstants.kTurnD);
            turnConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SwerveConstants.kTurnCurrentLimit);
            turnConfigs.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
            turnConfigs.Feedback
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
            turnConfigs.ClosedLoopGeneral
                .withContinuousWrap(true);
                
            magnetConfigs
                .withAbsoluteSensorDiscontinuityPoint(SwerveConstants.kCANcoderAbsDiscontPoint)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        }
    }

}
