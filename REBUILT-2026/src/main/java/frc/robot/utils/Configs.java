// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
        public static final TalonFXConfiguration m_driveConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration m_turnConfig = new TalonFXConfiguration();

        static {
            m_driveConfig.Slot0
                .withKP(SwerveConstants.driveP)
                .withKD(SwerveConstants.driveD)
                .withKS(SwerveConstants.driveS)
                .withKD(SwerveConstants.driveV);
            m_driveConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SwerveConstants.driveMotorCurrentLimit);
            m_driveConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
            m_driveConfig.Feedback
                .withSensorToMechanismRatio(SwerveConstants.kDriveConversionFactor)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

            m_turnConfig.Slot0
                .withKP(SwerveConstants.turnP)
                .withKD(SwerveConstants.driveD)
                .withKS(SwerveConstants.turnS);
            m_turnConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SwerveConstants.turnMotorCurrentLimit);
            m_turnConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
            m_turnConfig.Feedback
                .withSensorToMechanismRatio(SwerveConstants.kTurnConversionFactor);
            m_turnConfig.ClosedLoopGeneral
                .withContinuousWrap(true);
        }
    }
}
