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

        public static final MagnetSensorConfigs m_magnetConfigs = new MagnetSensorConfigs();

        static {
            m_driveConfig.Slot0
                .withKP(SwerveConstants.driveP)
                .withKV(SwerveConstants.driveV);
            m_driveConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SwerveConstants.driveMotorCurrentLimit);
            m_driveConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
            m_driveConfig.Feedback
                .withSensorToMechanismRatio(SwerveConstants.kDriveConversionFactor);

            m_turnConfig.Slot0
                .withKP(SwerveConstants.kTurnP)
                .withKD(SwerveConstants.kTurnD);
            m_turnConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SwerveConstants.turnMotorCurrentLimit);
            m_turnConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
            m_turnConfig.Feedback
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
                // .withSensorToMechanismRatio(SwerveConstants.kTurnGearRatio);
            m_turnConfig.ClosedLoopGeneral
                .withContinuousWrap(true);
                

            m_magnetConfigs
                .withAbsoluteSensorDiscontinuityPoint(SwerveConstants.kCANcoderAbsDiscontPoint)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
                
        }
    }
}
