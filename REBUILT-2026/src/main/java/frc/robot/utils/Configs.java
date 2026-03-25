// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import frc.robot.utils.Constants.IndexerConstants;
import frc.robot.utils.Constants.IntakeConstants;
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
            m_turnConfig.ClosedLoopGeneral
                .withContinuousWrap(true);
                

            m_magnetConfigs
                .withAbsoluteSensorDiscontinuityPoint(SwerveConstants.kCANcoderAbsDiscontPoint)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
                
        }
    }

    public static final class ShooterConfigs {
       public static final TalonFXConfiguration upperFlywheelConfigs = new TalonFXConfiguration();
       public static final TalonFXConfiguration lowerFlywheelConfigs = new TalonFXConfiguration();
       
       static {
            upperFlywheelConfigs.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);

            upperFlywheelConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20);
       }

       static {
            lowerFlywheelConfigs.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast);

            lowerFlywheelConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20);
       }
    }

    public static final class HoodConfigs {
       public static final TalonFXConfiguration kKrakenHoodConfig = new TalonFXConfiguration();

       static {
            kKrakenHoodConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
       }
    }


    public static final class IntakeConfigs  {
        public static final TalonFXConfiguration kPivotConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kIntakeConfig = new TalonFXConfiguration();

        static {
            kPivotConfig.Slot0
                .withKP(0)
                .withKD(0)
                .withKI(0);
            kPivotConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IntakeConstants.pivotCurrentLimit);
            kPivotConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
            kPivotConfig.Feedback
                .withSensorToMechanismRatio(IntakeConstants.kGearRatio / IntakeConstants.kGearCircumference);
        
            kIntakeConfig.Slot0
                .withKP(0)
                .withKD(0)
                .withKI(0);
            kIntakeConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IntakeConstants.intakeCurrentLimit);
            kIntakeConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
            kIntakeConfig.Feedback
                .withSensorToMechanismRatio(IntakeConstants.kGearRatio / IntakeConstants.kGearCircumference);
        }
    }

    public static final class IndexerConfigs  {
        public static final TalonFXConfiguration kIndexConfig = new TalonFXConfiguration();


        static {
            kIndexConfig.Slot0
                .withKP(0)
                .withKD(0)
                .withKI(0);
            kIndexConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IndexerConstants.indexerCurrentLimit);
            kIndexConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
            kIndexConfig.Feedback
                .withSensorToMechanismRatio(1);
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
