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
       public static final TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();

       static {
            hoodConfigs.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
       }
    }

    public static final class IntakeConfigs  {
        public static final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

        static {
            pivotConfigs.Slot0
                .withKP(0)
                .withKD(0)
                .withKI(0);
            pivotConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IntakeConstants.pivotCurrentLimit);
            pivotConfigs.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
            pivotConfigs.Feedback
                .withSensorToMechanismRatio(IntakeConstants.kGearRatio / IntakeConstants.kGearCircumference);
        
            rollerConfigs.Slot0
                .withKP(0)
                .withKD(0)
                .withKI(0);
            rollerConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IntakeConstants.intakeCurrentLimit);
            rollerConfigs.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
            rollerConfigs.Feedback
                .withSensorToMechanismRatio(IntakeConstants.kGearRatio / IntakeConstants.kGearCircumference);
        }
    }

    public static final class IndexerConfigs  {
        public static final TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();


        static {
            rollerConfigs.Slot0
                .withKP(0)
                .withKD(0)
                .withKI(0);
            rollerConfigs.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IndexerConstants.kRollerCurrentLimit);
            rollerConfigs.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
            rollerConfigs.Feedback
                .withSensorToMechanismRatio(1);
        }
    }

    public final static class CANdleConfigs {
        public static final CANdleConfiguration CANdleConfigs = new CANdleConfiguration();

        static {
            CANdleConfigs.LED
                .withStripType(StripTypeValue.GRB)
                .withBrightnessScalar(0.3);
        }
    }

}
