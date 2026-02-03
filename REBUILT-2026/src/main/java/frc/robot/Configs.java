package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class Configs {

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
                .withSupplyCurrentLimit(80);
            kPivotConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
            kPivotConfig.Feedback
                .withSensorToMechanismRatio(IntakeConstants.kGearRatio / IntakeConstants.kGearCircumference);
        }

         static {
            kIntakeConfig.Slot0
                .withKP(0)
                .withKD(0)
                .withKI(0);
            kIntakeConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80);
            kIntakeConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
            kIntakeConfig.Feedback
                .withSensorToMechanismRatio(IntakeConstants.kGearRatio / IntakeConstants.kGearCircumference);
        }

        

    }
}
