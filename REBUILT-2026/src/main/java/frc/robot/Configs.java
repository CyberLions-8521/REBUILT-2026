package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IndexerConstants;;

/** Add your docs here. */
public class Configs {

    public static final class IntakeConfigs  {
        public static final TalonFXConfiguration kIndexConfig = new TalonFXConfiguration();


        static {
            kIndexConfig.Slot0
                .withKP(0)
                .withKD(0)
                .withKI(0);
            kIndexConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80);
            kIndexConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
            kIndexConfig.Feedback
                .withSensorToMechanismRatio(IndexerConstants.kGearRatio / IndexerConstants.kGearCircumference);
        }



    }
}