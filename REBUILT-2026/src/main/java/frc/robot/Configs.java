package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Configs {

    public static final class OutputConfigs {
       public static final TalonFXConfiguration kKrakenLeaderConfig = new TalonFXConfiguration();

       static {
            kKrakenLeaderConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast);
       }
    }

    public static final class HoodConfigs {
       public static final TalonFXConfiguration kKrakenLeaderConfig = new TalonFXConfiguration();

       static {
            kKrakenLeaderConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
       }
    }




}
