package frc.robot.Subsystems;

import frc.robot.Subsystems.IntakeVision;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;

public class LEDSubsystem {
    
    CANdle ledController = new CANdle(12); // CAN ID 1 for the LED controller
    CANdleConfiguration ledConfig = new CANdleConfiguration();
    

    public LEDSubsystem() {

    }

}
