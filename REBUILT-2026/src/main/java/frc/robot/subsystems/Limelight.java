package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;

public class Limelight extends SubsystemBase {

    public CANdle LucasCANdle = new CANdle(4,(new CANBus("Ryan")));
    public String LucasLime = "Limelight3.0";

    //public/private ClassType className = ""

    public Limelight() {
        
        CANdleConfiguration Configs = new CANdleConfiguration();
        Configs.LED.withBrightnessScalar(0.5).withStripType(StripTypeValue.RGB); // Could be 2 separate Configs.LED...'s but is combined into one
        
        LucasCANdle.getConfigurator().apply(Configs);
        //derives from CANdle
    }

    @Override   
    public void periodic() {

        double tx = LimelightHelpers.getTX(LucasLime); // Gives x (horizontal) distance from the crosshair to the ATag [degrees]
        double ty = LimelightHelpers.getTY(LucasLime); // Gives y (vertical) distance from the crosshair to the ATag [degrees]
        double ta = LimelightHelpers.getTA(LucasLime); // How much of the image is seen: 0-100%
        boolean seeTarget = LimelightHelpers.getTV(LucasLime); // If it sees the ATag; not if it knows which ATag it is
        double txnc = LimelightHelpers.getTXNC(LucasLime); // Gives x (horizontal) distance from the middle of the ATag [degrees]
        double tync = LimelightHelpers.getTYNC(LucasLime); // Gives y (vertical) distance from the middle of the ATag [degrees]

        LimelightHelpers.setPipelineIndex(LucasLime, 0); // Pipelines change how the Lime sees the tags

        LimelightHelpers.setLEDMode_ForceOff(LucasLime); // Turns off the LED?
        LimelightHelpers.setLEDMode_ForceOn(LucasLime); // Turns on the LED?
        LimelightHelpers.setLEDMode_ForceBlink(LucasLime); // Makes Lime LED blink?
        LimelightHelpers.setLEDMode_PipelineControl(LucasLime); // Lets the LED be controlled by the current Pipeline

        System.out.println(seeTarget);
        
    }
}