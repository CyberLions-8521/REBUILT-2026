package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    // Constants used in CANdle construction
    final int kCANdleId = 0;
    final CANBus kCANdleCANbus = new CANBus("canivore");

     // Colors used in the animations
    final RGBWColor kGreen = new RGBWColor(0, 217, 0);
    final SolidColor color = new SolidColor(0, 10);

    // Construct the CANdle and control requests
    final CANdle candle = new CANdle(kCANdleId, kCANdleCANbus);

    public LED() {
        // Configure the CANdle for basic use
        CANdleConfiguration configs = new CANdleConfiguration();

        configs.LED.withStripType(StripTypeValue.GRB)
                    .withBrightnessScalar(0.5);

        // Disable status LED when being controlled
        configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        
        candle.getConfigurator().apply(configs);
        candle.setControl(color.withColor(kGreen));
 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}