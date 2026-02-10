package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

public class Limelight extends SubsystemBase {

    private final String LimelightName = "Limelight-3.0";
    private final CANdle CANdle = new CANdle(4, "ryan");
    
    public Limelight() {
        CANdleConfiguration Configs = new CANdleConfiguration();
        Configs.LED.withBrightnessScalar(0.5).
                 withStripType(StripTypeValue.GRB);

        CANdle.getConfigurator().apply(Configs);
    }

    @Override
    public void periodic() {
        if(LimelightHelpers.getTV(LimelightName)) {
            CANdle.setControl(new SolidColor(0, 120).withColor(new RGBWColor(0, 153, 0)));
            SmartDashboard.putNumber("TX", LimelightHelpers.getTX(LimelightName));
            SmartDashboard.putNumber("TY", LimelightHelpers.getTY(LimelightName));
        }else {
            CANdle.setControl(new SolidColor(0, 120).withColor(new RGBWColor(0, 153, 0)));
            SmartDashboard.putNumber("TX", LimelightHelpers.getTX(LimelightName));
            SmartDashboard.putNumber("TY", LimelightHelpers.getTY(LimelightName));
        }
        SmartDashboard.putBoolean("TV", LimelightHelpers.getTV(LimelightName));
        // This method will be called once per scheduler run
    }
}
