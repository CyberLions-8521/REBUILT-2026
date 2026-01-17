package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    
    public Limelight() {

        if(LimelightHelpers.getTV("")) {
            SmartDashboard.putNumber("TX", LimelightHelpers.getTX(""));
        }


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
