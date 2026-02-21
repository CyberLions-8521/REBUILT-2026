package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeVision extends SubsystemBase {

    private final PhotonCamera intakeCamera;
    
    private static final int FUEL_DETECT_PIPELINE = 0;

  
    public boolean hasTarget(){ 
        return intakeCamera.getLatestResult().hasTargets();
    }

    public double getTargetArea(){ 
        if (hasTarget()) {
            return intakeCamera.getLatestResult().getBestTarget().getArea();
        } else {
            return 0.0;
        }
    }
    
    public double getTargetYaw(){ 
        if (hasTarget()) {
            return intakeCamera.getLatestResult().getBestTarget().getYaw();
        } else {
            return 0.0;
        }
    }
}
