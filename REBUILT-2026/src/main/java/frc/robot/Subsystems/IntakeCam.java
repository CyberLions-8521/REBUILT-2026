package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;

public class IntakeCam {

    private final PhotonCamera intakeCamera;
    
    private static final int FUEL_DETECT_PIPELINE = 0;

    public IntakeCam(){
        intakeCamera = new PhotonCamera("Vu_WebCam");
        intakeCamera.setPipelineIndex(FUEL_DETECT_PIPELINE);
    }

    public boolean hasTarget(){ 
        return intakeCamera.getLatestResult().hasTargets();
    }
}

    
    
    
    
    
    

