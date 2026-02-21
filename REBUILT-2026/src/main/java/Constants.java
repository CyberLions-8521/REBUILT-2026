import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    
public static final class VisionConstants {
    public static final double MIN_TARGET_AREA = 0.1; // minimum area of target to be considered valid; change after testing 
    public static final double TARGET_EXP_TIME = 1.0; //expire time for target when lost; change after testing

    public static double intakeCam_XOffset = 0.0; // x offset from center to intake camera (inches); change when positioned
    public static double intakeCam_YOffset = 0.0; // y offset from center to intake camera (inches); change when positioned
    public static double intakeCam_ZOffset = 0.0; // z offset from ground to camera (inches); change when positioned
    
    public static double intakeCam_Roll = 0.0; // roll of intake camera in degrees; change when positioned
    public static double intakeCam_Pitch = 0.0; // pitch of intake camera in degrees; change when positioned
    public static double intakeCam_Yaw = 0.0; // yaw of intake camera in degrees; change when positioned

    public static final Transform3d INTAKE_CAMERA_POSITION = new Transform3d(
        new Translation3d(Units.inchesToMeters(intakeCam_XOffset), Units.inchesToMeters(intakeCam_YOffset), Units.inchesToMeters(intakeCam_ZOffset)),
        new Rotation3d(Units.degreesToRadians(intakeCam_Roll), Units.degreesToRadians(intakeCam_Pitch), Units.degreesToRadians(intakeCam_Yaw))); 
        // transform 3d position for camera given in meters; remove conversion if using inches 

    public static final Transform3d CAMERA_TO_FRONT = INTAKE_CAMERA_POSITION.inverse(); 

}

public static final class CameraConstants{
    public static final String kIntakeCamera = "Cris_IntakeCam";
    }

public static final class TargetProfiles {
    public static final int kHomePipeline = 0; // for makeshift testing target @ home
    public static final int kCompetitionPipeline = 1; // for Fuel tartget @ competition


    // create and add HSV thresholds for target detection; will need to be determined through testing and may require multiple sets for different lighting conditions
}

}