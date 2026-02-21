import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    
public static final class VisionConstants {
    public static final double maximumAmbiguity = 0.2; // adjust after testing

    public static double intakeCameraXOffset = 12.0; // x offset from center to intake camera (inches); change when positioned
    public static double intakeCameraYOffset = 10.0; // y offset from center to intake camera (inches); change when positioned
    public static double intakeCameraZOffset = 12.0; // z offset from ground to camera (inches); change when positioned
    
    public static double intakeCameraRoll = 0.0; // roll of intake camera in degrees; change when positioned
    public static double intakeCameraPitch = 0.0; // pitch of intake camera in degrees; change when positioned
    public static double intakeCameraYaw = 0.0; // yaw of intake camera in degrees; change when positioned

    public static final Transform3d INTAKE_CAMERA_TRANSFORM = new Transform3d(
        new Translation3d(Units.inchesToMeters(intakeCameraXOffset), Units.inchesToMeters(intakeCameraYOffset), Units.inchesToMeters(intakeCameraZOffset)),
        new Rotation3d(Units.degreesToRadians(intakeCameraRoll), Units.degreesToRadians(intakeCameraPitch), Units.degreesToRadians(intakeCameraYaw))
    ); // transform 3d position for camera given in meters; remove conversion if using inches 
}

public static final class CameraConstants{
    public static final String kIntakeCamera = "Cris_IntakeCam";
    }
}
