package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;

public class RobotContainer {

  CommandXboxController m_controller = new CommandXboxController(0);

  Shooter m_shooter = new Shooter(ShooterConstants.kShooterTopLeftID, 
                                  ShooterConstants.kShooterTopRightID, 
                                  ShooterConstants.kShooterBottomRightID, 
                                  ShooterConstants.kHoodID);

  public RobotContainer() {
    // from a hub's center apriltag to the opening of the hub
    LimelightHelpers.setFiducial3DOffset("limelight", 
        0.5969,    // Forward offset
        0.0,       // Side offset  
        0.70485    // Height offset
    );
    LimelightHelpers.setCameraPose_RobotSpace("limelight",
        0.0,  // Forward (m)
        0.0,  // Side (m)
        0.0,  // Up (m)
        0.0,  // Roll (deg)
        0.0,  // Pitch (deg)
        0.0   // Yaw (deg)
    );
    configureBindings();
  }

  private void configureBindings() {
    m_controller.y().whileTrue(
      Commands.run(() -> {
        if (!LimelightHelpers.getTV("limelight")) return;
        Pose3d targetPoseRobot = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
        SmartDashboard.putNumber("Target X (m)", targetPoseRobot.getX());
        SmartDashboard.putNumber("Target Y (m)", targetPoseRobot.getY());
        SmartDashboard.putNumber("Target Z (m)", targetPoseRobot.getZ());
        double range = Math.sqrt(
          Math.pow(targetPoseRobot.getX(), 2) + 
          Math.pow(targetPoseRobot.getY(), 2)
        );
        double angleDegs = m_shooter.getAngle(ShooterConstants.kDefaultShooterSpeed, range);
        if (Double.isNaN(angleDegs) || angleDegs < 40 || angleDegs > 70) return;
        m_shooter.runHoodMotor(angleDegs - ShooterConstants.kHoodLowDegFromHorizontal);
      }, m_shooter)
    );
    m_controller.povUp().whileTrue(m_shooter.setHoodDeg(15));

    m_controller.povDown().whileTrue(m_shooter.runFlywheel(
      ShooterConstants.kDefaultShooterMotorSpeed
    ));

    m_controller.x().whileTrue(m_shooter.runHood(0.05));
    m_controller.a().whileTrue(m_shooter.runHood(-0.05));
    m_controller.b().onTrue(m_shooter.zeroHood());
    m_shooter.setDefaultCommand(m_shooter.runHood(0.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
