package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // CommandXboxController m_controller = new CommandXboxController(0);
  CommandPS4Controller m_controller = new CommandPS4Controller(0);

  Shooter m_shooter = new Shooter(ShooterConstants.kShooterTopLeftID, 
                                  ShooterConstants.kShooterTopRightID, 
                                  ShooterConstants.kShooterBottomRightID, 
                                  ShooterConstants.kHoodID);

  public RobotContainer() {

    LimelightHelpers.setCameraPose_RobotSpace("limelight",
        0.0,  // Forward (m)
        0.0,  // Side (m)
        0.0,  // Up (m)
        0.0,  // Roll (deg)
        15.0,  // Pitch (deg)
        0.0   // Yaw (deg)
    );

    configureBindings();
  }

  private void configureBindings() {
    m_controller.povUp().whileTrue(m_shooter.shoot());
    m_controller.povRight().whileTrue(m_shooter.runFlywheelDashboard());
    m_controller.povDown().whileTrue(m_shooter.runFlywheel(() -> 55));
    m_shooter.setDefaultCommand(m_shooter.stopFlywheel());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
}
