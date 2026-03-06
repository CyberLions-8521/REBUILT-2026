// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {

  CommandXboxController m_controller = new CommandXboxController(0);

  Shooter m_shooter = new Shooter(ShooterConstants.kShooterTopLeftID, 
                                  ShooterConstants.kShooterTopRightID, 
                                  ShooterConstants.kShooterBottomRightID, 
                                  ShooterConstants.kHoodID);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_controller.y().whileTrue(
      m_shooter.hoodOnlySmartDashboard()
    );
    m_controller.x().whileTrue(
      m_shooter.runHood(0.05)
    );
    m_controller.a().whileTrue(
      m_shooter.runHood(-0.05)
    );
    m_controller.b().onTrue(
      m_shooter.zeroHood()
    );
    m_shooter.setDefaultCommand(
      m_shooter.runHood(0.0)
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
