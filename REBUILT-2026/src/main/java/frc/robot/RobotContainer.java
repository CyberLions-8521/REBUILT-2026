// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class RobotContainer {

  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final Intake m_intake = new Intake(IntakeConstants.kIntakeID, IntakeConstants.kPivotID);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_controller.a().whileTrue(m_intake.getIntakeCommand(0.02));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
