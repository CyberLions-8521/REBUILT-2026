// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final Indexer m_indexer = new Indexer(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_controller.rightTrigger().whileTrue(m_indexer.runIndexerCommand(0.02));
    m_controller.leftTrigger().whileTrue(m_indexer.runIndexerCommand(-0.02));
    // for when the controller is actually coded
    // m_gamepad.a().whileTrue(m_indexer.runIndexer(0.5));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
