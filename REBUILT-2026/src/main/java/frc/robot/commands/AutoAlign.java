// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
    private final SwerveDrivebase m_drivebase;
    private final PIDController radialPID = new PIDController(0, 0, 0);
    private final PIDController angularPID = new PIDController(0, 0, 0);
    private final double m_desiredAngle;

    int[] validIDs = {3,4};

  public AutoAlign(SwerveDrivebase drivebase, double desiredAngle) {
    this.m_drivebase = drivebase;
    this.m_desiredAngle = desiredAngle;
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offsetx = LimelightHelpers.getTargetPose3d_CameraSpace("limelight3.0").getX();
    double offsety = LimelightHelpers.getTargetPose3d_CameraSpace("limelight3.0").getY();
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}