// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
    private final SwerveDrivebase drivebase;
    private final PIDController xPID = new PIDController(0, 0, 0);
    private final PIDController zPID = new PIDController(0,0,0);
    private final PIDController angularPID = new PIDController(0, 0, 0);
    private final double desiredAngle;
    private final double desiredRadius;

    int[] validIDs = {3,4};

  public AutoAlign(SwerveDrivebase drivebase, double desiredAngle, double desiredRadius) {
    this.drivebase = drivebase;
    this.desiredAngle = desiredAngle;
    this.desiredRadius = desiredRadius;
    xPID.setTolerance(0.1);
    zPID.setTolerance(0.1);
    angularPID.setTolerance(0.1);
    angularPID.enableContinuousInput(Math.PI, Math.PI);
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightConstants.limelightName, validIDs);
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (LimelightHelpers.getTV(LimelightConstants.limelightName)) {
    double offsetX = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName).getX(); //make sure to configure the limelight in robot space!!
    double offsetZ = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName).getZ(); //.getTargetPose3d_RobotSpace returns the coordinates of the apriltag relative to the robot

    double xChange = MathUtil.clamp(xPID.calculate(offsetX, desiredRadius * Math.cos(desiredAngle)), -0.2 * SwerveConstants.kMaxMetersPerSecond, 0.2 * SwerveConstants.kMaxMetersPerSecond);
    double zChange = MathUtil.clamp(zPID.calculate(offsetZ, desiredRadius * Math.sin(desiredAngle)), -0.2 * SwerveConstants.kMaxMetersPerSecond, 0.2 * SwerveConstants.kMaxMetersPerSecond);
    double angularChange = MathUtil.clamp(angularPID.calculate(drivebase.getHeading().getRadians(), desiredAngle), -0.2 * SwerveConstants.kMaxAngularSpeed, 0.2 * SwerveConstants.kMaxAngularSpeed);

    drivebase.drive(xChange, zChange, angularChange, false);
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xPID.atSetpoint() && zPID.atSetpoint() && angularPID.atSetpoint());
  }
}
