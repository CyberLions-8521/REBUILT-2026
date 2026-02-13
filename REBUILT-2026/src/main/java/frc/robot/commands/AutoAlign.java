// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
    private final SwerveDrivebase drivebase;
    private final PIDController xPID = new PIDController(0,0,0);
    private final PIDController yPID = new PIDController(0,0,0);
    private final PIDController angularPID = new PIDController(0, 0, 0);
    private final double desiredAngle;
    private final double desiredRadius;

    int[] validIDs = {3,4};

  public AutoAlign(SwerveDrivebase drivebase, double desiredAngle, double desiredRadius) {
    this.drivebase = drivebase;
    this.desiredAngle = desiredAngle;
    this.desiredRadius = desiredRadius;
    xPID.setTolerance(0.1);
    yPID.setTolerance(0.1);
    angularPID.setTolerance(0.1);
    angularPID.enableContinuousInput(-Math.PI, Math.PI);
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightConstants.limelightName, validIDs);
    addRequirements(drivebase);
  }

  public void getCoordinates() {
    SmartDashboard.getNumber("Limelight X",LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName).getX());
    SmartDashboard.getNumber("Limelight Y",LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName).getY());
    SmartDashboard.getNumber("Limelight Z",LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName).getZ());
  }

  public void tunePID() {
    xPID.setP(SmartDashboard.getNumber("XP", 0));
    xPID.setI(SmartDashboard.getNumber("XI", 0));
    xPID.setD(SmartDashboard.getNumber("XD", 0));

    yPID.setP(SmartDashboard.getNumber("YP", 0));
    yPID.setI(SmartDashboard.getNumber("YI", 0));
    yPID.setD(SmartDashboard.getNumber("YD", 0));

    angularPID.setP(SmartDashboard.getNumber("omegaP", 0));
    angularPID.setI(SmartDashboard.getNumber("omegaI", 0));
    angularPID.setD(SmartDashboard.getNumber("omegaD", 0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(LimelightConstants.limelightName)) {
      // tunePID();
      getCoordinates();
      // double offsetX = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName).getX(); //make sure to configure the limelight in robot space!!
      // double offsetY = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName).getY(); //.getTargetPose3d_RobotSpace returns the coordinates of the apriltag relative to the robot

      // double xChange = MathUtil.clamp(xPID.calculate(offsetX, desiredRadius * Math.cos(desiredAngle)), -0.2 * SwerveConstants.kMaxMetersPerSecond, 0.2 * SwerveConstants.kMaxMetersPerSecond);
      // double yChange = MathUtil.clamp(yPID.calculate(offsetY, desiredRadius * Math.sin(desiredAngle)), -0.2 * SwerveConstants.kMaxMetersPerSecond, 0.2 * SwerveConstants.kMaxMetersPerSecond);
      // double angularChange = MathUtil.clamp(angularPID.calculate(drivebase.getHeading().getRadians(), desiredAngle), -0.2 * SwerveConstants.kMaxAngularSpeed, 0.2 * SwerveConstants.kMaxAngularSpeed);

      // drivebase.drive(xChange, yChange, angularChange, false);
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
    return (xPID.atSetpoint() && yPID.atSetpoint() && angularPID.atSetpoint());
  }
}
