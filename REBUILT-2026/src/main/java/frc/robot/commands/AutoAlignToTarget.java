// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivebase;

public class AutoAlignToTarget extends Command {
  private Translation2d m_targetPoint;
  private SwerveDrivebase m_drivebase;
  private Supplier<Double> i_vxInput;
  private Supplier<Double> i_vyInput;
  private PIDController m_pid;
  private SlewRateLimiter m_limiter;
  private SwerveDrivePoseEstimator m_estimator;
  private double i_maxAngularRadianRate;

  public AutoAlignToTarget (
    Translation2d m_targetPoint, 
    SwerveDrivebase m_drivebase, 
    Supplier<Double> i_vxInput,
    Supplier<Double> i_vyInput,
    PIDController m_pid, 
    SlewRateLimiter m_limiter, 
    SwerveDrivePoseEstimator m_estimator, 
    double i_maxAngularRadianRate
  ) {
    this.m_targetPoint = m_targetPoint;
    this.m_drivebase = m_drivebase;
    this.i_vxInput = i_vxInput;
    this.i_vyInput = i_vyInput;
    this.m_pid = m_pid;
    this.m_limiter = m_limiter;
    this.m_estimator = m_estimator;
    this.i_maxAngularRadianRate = i_maxAngularRadianRate;
    addRequirements(m_drivebase);
    this.m_pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize () {}

  @Override
  public void execute () {
    Pose2d estimatedPosition = m_estimator.getEstimatedPosition();

    // First part - calculate dynamic heading
    Translation2d robotLocation = estimatedPosition.getTranslation();
    Translation2d vector = m_targetPoint.minus(robotLocation);
    Rotation2d dynamicHeading = vector.getAngle();

    // Second part - convert dynamic heading to be suitable with drive method
    Rotation2d currentHeading = estimatedPosition.getRotation();
    double pidOutput = m_pid.calculate(currentHeading.getRadians(), dynamicHeading.getRadians());
    double turnOutput = pidOutput / i_maxAngularRadianRate;
    turnOutput = m_limiter.calculate(turnOutput);
    turnOutput = MathUtil.clamp(turnOutput, -1, 1); // just in case

    // Third part - check whether auto-align should run
    double distance = m_targetPoint.getDistance(robotLocation); // in meters
    if (distance > 0.5) { // 0.5 could totally be a constant
      m_drivebase.drive(i_vxInput.get(), i_vyInput.get(), turnOutput, true);
    } else {
      m_drivebase.drive(i_vxInput.get(), i_vyInput.get(), 0, true);
    }
  }

  @Override
  public void end (boolean interrupted) {}

  @Override
  public boolean isFinished () {
    return false;
  }
}