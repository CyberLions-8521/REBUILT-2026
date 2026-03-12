// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.utils.Constants.SwerveConstants;

public class RobotContainer {
  CommandXboxController m_driveController = new CommandXboxController(0);
  CommandXboxController m_subsystemController = new CommandXboxController(1);
  SwerveDrivebase m_drivebase = new SwerveDrivebase();
  Intake m_intake = new Intake();
  Indexer m_indexer = new Indexer();

  public static final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivebase.setDefaultCommand(this.getDriveCommand(
      1,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      () -> true));
    // m_driveController.leftBumper().and(() -> LimelightHelpers.getTV(LimelightConstants.limelightName)).whileTrue(this.getDriveCommand(
    //   1,
    //   getJoystickValues(m_driveController::getLeftY, vx_limiter),
    //   getJoystickValues(m_driveController::getLeftX, vy_limiter),
    //   m_drivebase.getTXAdujstmentRotation(omega_limiter),
    //   () -> false));
    m_intake.setDefaultCommand(m_intake.getIntakeCommand(0));
    m_indexer.setDefaultCommand(m_indexer.stopIndexerCommand());
    m_subsystemController.a().onTrue(m_intake.setPivotIn().withTimeout(1));
    m_subsystemController.b().onTrue(m_intake.setPivotOut().withTimeout(1));
    m_subsystemController.rightTrigger().whileTrue(m_intake.getIntakeCommand(0.65));
    m_subsystemController.rightTrigger().whileTrue(m_indexer.runIndexerCommand(0.4));

  
    
  }

   private Command getDriveCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
      () -> m_drivebase.drive(
        -vx.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -vy.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -omega.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        fieldRelative.get()),
      m_drivebase);    
  }

  public Supplier<Double> getJoystickValues(Supplier<Double> controller, SlewRateLimiter limiter) {
    return () -> {
      double deadBandValue = MathUtil.applyDeadband(controller.get(), 0.2);
      double squaredValue = Math.copySign(deadBandValue * deadBandValue, deadBandValue);
      return limiter.calculate(squaredValue);
    };
  }

  public Command getAutonomousCommand() {
    return m_intake.getResetEncoderPosition();
  }
}
