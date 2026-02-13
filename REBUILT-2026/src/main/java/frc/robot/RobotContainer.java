// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrivebase;

import frc.robot.commands.AutoAlignCommand;

public class RobotContainer {
  private final SwerveDrivebase m_db = new SwerveDrivebase();
  private final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  private final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  private final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  
  private final CommandXboxController m_driveController = new CommandXboxController(OperatorConstants.kDriveControllerPort);

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {
    //Swerve Controller (XBOX)
    // regular drive w/ slew rate
    m_db.setDefaultCommand(this.getDriveCommand(
      1, 
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      m_driveController.getHID()::getRightBumperButton
    ));

    // brake driving - left trigger
    m_driveController.leftTrigger().whileTrue(this.getDriveCommand(
      0.5,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      m_driveController.getHID()::getRightBumperButton));

    // auto align
    m_driveController.a().whileTrue(new AutoAlignCommand(m_db));
    /* 
    m_driveController.a().whileTrue(this.getDriveCommand(
      m_driveController::getTargetingForwardSpeed, 
      0, 
      m_driveController::getTargetingAngularVelocity, 
      false
    ));
    */
  }

  private Command getDriveCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
      () -> m_db.drive(
        -vx.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -vy.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -omega.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        !fieldRelative.get()),
      m_db);    
  }

  private Supplier<Double> getJoystickValues(Supplier<Double> controller, SlewRateLimiter limiter){
    return () -> {
      double deadBandValue = MathUtil.applyDeadband(controller.get(), ControllerConstants.kDeadband);
      double squaredValue = Math.copySign(deadBandValue * deadBandValue, deadBandValue);
      return limiter.calculate(squaredValue);
    };
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
