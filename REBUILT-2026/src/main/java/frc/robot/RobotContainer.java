// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.SwerveDrivebase;

public class RobotContainer {


  CommandXboxController m_controller = new CommandXboxController(0);
  SwerveDrivebase m_drivebase = new SwerveDrivebase();
  private final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  private final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  private final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);

  // AutoAlign align = new AutoAlign(m_drivebase, 0, 1.5);
  // LEDLights m_LEDLights = new LEDLights();
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // m_drivebase.setDefaultCommand(this.getDriveCommand(
    //   1,
    //   getJoystickValues(m_controller::getLeftY, vx_limiter),
    //   getJoystickValues(m_controller::getLeftX, vy_limiter),
    //   getJoystickValues(m_controller::getRightX, omega_limiter),
    //   m_controller.getHID()::getRightBumperButton));
  }

   private Command getDriveCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
      () -> m_drivebase.drive(
        -vx.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -vy.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -omega.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        !fieldRelative.get()),
      m_drivebase);    
  }

  private Supplier<Double> getJoystickValues(Supplier<Double> controller, SlewRateLimiter limiter) {
    return () -> {
      double deadBandValue = MathUtil.applyDeadband(controller.get(), 0.2);
      double squaredValue = Math.copySign(deadBandValue * deadBandValue, deadBandValue);
      return limiter.calculate(squaredValue);
    };
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
