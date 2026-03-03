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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.LEDLights.LEDMode;
import frc.robot.subsystems.SwerveDrivebase;

public class RobotContainer {

  // SwerveDrivebase m_drivebase = new SwerveDrivebase();
  CommandXboxController m_controller = new CommandXboxController(0);
  LEDLights m_LEDLights = new LEDLights();

  public static final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public final Trigger seesTagLeftBumperNotPressed = new Trigger(() -> LimelightHelpers.getTV(LimelightConstants.limelightName) && !m_controller.leftBumper().getAsBoolean());
  public final Trigger seesTagLeftBumperPressed = m_controller.leftBumper().and(() -> LimelightHelpers.getTV(LimelightConstants.limelightName));
  public final Trigger shootFail = m_controller.y(); //placeholder for the real condtition check (will do later)


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // m_drivebase.setDefaultCommand(this.getDriveCommand(
    //   1,
    //   getJoystickValues(m_controller::getLeftY, vx_limiter),
    //   getJoystickValues(m_controller::getLeftX, vy_limiter),
    //   getJoystickValues(m_controller::getRightX, omega_limiter),
    //   () -> true));
    // m_controller.leftBumper().and(() -> LimelightHelpers.getTV(LimelightConstants.limelightName)).whileTrue(this.getDriveCommand(
    //   1,
    //   getJoystickValues(m_controller::getLeftY, vx_limiter),
    //   getJoystickValues(m_controller::getLeftX, vy_limiter),
    //   m_drivebase.getTXAdujstmentRotation(omega_limiter),
    //   () -> false));f

    m_LEDLights.setDefaultCommand(m_LEDLights.setLEDCommand(LEDMode.Idle));
    seesTagLeftBumperNotPressed.whileTrue(m_LEDLights.setLEDCommand(LEDMode.SeesApriltag));
    seesTagLeftBumperPressed.whileTrue(m_LEDLights.setLEDCommand(LEDMode.TargetingApriltag));
    m_controller.x().whileTrue(m_LEDLights.setLEDCommand(LEDMode.Charging).withTimeout(2).andThen(m_LEDLights.setLEDCommand(LEDMode.Shooting)));
    shootFail.onTrue(m_LEDLights.setLEDCommand(LEDMode.ShootFail).withTimeout(1));

}

  //  private Command getDriveCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
  //   return new RunCommand(
  //     () -> m_drivebase.drive(
  //       -vx.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
  //       -vy.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
  //       -omega.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
  //       fieldRelative.get()),
  //     m_drivebase);    
  // }

  // public Supplier<Double> getJoystickValues(Supplier<Double> controller, SlewRateLimiter limiter) {
  //   return () -> {
  //     double deadBandValue = MathUtil.applyDeadband(controller.get(), 0.2);
  //     double squaredValue = Math.copySign(deadBandValue * deadBandValue, deadBandValue);
  //     return limiter.calculate(squaredValue);
  //   };
  // }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
