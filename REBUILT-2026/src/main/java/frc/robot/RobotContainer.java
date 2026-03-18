// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LimitSwitchConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.LEDLights.LEDMode;

public class RobotContainer {

  // SwerveDrivebase m_drivebase = new SwerveDrivebase();
  CommandXboxController m_controller = new CommandXboxController(0);
  LEDLights m_LEDLights = new LEDLights();
  SwerveDrivebase m_drivebase = new SwerveDrivebase();

  public static final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  //public final Trigger seesTagLeftBumperNotPressed = new Trigger(() -> LimelightHelpers.getTV(LimelightConstants.limelightName) && !m_controller.leftBumper().getAsBoolean());
  //public final Trigger seesTagLeftBumperPressed = m_controller.leftBumper().and(() -> LimelightHelpers.getTV(LimelightConstants.limelightName)); //placeholder for the real condtition check (will do later)


  public RobotContainer() {
    m_LEDLights.setDefaultCommand(m_LEDLights.limitSwitchLEDCommand());
    configureBindings();
  }

  private void configureBindings() {
    m_drivebase.setDefaultCommand(this.getDriveCommand(
      1,
      getJoystickValues(m_controller::getLeftY, vx_limiter),
      getJoystickValues(m_controller::getLeftX, vy_limiter),
      getJoystickValues(m_controller::getRightX, omega_limiter),
      () -> true));
    m_controller.leftBumper().and(() -> LimelightHelpers.getTV(LimelightConstants.limelightName)).whileTrue(this.getDriveAutoAlignCommand(
      0.2,
      getJoystickValues(m_controller::getLeftY, vx_limiter),
      getJoystickValues(m_controller::getLeftX, vy_limiter),
      m_drivebase.getTXAdujstmentRotation(omega_limiter, 0),
      () -> false));

    // m_LEDLights.setDefaultCommand(m_LEDLights.setLEDCommand(LEDMode.Idle));
    // seesTagLeftBumperNotPressed.whileTrue(m_LEDLights.setLEDCommand(LEDMode.SeesApriltag));
    // seesTagLeftBumperPressed.whileTrue(m_LEDLights.setLEDCommand(LEDMode.TargetingApriltag));
    // // m_controller.x().whileTrue(m_LEDLights.setLEDCommandTimed(LEDMode.Charging, LEDMode.Shooting, 2));
    // shootFail.onTrue(m_LEDLights.setLEDCommandTimed(LEDMode.ShootFail, 1));

}

   private Command getDriveCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
      () -> m_drivebase.drive(
        -vx.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -vy.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -omega.get() * multiplier * SwerveConstants.kMaxAngularSpeed,
        fieldRelative.get()),
      m_drivebase);    
  }

  private Command getDriveAutoAlignCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
      () -> m_drivebase.drive(
        -vx.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -vy.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -omega.get(),
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
    return Commands.print("No autonomous command configured");
  }
}
