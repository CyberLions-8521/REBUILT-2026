// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.utils.Constants.SwerveConstants;

public class RobotContainer {
  CommandXboxController m_driveController = new CommandXboxController(0);
  SwerveDrivebase m_drivebase = new SwerveDrivebase();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public static final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);

  // Auto align objects
  // Both hub locations are relative to the blue alliance
  // The alliance is defined in the constructor, default is blue origin as the odometry origin is that as well
  public static final Translation2d blueHubLocation = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
  public static final Translation2d redHubLocation = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));
  public DriverStation.Alliance alliance;
  public Translation2d selectedHub;

  public RobotContainer() {
      LimelightHelpers.setCameraPose_RobotSpace("limelight", // may need to be configured!!!! but idk
        0.0,  // Forward (m)
        0.0,  // Side (m)
        0.0,  // Up (m)
        0.0,  // Roll (deg)
        15.0,  // Pitch (deg)
        0.0   // Yaw (deg)
    );
    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    /*
     * (some of the original bindings from the REBUILT code)
     *
     * Drive controller bindings:
     * Brake drive - left trigger
     * Auto align - x
     *
     * Subsystem controller:
     * Move indexer - left & right bumper
     * Intake rollers - left trigger
     * Intake rollers + indexer - x
     */

    // default drive 
    m_drivebase.setDefaultCommand(this.getDriveCommand(
      1,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      () -> true));

    // brake drive - left trigger
    m_driveController.leftTrigger().whileTrue(this.getDriveCommand(
      0.5, 
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter), 
      getJoystickValues(m_driveController::getRightX, omega_limiter), 
      () -> true));

    // auto-align + auto distance to either hub depending on the alliance - x
    alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    selectedHub = (alliance == DriverStation.Alliance.Blue) ? blueHubLocation : redHubLocation;
    m_driveController.x().whileTrue(new SequentialCommandGroup(
      m_drivebase.odometryAutoAlign(
        selectedHub, 
        getJoystickValues(m_driveController::getLeftY, vx_limiter), 
        getJoystickValues(m_driveController::getLeftX, vy_limiter), 
        true
      ),
      m_drivebase.odometryAutoDistance(selectedHub)
    ));
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

  public Supplier<Double> getJoystickValues(Supplier<Double> controller, SlewRateLimiter limiter) {
    return () -> {
      double deadBandValue = MathUtil.applyDeadband(controller.get(), 0.2);
      double squaredValue = Math.copySign(deadBandValue * deadBandValue, deadBandValue);
      return limiter.calculate(squaredValue);
    };
  }

  // Sendable Chooser Autos

  public void configureAutos() {  
    m_autoChooser.setDefaultOption("New Auto", m_drivebase.getAutonomousCommand("New Auto"));
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
