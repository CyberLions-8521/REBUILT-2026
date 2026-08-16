// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.SwerveConstants;

public class RobotContainer {
  CommandXboxController m_driveController = new CommandXboxController(0);
  CommandXboxController m_subsystemController = new CommandXboxController(1);
  SwerveDrivebase m_drivebase = new SwerveDrivebase();
  Shooter m_shooter = new Shooter();
  Intake m_intake = new Intake();
  Indexer m_indexer = new Indexer();
  LEDLights m_lights = new LEDLights(m_shooter);
  
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
    SmartDashboard.putData(m_autoChooser);
  }

  private void configureBindings() {

    //======================== Drive controller ==============================================
    alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    selectedHub = (alliance == DriverStation.Alliance.Blue) ? blueHubLocation : redHubLocation;

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

    // auto-align, auto distance, and shoot - x
    m_driveController.x().whileTrue(
      Commands.parallel(
        m_shooter.ShootWithoutAprilTagCommand(60), // this will need to be tuned irl later!!!!
        new SequentialCommandGroup(
          m_drivebase.odometryAutoAlign(
            selectedHub, 
            getJoystickValues(m_driveController::getLeftY, vx_limiter), 
            getJoystickValues(m_driveController::getLeftX, vy_limiter), 
            true
            ),
          m_drivebase.odometryAutoDistance(selectedHub),
          Commands.waitUntil(() -> m_shooter.isUpperAtSpeed(60)),
          m_indexer.runIndexerCommand(0.5)
        )
      )
    );

    // auto-align only - b
    m_driveController.b().whileTrue(
      m_drivebase.odometryAutoAlign(
        selectedHub,
        getJoystickValues(m_driveController::getLeftY, vx_limiter), 
        getJoystickValues(m_driveController::getLeftX, vy_limiter), 
        false
        )
    );

    //======================== Subsystems controller ==============================================

    m_intake.setDefaultCommand(m_intake.getIntakeCommand(0));
    m_indexer.setDefaultCommand(m_indexer.stopIndexerCommand());
    m_shooter.setDefaultCommand(m_shooter.stopBothFlywheelCommand());

    // shoot
    m_subsystemController.rightTrigger().whileTrue(m_shooter.ShootWithAprilTagCommand());
    m_subsystemController.y().whileTrue(m_shooter.ShootWithoutAprilTagCommand(60));
    m_subsystemController.b().whileTrue(m_shooter.ShootWithoutAprilTagCommand(55));
    m_subsystemController.a().whileTrue(m_shooter.ShootWithoutAprilTagCommand(45));

    // indexer
    m_subsystemController.rightBumper().whileTrue(m_indexer.runIndexerCommand(0.5));
    m_subsystemController.leftBumper().whileTrue(m_indexer.runIndexerCommand(-0.2));

    // intake pivot
    m_subsystemController.povUp().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.retractedEncoderPosition).withTimeout(1));
    m_subsystemController.povDown().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.extendedEncoderPosition).withTimeout(1));
    m_subsystemController.povLeft().whileTrue(m_intake.setPivotPositionCommand(IntakeConstants.middleEncoderPosition));

    // intake rollers
    m_subsystemController.leftTrigger().whileTrue(m_intake.getIntakeCommand(0.75));
    m_subsystemController.x().whileTrue(m_intake.getIntakeCommand(0.65));
    m_subsystemController.x().whileTrue(m_indexer.runIndexerCommand(0.4));

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
