// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.Constants.SwerveConstants;

public class RobotContainer {
  CommandXboxController m_driveController = new CommandXboxController(0);
  CommandXboxController m_subsystemController = new CommandXboxController(1);
  SwerveDrivebase m_drivebase = SwerveDrivebase.getInstance();
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
  public static final Translation2d blueHubLocation = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
  public static final Translation2d redHubLocation = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));

  public RobotContainer() {
    NamedCommands.registerCommand("WarmUpShooter", m_shooter.WarmUpShooter(60));
    NamedCommands.registerCommand("IntakePivotOut", m_intake.setPivotPositionCommand(IntakeConstants.extendedEncoderPosition).withTimeout(1));
    NamedCommands.registerCommand("IntakeForDuration", m_intake.getIntakeCommand(0.6).withTimeout(4));
    NamedCommands.registerCommand("ShootForDuration", 
      Commands.deadline(
        new SequentialCommandGroup(
          Commands.waitUntil(() -> m_shooter.isShooterAtSpeed(m_shooter.getDynamicRPS(m_drivebase.getPose(), getAllianceHubLocation()))).withTimeout(5),
          m_indexer.runIndexerCommand(0.4).withTimeout(5)
        ),
        m_shooter.ShootWithoutAprilTagCommand(m_shooter.getDynamicRPS(m_drivebase.getPose(), getAllianceHubLocation()))
      )
    );

    LimelightHelpers.setCameraPose_RobotSpace(
      LimelightConstants.limelightName, 
      LimelightConstants.kLimelightForwardOffset,  // Forward (m)
      LimelightConstants.kLimelightSideOffset,  // Side (m)
      LimelightConstants.kLimelightUpOffset,  // Up (m)
      LimelightConstants.kLimelightRoll,  // Roll (deg)
      LimelightConstants.kLimelightPitch,  // Pitch (deg)
      LimelightConstants.kLimelightYaw   // Yaw (deg)
    );

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {

    //======================== Drive controller ==============================================

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
      new SequentialCommandGroup(
        Commands.deadline(
          new SequentialCommandGroup(
            m_drivebase.odometryAutoAlign(
              getAllianceHubLocation(), 
              getJoystickValues(m_driveController::getLeftY, vx_limiter), // joystick input does nothing lol
              getJoystickValues(m_driveController::getLeftX, vy_limiter), // cause stop makes it ignored
              true
            ),
            m_drivebase.odometryAutoDistance(getAllianceHubLocation())
          ),
          m_shooter.WarmUpShooter(60) // warm up the shooter ahead of time
        ),
        Commands.parallel(
          m_shooter.ShootWithoutAprilTagCommand(m_shooter.getDynamicRPS(m_drivebase.getPose(), getAllianceHubLocation())),
          new SequentialCommandGroup(
            Commands.waitUntil(() -> m_shooter.isShooterAtSpeed(m_shooter.getDynamicRPS(m_drivebase.getPose(), getAllianceHubLocation()))).withTimeout(5),
            m_indexer.runIndexerCommand(0.4)
          )
        )
      )
    );

    // auto-align only - b
    m_driveController.b().whileTrue(
      m_drivebase.odometryAutoAlign(
        getAllianceHubLocation(),
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

  private Translation2d getAllianceHubLocation() {
    return (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) ? blueHubLocation : redHubLocation;
  }

  // Sendable Chooser Autos

  public void configureAutos() {  
    m_autoChooser.addOption("LEFT Do Nothing", m_drivebase.resetPoseFromAuto("LEFT Shoot Preloaded"));
    m_autoChooser.addOption("LEFT Collect Neutral Zone", m_drivebase.getAutonomousCommand("LEFT Collect Neutral Zone"));
    m_autoChooser.addOption("LEFT Shoot Preloaded", m_drivebase.getAutonomousCommand("LEFT Shoot Preloaded"));
    m_autoChooser.setDefaultOption("MIDDLE Do Nothing", m_drivebase.resetPoseFromAuto("MIDDLE Shoot Preloaded"));
    m_autoChooser.addOption("MIDDLE Shoot Preloaded", m_drivebase.getAutonomousCommand("MIDDLE Shoot Preloaded"));
    m_autoChooser.addOption("RIGHT Collect Neutral Zone", m_drivebase.getAutonomousCommand("RIGHT Collect Neutral Zone"));
    m_autoChooser.addOption("RIGHT Do Nothing", m_drivebase.resetPoseFromAuto("RIGHT Shoot Preloaded"));
    m_autoChooser.addOption("RIGHT Shoot Outpost", m_drivebase.getAutonomousCommand("RIGHT Shoot Outpost"));
    m_autoChooser.addOption("RIGHT Shoot Preloaded", m_drivebase.getAutonomousCommand("RIGHT Shoot Preloaded"));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
  
}
