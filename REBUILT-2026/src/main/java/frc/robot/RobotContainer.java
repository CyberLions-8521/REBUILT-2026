// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.button.CommandGamepad;
import frc.robot.subsystems.*;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.SwerveConstants;

public class RobotContainer {
  CommandGamepad m_driveController = new CommandGamepad(0);
  CommandGamepad m_subsystemController = new CommandGamepad(1);
  SwerveDrivebase m_drivebase = SwerveDrivebase.getInstance();
  Shooter m_shooter = new Shooter();
  Intake m_intake = new Intake();
  Indexer m_indexer = new Indexer();
  LEDLights m_lights = new LEDLights(m_shooter, getAllianceHubLocation());
  
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
          Commands.waitUntil(() -> 
            m_shooter.isShooterAtSpeed(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation()))
          ).withTimeout(5),
          m_indexer.runIndexerCommand(0.4).withTimeout(5)
        ),
        m_shooter.ShootWithoutAprilTagCommand(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation()))
      )
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

    // auto-align, auto distance, and shoot - x [EXPERIMENTAL]
    m_driveController.button(Gamepad.Button.WEST_FACE).whileTrue(
      new SequentialCommandGroup(
        Commands.deadline(
          new SequentialCommandGroup(
            m_drivebase.odometryAutoAlign(getAllianceHubLocation()),
            m_drivebase.odometryAutoDistance(getAllianceHubLocation(), true)
          ),
          m_shooter.WarmUpShooter(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation())) // warm up the upper rollers ahead of time
        ),
        Commands.parallel(
          m_shooter.ShootWithoutAprilTagCommand(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation())),
          new SequentialCommandGroup(
            Commands.waitUntil(() -> 
              m_shooter.isShooterAtSpeed(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation()))
            ).withTimeout(5),
            m_indexer.runIndexerCommand(0.4)
          )
        )
      )
    );

    // auto-align and dynamic shooting - a [EXPERIMENTAL]
    m_driveController.button(Gamepad.Button.SOUTH_FACE).whileTrue(
      Commands.parallel(
        m_drivebase.odometryAutoAlign(
          getAllianceHubLocation(), 
          getJoystickValues(m_driveController::getLeftY, vx_limiter),
          getJoystickValues(m_driveController::getLeftX, vy_limiter),
          true
        ),
        Commands.either(
          m_shooter.ShootWithoutAprilTagCommand(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation())), 
          m_shooter.WarmUpShooter(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation())), 
          () -> m_drivebase.isAutoAligned()
        ),
        Commands.either(
          m_indexer.runIndexerCommand(0.4), 
          m_indexer.stopIndexerCommand(), 
          () -> 
            m_shooter.isShooterAtSpeed(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation())) 
            && m_drivebase.isAutoAligned()
        )
      )
    );

    // auto-align only - b
    m_driveController.button(Gamepad.Button.EAST_FACE).whileTrue(
      m_drivebase.odometryAutoAlign(
        getAllianceHubLocation(),
        getJoystickValues(m_driveController::getLeftY, vx_limiter), 
        getJoystickValues(m_driveController::getLeftX, vy_limiter)
        )
    );

    //======================== Subsystems controller ==============================================

    m_intake.setDefaultCommand(m_intake.getIntakeCommand(0));
    m_indexer.setDefaultCommand(m_indexer.stopIndexerCommand());
    m_shooter.setDefaultCommand(m_shooter.stopBothFlywheelCommand());

    // shoot
    m_subsystemController.rightTrigger().whileTrue(m_shooter.ShootWithAprilTagCommand());
    m_subsystemController.button(Gamepad.Button.NORTH_FACE).whileTrue(m_shooter.ShootWithoutAprilTagCommand(60)); // y
    m_subsystemController.button(Gamepad.Button.EAST_FACE).whileTrue(m_shooter.ShootWithoutAprilTagCommand(55)); // b
    m_subsystemController.button(Gamepad.Button.SOUTH_FACE).whileTrue(m_shooter.ShootWithoutAprilTagCommand(45)); // a

    // indexer
    m_subsystemController.rightBumper().whileTrue(m_indexer.runIndexerCommand(0.5));
    m_subsystemController.leftBumper().whileTrue(m_indexer.runIndexerCommand(-0.2));

    // intake pivot
    m_subsystemController.povUp().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.retractedEncoderPosition).withTimeout(1));
    m_subsystemController.povDown().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.extendedEncoderPosition).withTimeout(1));
    m_subsystemController.povLeft().whileTrue(m_intake.setPivotPositionCommand(IntakeConstants.middleEncoderPosition));

    // intake rollers
    m_subsystemController.leftTrigger().whileTrue(m_intake.getIntakeCommand(0.75));
    m_subsystemController.button(Gamepad.Button.WEST_FACE).whileTrue(m_intake.getIntakeCommand(0.65)); // x
    m_subsystemController.button(Gamepad.Button.WEST_FACE).whileTrue(m_indexer.runIndexerCommand(0.4)); // x

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

  private Supplier<Translation2d> getAllianceHubLocation() {
    // suppplier so that hub can change after binding has been created 
    return () -> { 
      return (m_drivebase.shouldFlipPathForAlliance()) ? redHubLocation : blueHubLocation; 
    };
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
