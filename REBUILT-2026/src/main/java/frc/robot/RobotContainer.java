// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

import com.pathplanner.lib.auto.NamedCommands;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.button.CommandXboxController;

import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDLights.LEDMode;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.SwerveConstants;

public class RobotContainer {
  //#region
  CommandXboxController m_driveController = new CommandXboxController(0);
  CommandXboxController m_subsystemController = new CommandXboxController(1);
  SwerveDrivebase m_drivebase = SwerveDrivebase.getInstance();
  Shooter m_shooter = new Shooter();
  Intake m_intake = new Intake();
  Indexer m_indexer = new Indexer();
  LEDLights m_lights = new LEDLights(m_shooter, getAllianceHubLocation());
  
  private final Selectable<Command> m_autoSelectable = new Selectable<>();

  public static final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);

  // Auto align objects
  // Both hub locations are relative to the blue alliance
  public static final Translation2d blueHubLocation = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
  public static final Translation2d redHubLocation = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));

  //Backup starting pose objects
  public static final Pose2d leftStartingPose = new Pose2d(3.5, 7.4, new Rotation2d());
  public static final Pose2d middleStartingPose = new Pose2d(3.5, 4.05, new Rotation2d());
  public static final Pose2d rightStartingPose = new Pose2d(3.5, 0.65, new Rotation2d());
  //#endregion

  public RobotContainer() {
    NamedCommands.registerCommand("WarmUpShooter", m_shooter.WarmUpShooter(60));
    NamedCommands.registerCommand("IntakePivotOut", m_intake.setPivotPositionCommand(IntakeConstants.extendedEncoderPosition).withTimeout(1));
    NamedCommands.registerCommand("IntakeForDuration", m_intake.getIntakeCommand(0.6).withTimeout(4));
    NamedCommands.registerCommand("ShootForDuration", 
      Commands.deadline(
        new SequentialCommandGroup(
          Commands.waitUntil(() ->
            m_shooter.isShooterAtSpeed(
              m_shooter.getDynamicRPS(
                m_drivebase.getPoseSupplier(),
                getAllianceHubLocation()
              )
            )
          ).withTimeout(5),
          m_indexer.runIndexerCommand(0.5).withTimeout(10)
        ),
        m_shooter.ShootWithoutAprilTagCommand(
          m_shooter.getDynamicRPS(
            m_drivebase.getPoseSupplier(),
            getAllianceHubLocation()
          )
        ),
        Commands.repeatingSequence(
          m_intake.setPivotPositionCommand(IntakeConstants.middleEncoderPosition),
          Commands.waitSeconds(1.5)
        )
      )
    );

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {

    m_drivebase.setDefaultCommand(this.getDriveCommand(
      1,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      () -> true
    ));
    m_intake.setDefaultCommand(m_intake.getIntakeCommand(0));
    m_indexer.setDefaultCommand(m_indexer.stopIndexerCommand());
    m_shooter.setDefaultCommand(m_shooter.stopBothFlywheelCommand());
    m_lights.setDefaultCommand(m_lights.setLEDCommand(LEDMode.Off));

    //======================== Drive controller ============================================== 
    // #region

    // brake drive - left bumper
    m_driveController.leftBumper().whileTrue(this.getDriveCommand(
      0.5, 
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter), 
      getJoystickValues(m_driveController::getRightX, omega_limiter), 
      () -> true));

    // auto-align commands
    m_driveController.leftTrigger().whileTrue( // auto-align only
      Commands.parallel(
        m_drivebase.odometryAutoAlign(
          getAllianceHubLocation(),
          getJoystickValues(m_driveController::getLeftY, vx_limiter), 
          getJoystickValues(m_driveController::getLeftX, vy_limiter)
        ),
        Commands.run(() -> {
          if (m_drivebase.isAutoAligned()) m_lights.setLEDMode(LEDMode.AlignedToTarget);
          else m_lights.setLEDMode(LEDMode.SeesAprilTag); // using SeesAprilTag as false condition for auto-align LOL cuz its red
        }, m_lights)
      )
    );

    m_driveController.rightTrigger().whileTrue( // [EXPERIMENTAL] auto-align w/ dynamic shooting and free movement 
      Commands.parallel(
        m_drivebase.odometryAutoAlign(
          getAllianceHubLocation(), 
          getJoystickValues(m_driveController::getLeftY, vx_limiter),
          getJoystickValues(m_driveController::getLeftX, vy_limiter),
          true
        ),
        Commands.sequence(
          Commands.deadline(
            Commands.waitUntil(() -> m_drivebase.isAutoAligned()),
            m_shooter.WarmUpShooter(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation()))
          ),
          m_shooter.ShootWithoutAprilTagCommand(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation()))
        ),
        Commands.sequence(
          Commands.waitUntil(() ->
            m_shooter.isShooterAtSpeed(m_shooter.getDynamicRPS(m_drivebase.getPoseSupplier(), getAllianceHubLocation())) 
            && m_drivebase.isAutoAligned()
          ),
          m_indexer.runIndexerCommand(0.4)
        ),
        Commands.run(() -> {
          Pose2d position = m_drivebase.getPose();
          Translation2d fieldLocation = position.getTranslation();
          double distance = getAllianceHubLocation().get().getDistance(fieldLocation);
          if (m_drivebase.isAutoAligned() && m_shooter.isShooterInRange(distance)) m_lights.setLEDMode(LEDMode.AlignedToTarget);
          else m_lights.setLEDMode(LEDMode.SeesAprilTag); // using SeesAprilTag as false condition for auto-align LOL cuz its red
        }, m_lights)
      )
    );

    //#endregion

    //======================== Subsystems controller ==============================================
    //#region

    // shoot manually 
    m_subsystemController.a().whileTrue(m_shooter.ShootWithoutAprilTagCommand(43)); // close 
    m_subsystemController.x().whileTrue(m_shooter.ShootWithoutAprilTagCommand(45)); // middle
    m_subsystemController.y().whileTrue(m_shooter.ShootWithoutAprilTagCommand(50)); // far
    m_subsystemController.rightTrigger().whileTrue(m_shooter.ShootWithoutAprilTagCommand(60)); // pass

    // indexer
    m_subsystemController.rightBumper().whileTrue(m_indexer.runIndexerCommand(0.7));
    m_subsystemController.leftBumper().whileTrue(m_indexer.runIndexerCommand(-0.4));

    // intake pivot
    m_subsystemController.dpadUp().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.retractedEncoderPosition).withTimeout(1));
    m_subsystemController.dpadDown().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.extendedEncoderPosition).withTimeout(1));
    m_subsystemController.dpadLeft().whileTrue(m_intake.setPivotPositionCommand(IntakeConstants.middleEncoderPosition));

    // intake
    m_subsystemController.b().whileTrue(
      Commands.parallel(
        m_intake.getIntakeCommand(1),
        m_lights.setLEDCommand(LEDMode.Intaking)
      )
    );
    m_subsystemController.dpadRight().whileTrue(
      Commands.parallel(
        m_intake.getIntakeCommand(-0.7),
        m_lights.setLEDCommand(LEDMode.Intaking)
      )
    );

    //#endregion

  }

  private Command getDriveCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
      () -> m_drivebase.drive(
        -vx.get() * multiplier * SwerveConstants.kMaxMetersPerSecond, // no negative cuz it flips joystick input
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

  // Selectable Autos

  public void configureAutos() {  
    // for autos that do nothing and only reset odometry use resetPoseFromAuto cuz initializeStartingPose() does not work during comp.

    if (m_drivebase.isPathPlannerAvailable()) {
      m_autoSelectable.add("LEFT Collect Neutral Zone", m_drivebase.getAutonomousCommand("LEFT Collect Neutral Zone"));
      m_autoSelectable.add("LEFT Do Nothing", m_drivebase.resetPoseFromAuto("LEFT Shoot Preloaded"));
      m_autoSelectable.add("LEFT Shoot Depot", m_drivebase.getAutonomousCommand("LEFT Shoot Depot"));
      m_autoSelectable.add("LEFT Shoot Preloaded", m_drivebase.getAutonomousCommand("LEFT Shoot Preloaded"));
      m_autoSelectable.addDefault("MIDDLE Do Nothing", m_drivebase.resetPoseFromAuto("MIDDLE Shoot Preloaded"));
      m_autoSelectable.add("MIDDLE Shoot Depot", m_drivebase.getAutonomousCommand("MIDDLE Shoot Depot"));
      m_autoSelectable.add("MIDDLE Shoot Outpost", m_drivebase.getAutonomousCommand("MIDDLE Shoot Outpost"));
      m_autoSelectable.add("MIDDLE Shoot Preloaded", m_drivebase.getAutonomousCommand("MIDDLE Shoot Preloaded"));
      m_autoSelectable.add("RIGHT Collect Neutral Zone", m_drivebase.getAutonomousCommand("RIGHT Collect Neutral Zone"));
      m_autoSelectable.add("RIGHT Do Nothing", m_drivebase.resetPoseFromAuto("RIGHT Shoot Preloaded"));
      m_autoSelectable.add("RIGHT Shoot Outpost", m_drivebase.getAutonomousCommand("RIGHT Shoot Outpost"));
      m_autoSelectable.add("RIGHT Shoot Preloaded", m_drivebase.getAutonomousCommand("RIGHT Shoot Preloaded"));
    } else {
      m_autoSelectable.add("LEFT Do Nothing", m_drivebase.resetPoseFromAuto(leftStartingPose));
      m_autoSelectable.addDefault("MIDDLE Do Nothing", m_drivebase.resetPoseFromAuto(middleStartingPose));
      m_autoSelectable.add("RIGHT Do Nothing", m_drivebase.resetPoseFromAuto(rightStartingPose));
    }

    Tunables.publish("Auto Selectable", m_autoSelectable);
  }

  public Command getAutonomousCommand() {
    return m_autoSelectable.getSelected();
  }
  
}
