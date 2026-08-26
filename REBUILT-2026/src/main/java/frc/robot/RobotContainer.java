// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.Constants.SwerveConstants;

public class RobotContainer {
  CommandXboxController m_driveController = new CommandXboxController(0);
  CommandXboxController m_subsystemController = new CommandXboxController(1);
  SwerveDrivebase m_drivebase = new SwerveDrivebase();
  Shooter m_shooter = new Shooter();
  Intake m_intake = new Intake();
  Indexer m_indexer = new Indexer();
  LEDLights m_lights = new LEDLights(m_shooter);

  public static final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);

  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  public RobotContainer() {
      LimelightHelpers.setCameraPose_RobotSpace("limelight",
        0.0,  // Forward (m)
        0.0,  // Side (m)
        0.0,  // Up (m)
        0.0,  // Roll (deg)
        15.0,  // Pitch (deg)
        0.0   // Yaw (deg)
    );
    configureBindings();
    configureAutos();
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {
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

    // auto align - x
    m_driveController.x().and(() -> LimelightHelpers.getTV(LimelightConstants.limelightName)).whileTrue(this.getDriveCommand(
      1,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      m_drivebase.getTXAdujstmentRotation(0),
      () -> false));

    m_driveController.a().onTrue(m_drivebase.resetGyroCommand());
    m_driveController.b().onTrue(m_drivebase.resetEncodersCommand());


   
    m_intake.setDefaultCommand(m_intake.getIntakeCommand(0));
    m_indexer.setDefaultCommand(m_indexer.stopIndexerCommand());  
    m_shooter.setDefaultCommand(m_shooter.stopBothFlywheelCommand());


     //SHOOT
    m_subsystemController.rightTrigger().whileTrue(m_shooter.ShootWithAprilTagCommand());

    m_subsystemController.y().whileTrue(m_shooter.ShootWithoutAprilTagCommand(60));
    m_subsystemController.b().whileTrue(m_shooter.ShootWithoutAprilTagCommand(55));
    m_subsystemController.a().whileTrue(m_shooter.ShootWithoutAprilTagCommand(45));

    //LEDS
  


    //INDEXER
    m_subsystemController.rightBumper().whileTrue(m_indexer.runIndexerCommand(0.5));
    m_subsystemController.leftBumper().whileTrue(m_indexer.runIndexerCommand(-0.2));


    //INTAKE PIVOT
    // m_subsystemController.povUp().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.retractedEncoderPosition).withTimeout(1));
    // m_subsystemController.povDown().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.extendedEncoderPosition).withTimeout(1));

    // m_subsystemController.povLeft().whileTrue(m_intake.setPivotPositionCommand(IntakeConstants.middleEncoderPosition));
    m_subsystemController.povUp().whileTrue(m_intake.blockPartyPivotCommand(0.2));
    
    //INTAKE ROLLERS
    // m_subsystemController.leftTrigger().whileTrue(m_intake.getIntakeCommand(0.75));

    // m_subsystemController.x().whileTrue(m_intake.getIntakeCommand(0.65));
    // m_subsystemController.x().whileTrue(m_indexer.runIndexerCommand(0.4));
    m_subsystemController.leftTrigger().whileTrue(m_intake.blockPartyIntakeCommand(0.75));



  
    
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

  // Sendable Chooser Autos

  public void configureAutos() {
    // Command driveBackCommand = m_drivebase.resetGyroCommand().andThen(new RunCommand(() -> m_drivebase.drive(-0.5, 0, 0, true)));
    // Command stopCommand = new InstantCommand(() -> m_drivebase.drive(0, 0, 0, true));
    // m_chooser.addOption("No Auto", null);
    // m_chooser.addOption("Preload Center", new SequentialCommandGroup(
    //     m_intake.getResetEncoderPosition()
    //     .andThen(driveBackCommand.withTimeout(3))
    //     .andThen(stopCommand)
    //     .andThen(m_intake.setPivotPositionCommand(IntakeConstants.extendedEncoderPosition))
    //     .andThen(m_shooter.shoot().alongWith(Commands.waitSeconds(2)
    //         .andThen(m_indexer.runIndexerCommand(0.6).alongWith(m_intake.getIntakeCommand(0.6)))).withTimeout(8))
    // ));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  // public Command getEthanAutoCommand(){
  //   //make an auto that 
  //   return 
  // }
}
