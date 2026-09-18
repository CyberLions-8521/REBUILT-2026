// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.button.CommandGamepad;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.SwerveConstants;
import com.limelightvision.Limelight;

public class RobotContainer {
  CommandGamepad m_driveController = new CommandGamepad(0);
  CommandGamepad m_subsystemController = new CommandGamepad(1);
  Limelight m_limelight = new Limelight("limelight", 0.0,0.0,0.0,0.0,15.0,0.0); 
  Shooter m_shooter = new Shooter(m_limelight);
  Intake m_intake = new Intake();
  Indexer m_indexer = new Indexer();
  SwerveDrivebase m_drivebase = new SwerveDrivebase(m_limelight);
  LEDLights m_lights = new LEDLights(m_shooter, m_limelight, m_drivebase);

  public static final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);

  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  public RobotContainer() {
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
    m_driveController.button(3).and(() -> m_limelight.hasTarget()).whileTrue(this.getDriveCommand(
      1,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      m_drivebase.getTXAdujstmentRotation(0),
      () -> false));

    m_driveController.button(1).onTrue(m_drivebase.resetGyroCommand());
    m_driveController.button(2).onTrue(m_drivebase.resetEncodersCommand());


   
    m_intake.setDefaultCommand(m_intake.getIntakeCommand(0));
    m_indexer.setDefaultCommand(m_indexer.stopIndexerCommand());  
    m_shooter.setDefaultCommand(m_shooter.stopBothFlywheelCommand());


     //SHOOT
    m_subsystemController.rightTrigger().whileTrue(m_shooter.ShootWithAprilTagCommand());

    m_subsystemController.button(4).whileTrue(m_shooter.ShootWithoutAprilTagCommand(60));
    m_subsystemController.button(2).whileTrue(m_shooter.ShootWithoutAprilTagCommand(55));
    m_subsystemController.button(1).whileTrue(m_shooter.ShootWithoutAprilTagCommand(45));

    //LEDS
  


    //INDEXER
    m_subsystemController.rightBumper().whileTrue(m_indexer.runIndexerCommand(0.5));
    m_subsystemController.leftBumper().whileTrue(m_indexer.runIndexerCommand(-0.2));

    //INTAKE PIVOT
    m_subsystemController.povUp().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.retractedEncoderPosition).withTimeout(1));
    m_subsystemController.povDown().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.extendedEncoderPosition).withTimeout(1));

    m_subsystemController.povLeft().whileTrue(m_intake.setPivotPositionCommand(IntakeConstants.middleEncoderPosition));
    
    //INTAKE ROLLERS
    m_subsystemController.leftTrigger().whileTrue(m_intake.getIntakeCommand(0.75));

    m_subsystemController.button(3).whileTrue(m_intake.getIntakeCommand(0.65));
    m_subsystemController.button(3).whileTrue(m_indexer.runIndexerCommand(0.4));



  
    
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
