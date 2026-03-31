// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  CommandXboxController m_driveController = new CommandXboxController(0);
  CommandXboxController m_subsystemController = new CommandXboxController(1);
  SwerveDrivebase m_drivebase = new SwerveDrivebase();
  Shooter m_shooter = new Shooter();
  Intake m_intake = new Intake();
  Indexer m_indexer = new Indexer();

  public static final SlewRateLimiter vx_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter vy_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);
  public static final SlewRateLimiter omega_limiter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);

  public static final int[] validTags = {12, 18, 20};

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
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightConstants.limelightName, validTags);

    //DRIVEBASE
    m_drivebase.setDefaultCommand(this.getDriveCommand(
      1,
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      getJoystickValues(m_driveController::getRightX, omega_limiter),
      () -> true));
    m_driveController.leftTrigger().whileTrue(this.getDriveCommand(
      0.3, 
      getJoystickValues(m_driveController::getLeftY, vx_limiter),
      getJoystickValues(m_driveController::getLeftX, vy_limiter), 
      getJoystickValues(m_driveController::getRightX, omega_limiter), 
      () -> true));
    m_driveController.leftTrigger().and(() -> LimelightHelpers.getTV(LimelightConstants.limelightName)).whileTrue(this.getDriveAutoAlignCommand(
      0.3,
      m_drivebase.getRadiusAdjustment(),
      getJoystickValues(m_driveController::getLeftX, vy_limiter),
      m_drivebase.getTXAdujstmentRotation(omega_limiter, 0, () -> {return getJoystickValues(m_driveController::getLeftX, vy_limiter).get() * 0.5 * SwerveConstants.kMaxMetersPerSecond;}),
      () -> false));
    m_driveController.a().onTrue(m_drivebase.resetGyroCommand());
    m_driveController.b().onTrue(m_drivebase.resetEncodersCommand());


    // m_shooter.setDefaultCommand(m_shooter.stopFlywheel());

    //SHOOTER
    // m_subsystemController.rightTrigger().whileTrue(m_shooter.ShootWithAprilTagCommand());

    // m_subsystemController.y().whileTrue(m_shooter.ShootWithoutAprilTagCommand(60));
    // m_subsystemController.b().whileTrue(m_shooter.ShootWithoutAprilTagCommand(55));
    // m_subsystemController.a().whileTrue(m_shooter.ShootWithoutAprilTagCommand(45));


    //INDEXER
    m_indexer.setDefaultCommand(m_indexer.stopIndexerCommand());  
    m_subsystemController.rightBumper().whileTrue(m_indexer.runIndexerCommand(0.5));
    m_subsystemController.leftBumper().whileTrue(m_indexer.runIndexerCommand(-0.2));

    
    //INTAKE ROLLERS
    m_subsystemController.leftTrigger().whileTrue(m_intake.getIntakeCommand(0.75));
    m_subsystemController.x().whileTrue(m_intake.getIntakeCommand(0.65));
    m_subsystemController.x().whileTrue(m_indexer.runIndexerCommand(0.4));

    //INTAKE
    m_intake.setDefaultCommand(m_intake.defaultCommand());
    m_subsystemController.povUp().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.retractedEncoderPosition).withTimeout(2));
    m_subsystemController.povDown().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.extendedEncoderPosition).withTimeout(2));
    m_subsystemController.povLeft().onTrue(m_intake.setPivotPositionCommand(IntakeConstants.middleEncoderPosition).withTimeout(2));
    



  
    
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

  private Command getDriveAutoAlignCommand(double multiplier, Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> omega, Supplier<Boolean> fieldRelative) {
    return new RunCommand(
      () -> m_drivebase.drive(
        -vx.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        -vy.get() * multiplier * SwerveConstants.kMaxMetersPerSecond,
        omega.get(),
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
    //     .andThen(m_intake.setPivotOut())
    //     .andThen(m_shooter.shoot().alongWith(Commands.waitSeconds(2)
    //         .andThen(m_indexer.runIndexerCommand(0.6).alongWith(m_intake.getIntakeCommand(0.6)))).withTimeout(8))
    // ));
  }

  public Command getAutonomousCommand() {
    return m_intake.getResetEncoderPosition();
  }

}
