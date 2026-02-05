// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants.SwerveConstants;
import frc.robot.utils.SwerveModule;


public class SwerveDrivebase extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final SwerveDriveKinematics m_kinematics;

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final SlewRateLimiter filter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);

  public SwerveDrivebase() {
    m_gyro.reset();

    m_frontLeft = new SwerveModule(
      SwerveConstants.kFrontLeftDriveID,
      SwerveConstants.kFrontLeftTurnID,
      SwerveConstants.kFrontLeftCANCoderID,
      SwerveConstants.kFrontLeftCANCoderMagnetOffset
    );

    m_frontRight = new SwerveModule(
      SwerveConstants.kFrontRightDriveID,
      SwerveConstants.kFrontRightTurnID,
      SwerveConstants.kFrontRightCANCoderID,
      SwerveConstants.kFrontRightCANCoderMagnetOffset
    );

    m_backLeft = new SwerveModule(
      SwerveConstants.kBackLeftDriveID,
      SwerveConstants.kBackLeftTurnID,
      SwerveConstants.kBackLeftCANCoderID,
      SwerveConstants.kBackLeftCANCoderMagnetOffset
    );

    m_backRight = new SwerveModule(
      SwerveConstants.kBackRightDriveID,
      SwerveConstants.kBackRightTurnID,
      SwerveConstants.kBackRightCANCoderID,
      SwerveConstants.kBackRightCANCoderMagnetOffset
    );

    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveConstants.kWheelBase / 2, SwerveConstants.kTrackWidth / 2),
      new Translation2d(SwerveConstants.kWheelBase / 2, -SwerveConstants.kTrackWidth / 2),
      new Translation2d(-SwerveConstants.kWheelBase / 2, SwerveConstants.kTrackWidth / 2),
      new Translation2d(-SwerveConstants.kWheelBase / 2, -SwerveConstants.kTrackWidth / 2)
    );
    logData();
  }

  //DATA LOGGING

  public void logData() {

    SmartDashboard.putNumber("gyro", -m_gyro.getAngle());
    
  }

  //DRIVE COMMANDS
  public void drive(double vx, double vy, double omega, boolean fieldRelative) {
    SwerveModuleState[] m_swerveModuleStates;
    if(fieldRelative) {
      m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(-m_gyro.getAngle())));
    } else {
      m_swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, omega));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_swerveModuleStates, SwerveConstants.kMaxMetersPerSecond);
    m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
    m_frontRight.setDesiredState(m_swerveModuleStates[1]);
    m_backLeft.setDesiredState(m_swerveModuleStates[2]);
    m_backRight.setDesiredState(m_swerveModuleStates[3]);
  }

  public FunctionalCommand getDriveCommand(double distance) {
    return new FunctionalCommand (
      () -> this.resetEncoders(),
      () -> this.drive(1.0, 0, 0,true),
      interrupted -> this.drive(0, 0, 0, true), 
      () -> MathUtil.isNear(distance, this.getStraightDistance(), 0.1), 
      this);
  }

  public FunctionalCommand getReversedDriveCommand(double distance) {
    return new FunctionalCommand (
      () -> this.resetEncoders(),
      () -> this.drive(-1.0, 0, 0,true),
      interrupted -> this.drive(0, 0, 0, true), 
      () -> MathUtil.isNear(distance, this.getStraightDistance(), 0.1), 
      this);
  }

  public void resetGyro() {
    m_gyro.reset();
    this.setGyro(0);
  }

  public void setGyro(double angle){
    m_gyro.setAngleAdjustment(angle);
  }

  public Command setGyroCommand(double angle) {
    return this.runOnce(() -> this.setGyro(angle));
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }

  public double getRoll() {
    return m_gyro.getRoll();
  }
  
  public Command resetEncodersCommand() {
    return this.runOnce(this::resetEncoders);
  }

  public Command resetGyroCommand() {
    return this.runOnce(this::resetGyro);
  }
  

  public void resetEncoders(){
    m_frontLeft.resetDriveEncoder();
    m_frontRight.resetDriveEncoder();
    m_backLeft.resetDriveEncoder();
    m_backRight.resetDriveEncoder();
  }

  public void setEncoderDistance(double distance){
    m_frontLeft.setEncoderDistance(distance);
    m_frontRight.setEncoderDistance(distance);
    m_backLeft.setEncoderDistance(distance);
    m_backRight.setEncoderDistance(distance);
  }

  public double getStraightDistance() { // meters
    return (Math.abs(m_frontLeft.getDriveDistance())  +
            Math.abs(m_frontRight.getDriveDistance()) +
            Math.abs(m_backLeft.getDriveDistance())   +
            Math.abs(m_backRight.getDriveDistance())) / 4.0;
  }

}