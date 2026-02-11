// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import java.lang.Exception;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Swerve extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final PathPlannerAuto m_auto;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final SwerveDriveKinematics m_kinematics;
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private final SlewRateLimiter filter = new SlewRateLimiter(SwerveDrivebaseConstants.kSlewRateLimiter);
  
  private final PIDController m_alignPID = 
    new PIDController(0,0,0);

  public Swerve() {
    m_gyro.reset();

    m_frontLeft = new SwerveModule(
      SwerveDrivebaseConstants.kFrontLeftDriveID,
      SwerveDrivebaseConstants.kFrontLeftTurnID,
      SwerveDrivebaseConstants.kFrontLeftCANCoderID,
      SwerveDrivebaseConstants.kFrontLeftCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kCANcoderAbsDiscontPoint
    );

    m_frontRight = new SwerveModule(
      SwerveDrivebaseConstants.kFrontRightDriveID,
      SwerveDrivebaseConstants.kFrontRightTurnID,
      SwerveDrivebaseConstants.kFrontRightCANCoderID,
      SwerveDrivebaseConstants.kFrontRightCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kCANcoderAbsDiscontPoint
    );

    m_backLeft = new SwerveModule(
      SwerveDrivebaseConstants.kBackLeftDriveID,
      SwerveDrivebaseConstants.kBackLeftTurnID,
      SwerveDrivebaseConstants.kBackLeftCANCoderID,
      SwerveDrivebaseConstants.kBackLeftCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kCANcoderAbsDiscontPoint
    );

    m_backRight = new SwerveModule(
      SwerveDrivebaseConstants.kBackRightDriveID,
      SwerveDrivebaseConstants.kBackRightTurnID,
      SwerveDrivebaseConstants.kBackRightCANCoderID,
      SwerveDrivebaseConstants.kBackRightCANCoderMagnetOffset,
      SwerveDrivebaseConstants.kCANcoderAbsDiscontPoint
    );

    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveDrivebaseConstants.kWheelBase / 2, SwerveDrivebaseConstants.kTrackWidth / 2),
      new Translation2d(SwerveDrivebaseConstants.kWheelBase / 2, -SwerveDrivebaseConstants.kTrackWidth / 2),
      new Translation2d(-SwerveDrivebaseConstants.kWheelBase / 2, SwerveDrivebaseConstants.kTrackWidth / 2),
      new Translation2d(-SwerveDrivebaseConstants.kWheelBase / 2, -SwerveDrivebaseConstants.kTrackWidth / 2)
    );
    logData();

    m_auto = new PathPlannerAuto(AutoBuilder.getAllAutoNames().get(0));
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, Rotation2d.fromDegrees(-m_gyro.getAngle()), getModulePositions(), m_auto.getStartingPose());
  
    //Loads config from GUI settings but can also be stored in constants later
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    //Configure AutoBuilder
    
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> drive(speeds), 
      new PPHolonomicDriveController(
        new PIDConstants(0.0, 0.0, 0.0), 
        new PIDConstants(0.0, 0.0, 0.0)
      ), 
      config, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
      );
    
  }


  //DATA LOGGING
  public void logData() {
    // SmartDashboard.putNumber("turnP", 0);
    // /
    // SmartDashboard.putNumber("turnI", 0);
    // SmartDashboard.putNumber("turnD", 0);

    // //not necessary but can be useful for debugging
    SmartDashboard.putNumber("gyro", -m_gyro.getAngle());
    // SmartDashboard.putNumber("gyro rate", m_gyro.getRate());
    // SmartDashboard.putNumber("gyro pitch", m_gyro.getPitch());
    // SmartDashboard.putNumber("gyro roll", m_gyro.getRoll());
    
  }

  //PATHPLANNER

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
        m_swerveModuleStates, SwerveDrivebaseConstants.kMaxMetersPerSecond);
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

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
      m_frontLeft.getSwerveModulePosition(),
      m_frontRight.getSwerveModulePosition(),
      m_backLeft.getSwerveModulePosition(),
      m_backRight.getSwerveModulePosition()
    };
  }

  public Pose2d getPose(){
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d newPose){
    m_poseEstimator.resetPose(newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(m_frontLeft.getCurrentState(), m_frontRight.getCurrentState(), m_backLeft.getCurrentState(), m_backRight.getCurrentState());
  }
  
  public Command resetEncodersCommand() {
    return this.runOnce(this::resetEncoders);
  }

  public Command resetGyroCommand() {
    return this.runOnce(this::resetGyro);
  }
  

  public void resetEncoders(){
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_backLeft.resetEncoder();
    m_backRight.resetEncoder();
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

  //AUTO ALIGN
  double getTargetingAngularVelocity() { // aiming control
      double tx = LimelightHelpers.getTX(LimelightConstants.kName);

      double targetingAngularVelocity = tx * LimelightConstants.kAimP;

      //conversion to radians/second
      targetingAngularVelocity *= SwerveDrivebaseConstants.kMaxAngularSpeed;
      //invert since tx is positive to the right
      targetingAngularVelocity *= -1.0;

      return targetingAngularVelocity;
  }

  double getTargetingForwardSpeed() { // ranging control
      double targetingForwardSpeed = LimelightHelpers.getTY(LimelightConstants.kName) * LimelightConstants.kRangeP;
      targetingForwardSpeed *= SwerveDrivebaseConstants.kMaxMetersPerSecond;
      targetingForwardSpeed *= -1.0; //invert since ty is positive when target is above crosshair
      return targetingForwardSpeed;
  }

  public void periodic() {
    logData();
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX(LimelightConstants.kName));
    SmartDashboard.putNumber("TY", LimelightHelpers.getTY(LimelightConstants.kName));
    SmartDashboard.putNumber("Straight Distance", this.getStraightDistance());
    // SmartDashboard.putNumber("Offset", this.calculateDistanceFromAprilTag());
  }

}