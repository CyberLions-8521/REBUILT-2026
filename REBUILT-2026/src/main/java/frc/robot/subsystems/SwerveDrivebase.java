// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.studica.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Constants.SwerveConstants;
import frc.robot.utils.SwerveModule;



public class SwerveDrivebase extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final SwerveDriveKinematics m_kinematics;

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private PIDController m_TXController = new PIDController(LimelightConstants.TXControllerP, 0, LimelightConstants.TXControllerD);
  private PIDController m_radiusController = new PIDController(LimelightConstants.radiusControllerP, 0, LimelightConstants.radiusControllerD);

  private Pose3d targetPoseRobot;

  public SwerveDrivebase() {
    m_gyro.reset();
    SmartDashboard.putNumber("TX P", LimelightConstants.TXControllerP);
    SmartDashboard.putNumber("TX D", LimelightConstants.TXControllerD);
    SmartDashboard.putNumber("TX FF", LimelightConstants.TXControllerFF);
    SmartDashboard.putNumber("radius P", LimelightConstants.radiusControllerP);
    SmartDashboard.putNumber("radius D", LimelightConstants.TXControllerD);

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

    SmartDashboard.putNumber("turnP", 0);
    SmartDashboard.putNumber("turnD", 0);

    m_radiusController.setTolerance(0.1);
    m_TXController.setTolerance(2);

  }


  public void logData() {
    SmartDashboard.putNumber("gyro", -m_gyro.getAngle());
    m_frontLeft.logData("Front Left");
    m_frontRight.logData("Front Right");
    m_backLeft.logData("Back Left");
    m_backRight.logData("Back Right");

  }

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
    m_gyro.setAngleAdjustment(0);
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

  public void zeroHeading() {
    m_gyro.reset();
  }
  
  public Command resetEncodersCommand() {
    return this.runOnce(this::resetEncoders);
  }

  public Command resetGyroCommand() {
    return this.runOnce(this::resetGyro);
  }
  

  public void resetEncoders(){
    m_frontLeft.zeroDriveEncoder();
    m_frontRight.zeroDriveEncoder();
    m_backLeft.zeroDriveEncoder();
    m_backRight.zeroDriveEncoder();
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

  @Override
  public void periodic() {
    // tunePID();
    // tuneTXController();
    // getLimelightData();
  }

  // public void tunePID () {
  //   double turnP = SmartDashboard.getNumber("turnP", 0);
  //   double turnD = SmartDashboard.getNumber("turnD", 0);
  

  //   if (SwerveConstants.kTurnP != turnP || SwerveConstants.kTurnD != turnD) {
  //       m_frontLeft.configTurnPID(turnP, turnD);
  //       m_frontRight.configTurnPID(turnP, turnD);
  //       m_backLeft.configTurnPID(turnP, turnD);
  //       m_backRight.configTurnPID(turnP, turnD);
  //       SwerveConstants.kTurnP = turnP;
  //       SwerveConstants.kTurnD = turnD;
  //   }
  // }

  public void tuneTXController() {
    double TXP = SmartDashboard.getNumber("TX P", LimelightConstants.TXControllerP);
    double TXD = SmartDashboard.getNumber("TX D", LimelightConstants.TXControllerD);
    double TXFF = SmartDashboard.getNumber("TX FF", LimelightConstants.TXControllerFF);
    double radiusP = SmartDashboard.getNumber("radius P", LimelightConstants.radiusControllerP);
    double radiusD = SmartDashboard.getNumber("radius D", LimelightConstants.TXControllerD);
    LimelightConstants.TXControllerFF = TXFF;
    m_TXController.setP(TXP);
    m_TXController.setD(TXD);
    m_radiusController.setP(radiusP);
    m_radiusController.setD(radiusD);
  }

  public void getLimelightData() {
    SmartDashboard.putNumber("TX (degrees)", LimelightHelpers.getTX(LimelightConstants.limelightName));
    SmartDashboard.putNumber("TY (degrees)", LimelightHelpers.getTY(LimelightConstants.limelightName));
  }

  public Supplier<Double> getTXAdujstmentRotation(double angle) {
    return () -> {
        double feedforward = 0; //LimelightConstants.TXControllerFF * (tangentialVelocity.get() / getRadiusSupplier().get());
        double adjustment = feedforward + m_TXController.calculate(LimelightHelpers.getTX(LimelightConstants.limelightName), angle);
        double deadBandAdjustment = MathUtil.applyDeadband(adjustment, 0.1);
        return MathUtil.clamp(deadBandAdjustment, -6, 6);
    };
  }

  public double getRadius() {
      targetPoseRobot = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName);
      double x = targetPoseRobot.getX();
      double z = targetPoseRobot.getZ();
      return Math.sqrt(x * x + z * z);
  }

  public Supplier<Double> getRadiusSupplier() {
    return () -> getRadius();
  }

  public Supplier<Double> getRadiusAdjustment() {
    return () -> {
      if (LimelightHelpers.getTV(LimelightConstants.limelightName) && getRadiusSupplier().get() < LimelightConstants.minimumDistance - 0.1) {
          double adjustment = m_radiusController.calculate(getRadiusSupplier().get(), LimelightConstants.minimumDistance);
          return MathUtil.applyDeadband(adjustment, 0.05);
        } else {
          return 0.0;
        }
      };
    }

    public static final class LimelightConstants {
        public static final String limelightName = "limelight";
        public static  double TXControllerP = 0.062;
        public static  double TXControllerD = 0;
        public static  double TXControllerFF = 0;
        public static  double radiusControllerP = 1.5;
        public static  double radiusControllerD = 0;
        public static final double tagCenterOffset = 0;
        public static final double minimumDistance = 2.25;
    }
  
}