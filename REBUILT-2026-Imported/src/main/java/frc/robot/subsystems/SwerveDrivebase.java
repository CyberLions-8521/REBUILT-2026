// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.SubsystemBase;
import frc.robot.utils.Constants.SwerveConstants;
import frc.robot.utils.SwerveModule;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimateType;

import org.wpilib.math.kinematics.SwerveModuleVelocity;


public class SwerveDrivebase extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final Limelight m_limelight;
  private final SwerveDriveKinematics m_kinematics;

  private final OnboardIMU m_gyro = new OnboardIMU(OnboardIMU.MountOrientation.FLAT);

  private PIDController m_TXController = new PIDController(LimelightConstants.TXControllerP, 0, LimelightConstants.TXControllerD);
  private PIDController m_radiusController = new PIDController(LimelightConstants.radiusControllerP, 0, LimelightConstants.radiusControllerD);

  private Pose3d targetPoseRobot;

  public SwerveDrivebase(Limelight limelight) {
    m_limelight = limelight;
    m_gyro.resetYaw();
    SmartDashboard.putNumber("TX P", LimelightConstants.TXControllerP);
    SmartDashboard.putNumber("TX D", LimelightConstants.TXControllerD);
    SmartDashboard.putNumber("TX FF", LimelightConstants.TXControllerFF);
    SmartDashboard.putNumber("radius P", LimelightConstants.radiusControllerP);
    SmartDashboard.putNumber("radius D", LimelightConstants.radiusControllerD);

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
    SmartDashboard.putNumber("gyro", -m_gyro.getAngleZ());
    m_frontLeft.logData("Front Left");
    m_frontRight.logData("Front Right");
    m_backLeft.logData("Back Left");
    m_backRight.logData("Back Right");

  }

  public void drive(double vx, double vy, double omega, boolean fieldRelative) {
    SwerveModuleVelocity[] m_swerveModuleStates;
    if(fieldRelative) {
      m_swerveModuleStates = m_kinematics.toSwerveModuleVelocities(
      new ChassisVelocities(vx,vy,omega).toFieldRelative(getHeading()));
    } else {
      m_swerveModuleStates = m_kinematics.toSwerveModuleVelocities(new ChassisVelocities(vx, vy, omega));
    }

    m_swerveModuleStates = SwerveDriveKinematics.desaturateWheelVelocities(
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
    m_gyro.resetYaw();
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngleZ());
  }

  public void zeroHeading() {
    m_gyro.resetYaw();
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
    SmartDashboard.putNumber("TX (degrees)", m_limelight.getTXDegrees());
    SmartDashboard.putNumber("TY (degrees)", m_limelight.getTYDegrees());
  }

  public Supplier<Double> getTXAdujstmentRotation(double angle) {
    return () -> {
        double feedforward = 0; //LimelightConstants.TXControllerFF * (tangentialVelocity.get() / getRadiusSupplier().get());
        double adjustment = feedforward + m_TXController.calculate(m_limelight.getTXDegrees(), angle);
        double deadBandAdjustment = MathUtil.applyDeadband(adjustment, 0.1);
        return Math.clamp(deadBandAdjustment, -6, 6);
    };
  }

  public boolean isAutoAligned() {
    return m_TXController.atSetpoint() && m_limelight.hasTarget();
  }

  public double getRadius() {
      targetPoseRobot = m_limelight.getRobotPose(PoseEstimateType.MT1_WPIBLUE);
      double x = targetPoseRobot.getX();
      double z = targetPoseRobot.getZ();
      return Math.sqrt(x * x + z * z);
  }

  public Supplier<Double> getRadiusSupplier() {
    return () -> getRadius();
  }

  public Supplier<Double> getRadiusAdjustment() {
    return () -> {
      if (m_limelight.hasTarget() && getRadiusSupplier().get() < LimelightConstants.minimumDistance - 0.1) {
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