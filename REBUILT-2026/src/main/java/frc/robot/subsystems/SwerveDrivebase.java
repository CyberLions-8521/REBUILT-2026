// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.SwerveConfigs;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;


public class SwerveDrivebase extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final SwerveDriveKinematics m_kinematics;

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final SlewRateLimiter filter = new SlewRateLimiter(SwerveConstants.kSlewRateLimiter);

  private Pose3d targetPoseRobot = new Pose3d();

  private PIDController m_TXController;
  private PIDController m_radiusController = new PIDController(LimelightConstants.radiusControllerP, 0, LimelightConstants.radiusControllerD);
  private SwerveDrivePoseEstimator m_poseEstimator;

  public SwerveDrivebase() {
    resetGyro();
    SmartDashboard.putNumber("TX P", LimelightConstants.TXControllerP);
    SmartDashboard.putNumber("TX D", LimelightConstants.TXControllerD);
    SmartDashboard.putNumber("TX FF", LimelightConstants.TXControllerFF);

    SmartDashboard.putNumber("radius P", LimelightConstants.radiusControllerP);
    SmartDashboard.putNumber("radius D", LimelightConstants.radiusControllerD);

    SmartDashboard.putNumber("drive FF", SwerveConstants.driveFF);
    SmartDashboard.putNumber("drive P", SwerveConstants.driveP);
    SmartDashboard.putNumber("drive D", SwerveConstants.driveD);

    m_frontLeft = new SwerveModule(
      SwerveConstants.kFrontLeftDriveID,
      SwerveConstants.kFrontLeftTurnID,
      SwerveConstants.kFrontLeftMagEncoderID,
      SwerveConstants.kFrontLeftMagEncoderMagnetOffset
    );

    m_frontRight = new SwerveModule(
      SwerveConstants.kFrontRightDriveID,
      SwerveConstants.kFrontRightTurnID,
      SwerveConstants.kFrontRightMagEncoderID,
      SwerveConstants.kFrontRightMagEncoderMagnetOffset
    );

    m_backLeft = new SwerveModule(
      SwerveConstants.kBackLeftDriveID,
      SwerveConstants.kBackLeftTurnID,
      SwerveConstants.kBackLeftMagEncoderID,
      SwerveConstants.kBackLeftMagEncoderMagnetOffset
    );

    m_backRight = new SwerveModule(
      SwerveConstants.kBackRightDriveID,
      SwerveConstants.kBackRightTurnID,
      SwerveConstants.kBackRightMagEncoderID,
      SwerveConstants.kBackRightMagEncoderMagnetOffset
    );

    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveConstants.kWheelBase / 2, SwerveConstants.kTrackWidth / 2),
      new Translation2d(SwerveConstants.kWheelBase / 2, -SwerveConstants.kTrackWidth / 2),
      new Translation2d(-SwerveConstants.kWheelBase / 2, SwerveConstants.kTrackWidth / 2),
      new Translation2d(-SwerveConstants.kWheelBase / 2, -SwerveConstants.kTrackWidth / 2)
    );

    // m_TXController = new PIDController(SwerveConstants.TXP, 0, SwerveConstants.TXD);
    // m_TXController.setTolerance(Units.degreesToRadians(1));
    // m_TXController = new PIDController(LimelightConstants.TXControllerP, 0, LimelightConstants.TXControllerD);
    // m_TXController.setTolerance(1);

    m_poseEstimator = 
      new SwerveDrivePoseEstimator(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          getStartingPose(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = null;
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (chassisSpeeds, feedforward) -> drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
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

  @Override
  public void periodic() {
    this.resetEncoders();
  }

  public void getLimelightData() {
    SmartDashboard.putNumber("TX (deg)", LimelightHelpers.getTX(LimelightConstants.limelightName));
    SmartDashboard.putNumber("TY (deg)", LimelightHelpers.getTY(LimelightConstants.limelightName));
    // targetPoseRobot = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName);
    // SmartDashboard.putNumber("Limelight X (m)", targetPoseRobot.getX());
    // SmartDashboard.putNumber("Limelight Z (m)", targetPoseRobot.getZ());
    // SmartDashboard.putNumber("target offset", getTXTargetOffset());
  }

  public void tuneTXController() {
    double TXP = SmartDashboard.getNumber("TX P", LimelightConstants.TXControllerP);
    double TXD = SmartDashboard.getNumber("TX D", LimelightConstants.TXControllerD);
    double TXFF = SmartDashboard.getNumber("TX FF", LimelightConstants.TXControllerFF);
    double radiusP = SmartDashboard.getNumber("radius P", LimelightConstants.radiusControllerP);
    double radiusD = SmartDashboard.getNumber("radius D", LimelightConstants.TXControllerD);
    LimelightConstants.TXControllerFF = TXFF;
    if (TXP != LimelightConstants.TXControllerP || TXD != LimelightConstants.TXControllerD)
    m_TXController.setP(TXP);
    m_TXController.setD(TXD);
    LimelightConstants.TXControllerP = TXP;
    LimelightConstants.TXControllerD = TXD;
    m_radiusController.setP(radiusP);
    m_radiusController.setD(radiusD);
  }

  public Supplier<Double> getTXAdujstmentRotation(SlewRateLimiter limiter, double angle, Supplier<Double> tangentialVelocity) {
    return () -> {
      double feedforward = LimelightConstants.TXControllerFF * (tangentialVelocity.get() / getRadiusSupplier().get());
      double adjustment = feedforward + m_TXController.calculate(LimelightHelpers.getTX(LimelightConstants.limelightName), angle);
      return MathUtil.clamp(adjustment, -6, 6);
    };
  }

  public double getRadius() {
      targetPoseRobot = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName);
      double x = targetPoseRobot.getX();
      double z = targetPoseRobot.getZ();
      return Math.sqrt(x * x + z * z);
  }

  public double getTXTargetOffset() {
    targetPoseRobot = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.limelightName);
    double offsetX = targetPoseRobot.getX();
    double offsetZ1 = targetPoseRobot.getZ();
    double offsetZ2 = targetPoseRobot.getZ() + LimelightConstants.tagCenterOffset;
    double angle1 = Math.atan(offsetX / offsetZ1);
    double angle2 = Math.atan(offsetX / offsetZ2);

    return angle1 - angle2;
  }

  public Supplier<Double> getRadiusSupplier() {
    return () -> getRadius();
  }

  public Supplier<Double> getRadiusAdjustment() {
    return () -> {
      if (LimelightHelpers.getTV(LimelightConstants.limelightName) && Math.abs(LimelightHelpers.getTX(LimelightConstants.limelightName)) <= 2) {
          return m_radiusController.calculate(getRadiusSupplier().get(), LimelightConstants.minimumDistance);
        } else {
          return 0.0;
        }
      };
    }

  public void TunePID() {
    double driveP = SmartDashboard.getNumber("Drive P", SwerveConstants.driveD);
    double driveD = SmartDashboard.getNumber("Drive D", SwerveConstants.driveD);
    double driveFF = SmartDashboard.getNumber("Drive FF", SwerveConstants.driveFF);

    double turnP = SmartDashboard.getNumber("Turn P", SwerveConstants.turnP);
    double turnD = SmartDashboard.getNumber("Turn D", SwerveConstants.turnD);

    SparkMaxConfig m_driveConfig = SwerveConfigs.m_configDrive;
    SparkMaxConfig m_turnConfig = SwerveConfigs.m_configTurn;

    if (driveP != SwerveConstants.driveP || driveD != SwerveConstants.driveD || turnP != SwerveConstants.turnP || turnD != SwerveConstants.turnD || SwerveConstants.driveFF != driveFF) {
      m_driveConfig.closedLoop.pidf(driveP, 0, driveD, driveFF);
      m_turnConfig.closedLoop.pidf(turnP, 0, turnD, 0);
      m_frontLeft.configure(m_driveConfig, m_turnConfig);
      m_frontRight.configure(m_driveConfig, m_turnConfig);
      m_backLeft.configure(m_driveConfig, m_turnConfig);
      m_backRight.configure(m_driveConfig, m_turnConfig);
      SwerveConstants.driveP = driveP;
      SwerveConstants.driveD = driveD;
      SwerveConstants.turnP = turnP;
      SwerveConstants.turnD = turnD;
      SwerveConstants.driveFF = driveFF;
    }
  }

    //Taken from example on limelight docs
  public void updateOdometry() {
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });


    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  private Pose2d getStartingPose() {
    return new Pose2d();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(
      new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
      }
    );
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPose(pose);
  }
  
}
