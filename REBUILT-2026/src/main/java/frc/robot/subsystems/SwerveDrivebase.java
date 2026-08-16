// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.Constants.SwerveConstants;
import frc.robot.utils.SwerveModule;



public class SwerveDrivebase extends SubsystemBase {
  
  // ---------------------------------------------------- Fields ----------------------------------------------------
  //#region
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Field2d m_field;

  private Rotation2d m_simHeading = new Rotation2d();
  private ChassisSpeeds m_lastRobotRelativeSpeeds = new ChassisSpeeds();

  private ProfiledPIDController m_autoAlignPID;
  private ProfiledPIDController m_autoDistancePID;

  private double m_lastDriveP = SwerveConstants.kDriveP;
  private double m_lastDriveV = SwerveConstants.kDriveV;
  private double m_lastTurnP = SwerveConstants.kTurnP;
  private double m_lastTurnD = SwerveConstants.kTurnD;
  private double m_lastAutoAlignP = SwerveConstants.kAutoAlignP;
  private double m_lastAutoAlignI = SwerveConstants.kAutoAlignI;
  private double m_lastAutoAlignD = SwerveConstants.kAutoAlignD;
  private double m_lastAutoDistanceP = SwerveConstants.kAutoDistanceP;
  private double m_lastAutoDistanceI = SwerveConstants.kAutoDistanceI;
  private double m_lastAutoDistanceD = SwerveConstants.kAutoDistanceD;

  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  
  //#endregion






  /* ---------------------------------------------------- Basic ----------------------------------------------------
  * The minimum required methods for the drivebase to function
  * Works by adjusting the state of each swerve module. The states are calculated through inverse kinematics
  * Desaturating wheel speeds makes sure they are below the max attainable speed 
  * getHeading() is used by drive(). Otherwise, it would've eneded up in the odometry or auto-alignment section
  */
  //#region

  /** Creates the swerve drivebase and initializes its modules, kinematics, and telemetry. */
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
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, getHeading(), getModulePositions(), new Pose2d());
    m_field = new Field2d();

    m_autoAlignPID = new ProfiledPIDController(
      SwerveConstants.kAutoAlignP, 
      SwerveConstants.kAutoAlignI, 
      SwerveConstants.kAutoAlignD, 
      new TrapezoidProfile.Constraints(SwerveConstants.kMaxAngularSpeed, SwerveConstants.kMaxAngularAcceleration)
    );
    m_autoAlignPID.enableContinuousInput(-Math.PI, Math.PI); // angle wrapping
    m_autoAlignPID.setTolerance(SwerveConstants.kAutoAlignTolerance);
    m_autoDistancePID = new ProfiledPIDController(
      SwerveConstants.kAutoDistanceP, 
      SwerveConstants.kAutoDistanceI, 
      SwerveConstants.kAutoDistanceD, 
      new TrapezoidProfile.Constraints(SwerveConstants.kMaxMetersPerSecond, SwerveConstants.kMaxAcceleration)
    );
    m_autoDistancePID.setTolerance(SwerveConstants.kAutoDistanceTolerance);

    SmartDashboard.putNumber("turnP", 0);
    SmartDashboard.putNumber("turnD", 0);

    SmartDashboard.putData("Field", m_field); // puts robot data on the field for simulation

    SmartDashboard.putNumber("Drive P", SwerveConstants.kDriveP);
    SmartDashboard.putNumber("Drive V (feed forward)", SwerveConstants.kDriveV);
    SmartDashboard.putNumber("Turn P", SwerveConstants.kTurnP);
    SmartDashboard.putNumber("Turn D", SwerveConstants.kTurnD);
    SmartDashboard.putNumber("Auto-align P", SwerveConstants.kAutoAlignP);
    SmartDashboard.putNumber("Auto-align I", SwerveConstants.kAutoAlignI);
    SmartDashboard.putNumber("Auto-align D", SwerveConstants.kAutoAlignD);
    SmartDashboard.putNumber("Auto-distance P", SwerveConstants.kAutoDistanceP);
    SmartDashboard.putNumber("Auto-distance I", SwerveConstants.kAutoDistanceI);
    SmartDashboard.putNumber("Auto-distance D", SwerveConstants.kAutoDistanceD);

    setupPathPlanner();
  }

  /** Drives the robot with the given chassis speeds, optionally relative to the field. */
  public void drive(double vx, double vy, double omega, boolean fieldRelative) { 
    SwerveModuleState[] m_swerveModuleStates;
    if(fieldRelative) {
      m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeading()));
    } else {
      m_swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, omega));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
      m_swerveModuleStates, SwerveConstants.kMaxMetersPerSecond);
    m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
    m_frontRight.setDesiredState(m_swerveModuleStates[1]);
    m_backLeft.setDesiredState(m_swerveModuleStates[2]);
    m_backRight.setDesiredState(m_swerveModuleStates[3]);

    m_lastRobotRelativeSpeeds = m_kinematics.toChassisSpeeds(m_swerveModuleStates);
  }

  /** Returns the current robot heading. */
  public Rotation2d getHeading() { 
    if (RobotBase.isSimulation()) return m_simHeading;
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /** Updates Limelight data, odometry, and the dashboard field display each scheduler loop. */
  @Override
  public void periodic() { 
    getLimelightData();
    tunePIDControllers();
    
    m_poseEstimator.update(getHeading(), getModulePositions());

    LimelightHelpers.PoseEstimate visionEstimation = getLimelightPose(LimelightConstants.limelightName);
    if (visionEstimation != null) {
      double xyStdDev = getStandardDeviation(visionEstimation);
      m_poseEstimator.addVisionMeasurement(
        visionEstimation.pose,
        visionEstimation.timestampSeconds,
        VecBuilder.fill(
          xyStdDev,
          xyStdDev,
          9999999 // since higher numbers translate to less trust, dont trust Limelight rotations at all!!!!! that's the gyro's job
        )
      );
    }
    
    m_field.setRobotPose(getPose());

  }

  //#endregion






  /* --------------------------------------------------- Odometry ---------------------------------------------------
  * Calculates an estimated position of the robot on the field given field data from encoders, the gyro, and Limelight/vision
  * The readings are given through periodic()
  * Limelight/vision can be a very useful sensor for odometry given that it is trustable. So getLimelightPose may return null...
  * Standard deviation is used to tell the pose estimator/odometry how much to trust a sensor. 
  * The trust on vision is dynamic through # of tags, distance, etc.
  * Pathplanner is very important for odometry as it returns the field data for auto after it's done
  */
  //#region
  
  /** Returns all module positions in the order expected by the swerve kinematics. */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  /** Returns the current estimated robot pose. (Also relied on for auto-align) */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Resets drive encoder distance and starts odometry from the given pose. */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /** Returns the estimated pose and timestamp from a Limelight that can be added to the pose estimation. */
  private LimelightHelpers.PoseEstimate getLimelightPose(String name) {
    LimelightHelpers.SetRobotOrientation( // Set orientation values for the MegaTag2 algorithm
      name,
      getHeading().getDegrees(),
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    );

    LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    if (estimate.tagCount == 0) return null; // If there are no tags in sight, return null

    if (Math.abs(m_gyro.getRate()) > LimelightConstants.kMaxViableGyroRate) return null; // If there is too much rotational motion, return null
    
    if ( // Check x, y, and timestamp values
      !Double.isFinite(estimate.pose.getX()) ||
      !Double.isFinite(estimate.pose.getY()) ||
      !Double.isFinite(estimate.timestampSeconds)
    ) return null;

    return estimate;

  }

  /** Returns a varying standard deviation value for visionary estimation depending on # of tags, distance from tags, etc. Smaller value = more trust, vice versa*/
  private double getStandardDeviation(LimelightHelpers.PoseEstimate estimate) {
    if (estimate.tagCount >= 2) return 0.25;
    else if (estimate.avgTagDist < 2.0) return 0.5;
    else return 1.5;
  }

  //#endregion





  
  /* -------------------------------------------------- Simulation --------------------------------------------------
  * Simulation is the least straight-forward for me out of the rest of the sections so this is the most important description
  * As said in the module's description for its job during simulation, each module holds "fake" values of data - turn position, distance, and velocity
  * The false data of the gyroscope is the simulation heading
    * getHeading() is one of those methods that starts with the conditional for simulation, just like in the module code
  * The Field2d is used for both the actual robot movement out of simulation and the simulated movement during simulation
  * The best way to think of simulation is the ideal movement of the robot from input
  */
  //#region

  /** Advances the simulated heading and module positions each simulation loop. */
  @Override
  public void simulationPeriodic() {
    double delay = 0.02;
    
    m_simHeading = m_simHeading.plus(Rotation2d.fromRadians(m_lastRobotRelativeSpeeds.omegaRadiansPerSecond * delay));

    m_frontLeft.updateSim(delay);
    m_frontRight.updateSim(delay);
    m_backLeft.updateSim(delay);
    m_backRight.updateSim(delay);
  }

  //#endregion





  
  /* ------------------------------------------------ Auto-alignment ------------------------------------------------
  * Reducing the load on the drivers is very important during the game!!!! This means assisting movement through code
  * Auto-alignment makes it easier for a driver to move to a certain destination
  * Auto-alignment can be based on odometry or Limelight vision
  * Using a profiled PID is better than a PID, because of the relationship between error and distance; farther = more aggresive on the motors = bad
  */
  //#region

  /** Auto-align to a certain point on the field using odometry */
  public Command odometryAutoAlign(Translation2d m_targetPoint, Supplier<Double> i_vxInput, Supplier<Double> i_vyInput, boolean stop) {
    return new FunctionalCommand(
      () -> {
        // compacted math of below to set the current position and target before command runs
        Pose2d position = getPose();
        Rotation2d targetHeading = m_targetPoint.minus(position.getTranslation()).getAngle();
        m_autoAlignPID.reset(position.getRotation().getRadians());
        m_autoAlignPID.setGoal(targetHeading.getRadians()); // this does not need to be run constantly when stop is false (tested)
      },
      () -> {
        // First part - calculate dynamic heading
        Pose2d position = getPose();
        Translation2d fieldLocation = position.getTranslation();
        Translation2d vectorBetweenPoints = m_targetPoint.minus(fieldLocation);
        Rotation2d dynamicHeading = vectorBetweenPoints.getAngle();

        // Second part - convert dynamic heading to be suitable with the drive method
        Rotation2d currentHeading = position.getRotation();
        double pidOutput = m_autoAlignPID.calculate(currentHeading.getRadians(), dynamicHeading.getRadians());
        pidOutput = MathUtil.clamp(pidOutput, -SwerveConstants.kMaxAngularSpeed, SwerveConstants.kMaxAngularSpeed);

        // Third part - Run drive command and stop alignment when needed
        // The only time it shouldn't is when the robot is too close to the target and break the math
        double distance = m_targetPoint.getDistance(fieldLocation);
        this.drive(
          -i_vxInput.get() * SwerveConstants.kMaxMetersPerSecond * ((stop) ? 0.0 : 1.0), // if the robot needs to stop at the end, ignore joystick input
          -i_vyInput.get() * SwerveConstants.kMaxMetersPerSecond * ((stop) ? 0.0 : 1.0),
          (distance > 0.5) ? pidOutput : 0.0,
          true
        );
      }, 
      (interrupted) -> {}, // no need to stop the robot if the command ends prematurely because the default command covers it 
      () -> m_autoAlignPID.atGoal() && stop, 
      this
    );
  }

  /** Auto-distance to a certain point on the field (assuming that the robot is facing it already) */
  public Command odometryAutoDistance(Translation2d m_targetPoint) {
    return new FunctionalCommand(
      () -> {
        // compacted math of below to set the current position and target before command runs
        m_autoDistancePID.reset(m_targetPoint.getDistance(getPose().getTranslation()));
        m_autoDistancePID.setGoal(SwerveConstants.kAutoDistanceTarget);
      },
      () -> {
        // First part - calculate distance between robot and target
        Pose2d position = getPose();
        Translation2d fieldLocation = position.getTranslation();
        double distance = m_targetPoint.getDistance(fieldLocation);

        // Second part - feed distance into PID
        double pidOutput = m_autoDistancePID.calculate(distance, SwerveConstants.kAutoDistanceTarget);
        pidOutput = MathUtil.clamp(pidOutput, -SwerveConstants.kMaxMetersPerSecond, SwerveConstants.kMaxMetersPerSecond);

        // Third part - Run drive command
        this.drive(
          -pidOutput,
          0,
          0,
          false
        );
      }, 
      (interrupted) -> {}, // no need to stop the robot if the command ends prematurely because the default command covers it 
      () -> false, 
      this
    );
  }

  //#endregion





  
  /* -------------------------------------------------- Pathplanner -------------------------------------------------
  * Pathplanner is used to prevent hard-coding autos and makes it much easier on programming
  * I'm gonna be so fr, a lot of the Pathplanner code was copied from a guide ty YASS (Yet Another Software Suite)
  * A lot of Pathplanner code is setting up something called the AutoBuilder which converts paths in the GUI into commands
  * NamedCommands are used to bridge Pathplanner GUI to the rest of the code so other subsystems or commands can be triggered by it
  * Yes there is another driving method because Pathplanner needs it. The difference with this one is that it's robot relative and not field relative
  * We rely on Pathplanner for the initial pose/pose after auto inside of the pose estimator
  */
  //#region
  
  /** Drives the robot using robot-relative chassis speeds, as required by PathPlanner. */
  private void driveRobotRelative(ChassisSpeeds speedsRobotRelative) {
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speedsRobotRelative);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates,
        SwerveConstants.kMaxMetersPerSecond
    );

    m_frontLeft.setDesiredState(moduleStates[0]);
    m_frontRight.setDesiredState(moduleStates[1]);
    m_backLeft.setDesiredState(moduleStates[2]);
    m_backRight.setDesiredState(moduleStates[3]);

    m_lastRobotRelativeSpeeds = m_kinematics.toChassisSpeeds(moduleStates);
  }

  /** Returns the robot-relative chassis speed used by PathPlanner. */
  private ChassisSpeeds getRobotVelocity() {
    return m_kinematics.toChassisSpeeds(
      new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
      }
    );
  }

  /** Setup AutoBuilder for PathPlanner */
  private void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> driveRobotRelative(speedsRobotRelative),
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(SwerveConstants.kTranslationP, SwerveConstants.kTranslationI, SwerveConstants.kTranslationD),
              // Translation PID constants
              new PIDConstants(SwerveConstants.kRotationP, SwerveConstants.kRotationI, SwerveConstants.kRotationD)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /** Return Pathplanner auto command */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  //#endregion





  
  /* ------------------------------------------------- Calibration --------------------------------------------------
  * These commands/methods were leftover from when I started to modify the REBUILT code of the drivebase
  * I'm pretty sure these are not meant for odometry as it could break it completely
  * I have not tested whether these are needed nor if I should even remove them or not
  * They get their own section because they generally do the same thing for the robot
  */
  //#region

  // USING THESE MAY BREAK ODOMETRY DURING OPERATION, HASN'T BEEN TEST YET

  /** Zeros all four drive encoders. */
  private void resetEncoders(){
    m_frontLeft.zeroDriveEncoder();
    m_frontRight.zeroDriveEncoder();
    m_backLeft.zeroDriveEncoder();
    m_backRight.zeroDriveEncoder();
    // safely reset odometry too
    m_poseEstimator.resetPosition(getHeading(), getModulePositions(), getPose());
  }

  /** Resets the gyro and simulated heading to zero. */
  private void resetGyro() {
    if (RobotBase.isSimulation()) m_simHeading = new Rotation2d();
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
    // safely reset odometry too
    m_poseEstimator.resetPosition(getHeading(), getModulePositions(), getPose());
  }

  /** Resets the gyro heading to zero. */
  private void zeroHeading() { 
    m_gyro.reset();
  }

  /** Returns a one-shot command that resets the gyro. */
  public Command resetGyroCommand() { 
    return this.runOnce(this::resetGyro);
  }

  /** Returns a one-shot command that resets all drive encoders. */
  public Command resetEncodersCommand() { 
    return this.runOnce(this::resetEncoders);
  }

  //#endregion





  
  /* -------------------------------------------------- Extraneous --------------------------------------------------
  * These methods were also leftover from when I started to modify the REBUILT drivebase code
  * It's not 100% known for me if they are needed, but it could be good to have anyway
  * tunePIDControllers() is the golden child in this section as it lets us tune PID dynamically during operation without having to recompile the code 
  */
  //#region

  /** Dynamically tune the PID controllers of the drivebase */
  public void tunePIDControllers () {
    double driveP = SmartDashboard.getNumber("Drive P", SwerveConstants.kDriveP);
    double driveV = SmartDashboard.getNumber("Drive V (feed forward)", SwerveConstants.kDriveV);
    double turnP = SmartDashboard.getNumber("Turn P", SwerveConstants.kTurnP);
    double turnD = SmartDashboard.getNumber("Turn D", SwerveConstants.kTurnD);
    double autoAlignP = SmartDashboard.getNumber("Auto-align P", SwerveConstants.kAutoAlignP);
    double autoAilgnI = SmartDashboard.getNumber("Auto-align I", SwerveConstants.kAutoAlignI);
    double autoAlignD = SmartDashboard.getNumber("Auto-align D", SwerveConstants.kAutoAlignD);
    double autoDistanceP = SmartDashboard.getNumber("Auto-distance P", SwerveConstants.kAutoDistanceP);
    double autoDistanceI = SmartDashboard.getNumber("Auto-distance I", SwerveConstants.kAutoDistanceI);
    double autoDistanceD = SmartDashboard.getNumber("Auto-distance D", SwerveConstants.kAutoDistanceD);

    if (driveP != m_lastDriveP || driveV != m_lastDriveV) {
      m_frontLeft.configDrivePID(driveP, driveV);
      m_frontRight.configDrivePID(driveP, driveV);
      m_backLeft.configDrivePID(driveP, driveV);
      m_backRight.configDrivePID(driveP, driveV);

      m_lastDriveP = driveP;
      m_lastDriveV = driveV; 
    }

    if (turnP != m_lastTurnP || turnD != m_lastTurnD) {
      m_frontLeft.configTurnPID(turnP, turnD);
      m_frontRight.configTurnPID(turnP, turnD);
      m_backLeft.configTurnPID(turnP, turnD);
      m_backRight.configTurnPID(turnP, turnD);
    
      m_lastTurnP = turnP;
      m_lastTurnD = turnD;
    }

    if (autoAlignP != m_lastAutoAlignP || autoAilgnI != m_lastAutoAlignI || autoAlignD != m_lastAutoAlignD) {
      m_autoAlignPID.setP(autoAlignP);
      m_autoAlignPID.setI(autoAilgnI);
      m_autoAlignPID.setD(autoAlignD);

      m_lastAutoAlignP = autoAlignP;
      m_lastAutoAlignI = autoAilgnI;
      m_lastAutoAlignD = autoAlignD;
    }

    if (m_lastAutoDistanceP != autoDistanceP || m_lastAutoDistanceI != autoDistanceI || m_lastAutoDistanceD != autoDistanceD) {
      m_autoDistancePID.setP(autoDistanceP);
      m_autoDistancePID.setI(autoDistanceI);
      m_autoDistancePID.setD(autoDistanceD);

      m_lastAutoDistanceP = autoDistanceP;
      m_lastAutoDistanceI = autoDistanceI;
      m_lastAutoDistanceD = autoDistanceD;
    }
  }

  /** Logs gyro and module telemetry to SmartDashboard. */
  public void logData() {
    SmartDashboard.putNumber("gyro", -m_gyro.getAngle());
    m_frontLeft.logData("Front Left");
    m_frontRight.logData("Front Right");
    m_backLeft.logData("Back Left");
    m_backRight.logData("Back Right");
  }

  /** Logs the Limelight target offsets to SmartDashboard. */
  private void getLimelightData() {
    SmartDashboard.putNumber("TX (degrees)", LimelightHelpers.getTX(LimelightConstants.limelightName));
    SmartDashboard.putNumber("TY (degrees)", LimelightHelpers.getTY(LimelightConstants.limelightName));
  }

  /** Returns a command that drives forward until the average module distance reaches the target. */
  public FunctionalCommand getDriveCommand(double distance) { 
    return new FunctionalCommand (
      () -> this.resetEncoders(),
      () -> this.drive(1.0, 0, 0,true),
      interrupted -> this.drive(0, 0, 0, true), 
      () -> MathUtil.isNear(distance, this.getStraightDistance(), 0.1), 
      this);
  }

  /** Returns a command that drives backward until the average module distance reaches the target. */
  public FunctionalCommand getReversedDriveCommand(double distance) {  
    return new FunctionalCommand (
      () -> this.resetEncoders(),
      () -> this.drive(-1.0, 0, 0,true),
      interrupted -> this.drive(0, 0, 0, true), 
      () -> MathUtil.isNear(distance, this.getStraightDistance(), 0.1), 
      this);
  }

  /** Stops all four swerve modules. */
  public void stopModules() { 
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  /** Sets all drive encoder distances to the given value in meters. */
  private void setEncoderDistance(double distance){ 
    m_frontLeft.setEncoderDistance(distance);
    m_frontRight.setEncoderDistance(distance);
    m_backLeft.setEncoderDistance(distance);
    m_backRight.setEncoderDistance(distance);
  }

  /** Returns the average absolute drive distance across all four modules in meters. */
  private double getStraightDistance() { // meters
    return (Math.abs(m_frontLeft.getDriveDistance())  +
            Math.abs(m_frontRight.getDriveDistance()) +
            Math.abs(m_backLeft.getDriveDistance())   +
            Math.abs(m_backRight.getDriveDistance())) / 4.0;
  }

  //#endregion
  
}
