// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimateType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.simulation.OnboardIMUSim;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.FieldObject2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.Constants.SwerveConstants;
import frc.robot.utils.Configs.SwerveConfigs;



public class SwerveDrivebase extends SubsystemBase {
  
  // ---------------------------------------------------- Fields ----------------------------------------------------
  //#region
  private final SwerveModule m_frontLeft; // basic section
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  private final SwerveDriveKinematics m_kinematics;
  private final OnboardIMU m_gyro = new OnboardIMU(OnboardIMU.MountOrientation.FLAT);
  private boolean m_pathPlannerAvailable = false;

  private final SwerveDrivePoseEstimator m_poseEstimator; // odometry
  private final Pose3d m_cameraPoseRobotSpace = new Pose3d(
    LimelightConstants.kLimelightForwardOffset, 
    LimelightConstants.kLimelightSideOffset, 
    LimelightConstants.kLimelightUpOffset, 
    new Rotation3d(
      LimelightConstants.kLimelightRoll,
      LimelightConstants.kLimelightPitch,
      LimelightConstants.kLimelightYaw
    )
  );
  private Limelight m_camera = new Limelight(LimelightConstants.limelightName, m_cameraPoseRobotSpace);
  private TunableBoolean m_limelightToggleTunable = Tunables.addBoolean("Odometry/Use Limelight pose", true);

  private final Field2d m_field; // simulation section
  private final FieldObject2d m_frontLeftObject2d;
  private final FieldObject2d m_frontRighObject2d;
  private final FieldObject2d m_backLeftObject2d;
  private final FieldObject2d m_backRightObject2d;
  private Rotation2d m_simHeading = new Rotation2d(); 
  private ChassisVelocities m_lastRobotRelativeSpeeds = new ChassisVelocities();
  private TunableBoolean m_simToggleTunable = Tunables.addBoolean("Simulation/(Simulation Only) Blue Alliance", true);

  private ProfiledPIDController m_autoAlignPID; // auto-alignment section
  private ProfiledPIDController m_autoDistancePID;

  private TunableDouble m_pDriveTunable = Tunables.addDouble("PID Constants/Drive P", SwerveConstants.kDriveP); // extraneous section
  private TunableDouble m_vDriveTunable = Tunables.addDouble("PID Constants/Drive V (feed forward)", SwerveConstants.kDriveV);
  private TunableDouble m_pTurnTunable = Tunables.addDouble("PID Constants/Turn P", SwerveConstants.kTurnP);
  private TunableDouble m_dTurnTunable = Tunables.addDouble("PID Constants/Turn D", SwerveConstants.kTurnD);
  private TunableDouble m_pAutoAlignTunable = Tunables.addDouble("PID Constants/Auto-align P", SwerveConstants.kAutoAlignP);
  private TunableDouble m_iAutoAlignTunable = Tunables.addDouble("PID Constants/Auto-align I", SwerveConstants.kAutoAlignI);
  private TunableDouble m_dAutoAlignTunable = Tunables.addDouble("PID Constants/Auto-align D", SwerveConstants.kAutoAlignD);
  private TunableDouble m_pAutoDistanceTunable = Tunables.addDouble("PID Constants/Auto-distance P", SwerveConstants.kAutoDistanceP);
  private TunableDouble m_iAutoDistanceTunable = Tunables.addDouble("PID Constants/Auto-distance I", SwerveConstants.kAutoDistanceI);
  private TunableDouble m_dAutoDistanceTunable = Tunables.addDouble("PID Constants/Auto-distance D", SwerveConstants.kAutoDistanceD);

  //#endregion






  /* ---------------------------------------------------- Basic ----------------------------------------------------
  * The minimum required methods for the drivebase to function
  * Works by adjusting the state of each swerve module. The states are calculated through inverse kinematics
  * Desaturating wheel speeds makes sure they are below the max attainable speed 
  * getRawGyroHeading() is used by drive(). Otherwise, it would've eneded up in the odometry or auto-alignment section
  * This class is a singleton because it makes sense - I mean, there's only one drivebase per robot anyway
  * There are three versions of a method that returns a heading (not overloaded) because of a severe bug when an auto was canceled prematurely (and then another one after that)
    * If the robot was facing a certain direction during an auto and the robot was disabled, driving offset in that direction
    * The auto-align commands worked which proved that odometry still worked
      * So a odometry-relative heading method was created and is used in the getLimelightPose() method and drive() (sort of)
    * After simulating the change, a new bug arose where driving is always blue alliance relative
      * getDriverHeading() was created to fix this bug (just adds 180 degrees the field relative heading if its the red alliance)
      * Used in only drive()
    * There is another severe bug where the robot will continue to move if it was during the auto before it was disconnected
      *(explained more in the pathplanner section)
  * getRawGyroHeading() = pose estimation input, getFieldHeading() = pose, auto-alignment, Limelight yaw, pathplanner pose, getDriverHeading() = real driver input
  */
  //#region

  private static class SwerveSingleton {
    private static final SwerveDrivebase instance = new SwerveDrivebase();
  }

  /** Creates the swerve drivebase and initializes its modules, kinematics, and telemetry. */
  private SwerveDrivebase() {
    m_gyro.resetYaw();

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

    m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics, 
      getRawGyroHeading(), 
      getModulePositions(), 
      new Pose2d()
    );
    
    m_camera
      .withPoseEstimateConfig_MT2(SwerveConfigs.mt2Config)
      .withTelemetry(true);
    
    m_field = new Field2d();
    m_frontLeftObject2d = m_field.getObject("Front Left Module");
    m_frontRighObject2d = m_field.getObject("Front Right Module");
    m_backLeftObject2d = m_field.getObject("Back Left Module");
    m_backRightObject2d = m_field.getObject("Back Right Module");

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

    setupPathPlanner();
  }

  /** Returns the instance of the SwerveDrivebase singleton */
  public static SwerveDrivebase getInstance() {
    return SwerveSingleton.instance;
  }

  /** Drives the robot with the given chassis speeds, optionally relative to the field. */
  public void drive(double vx, double vy, double omega, boolean fieldRelative) { 
    SwerveModuleVelocity[] m_swerveModuleStates;
    if(fieldRelative) {
      m_swerveModuleStates = m_kinematics.toSwerveModuleVelocities(
        //ChassisVelocities.fromFieldRelativeSpeeds(vx, vy, omega, getDriverHeading())
        new ChassisVelocities(vx, vy, omega).toRobotRelative(getDriverHeading())
      );
    } else {
      m_swerveModuleStates = m_kinematics.toSwerveModuleVelocities(new ChassisVelocities(vx, vy, omega));
    }

    m_swerveModuleStates = SwerveDriveKinematics.desaturateWheelVelocities(
      m_swerveModuleStates, SwerveConstants.kMaxMetersPerSecond);
    m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
    m_frontRight.setDesiredState(m_swerveModuleStates[1]);
    m_backLeft.setDesiredState(m_swerveModuleStates[2]);
    m_backRight.setDesiredState(m_swerveModuleStates[3]);

    m_lastRobotRelativeSpeeds = m_kinematics.toChassisVelocities(m_swerveModuleStates);
  }

  /** Returns the current robot heading based on the raw gyro value. */
  public Rotation2d getRawGyroHeading() { 
    return Rotation2d.fromRadians(m_gyro.getYawRadians());
  }

  /** Returns the current robot heading based on the field and odometry. */
  public Rotation2d getFieldHeading() {
    // does not need the simulation conditional because it will not be used in simulation, but its counterpart above will
    return getPose().getRotation();
  }

  /** Returns the current robot heading based on odometry and the current aliance. */
  public Rotation2d getDriverHeading() {
    Rotation2d heading = getFieldHeading();

    if (shouldFlipPathForAlliance()) return heading.plus(Rotation2d.fromDegrees(180));
    return heading;
  }

  /** Updates Limelight data, odometry, and the dashboard field display each scheduler loop. */
  @Override
  public void periodic() { 
    tunePIDControllers();
    
    refreshOdometryStatusSignals();
    m_poseEstimator.updateWithTime(
      getOdometryTimestampAvgSeconds(),
      getRawGyroHeading(),
      getModulePositions()
    );

    boolean useVisionPoseToggle = m_limelightToggleTunable.get(); 
    boolean isValidYawRate = Math.abs(m_gyro.getGyroRateZ()) <= LimelightConstants.kMaxViableGyroRate;

    if (useVisionPoseToggle && RobotBase.isReal() && isValidYawRate) {
      Limelight.setSharedRobotOrientation(getFieldHeading().getDegrees()); 
      for (var estimate : m_camera.readAcceptedPoseEstimates(PoseEstimateType.MT2_WPIBLUE)) {
        m_poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
      }
    }

    m_camera.getLatestResults();
    
    if (m_camera.hasTarget()) {
      Telemetry.log("LimelightData/Camera TX", m_camera.getTXDegrees());
      Telemetry.log("LimelightData/Camera TY", m_camera.getTYDegrees());
    }

    updateSimField();

  }

  //#endregion






  /* --------------------------------------------------- Odometry ---------------------------------------------------
  * Calculates an estimated position of the robot on the field given field data from encoders, the gyro, and Limelight/vision
  * The readings are given through periodic()
  * Standard deviation is used to tell the pose estimator/odometry how much to trust a sensor. 
  * The trust on vision is dynamic through # of tags, distance, etc.
  * Pathplanner is very important for odometry as it returns the field data for auto after it's done
  * StatusSignal<T>s are used to return module data over the motors returning them directly for tracking the timestamp between updates
    * They cache their data as well for less memory usage (I think) instead of repeatedly getting data from the motors
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

  /** Returns a supplier that supplies the pose of the robot */
  public Supplier<Pose2d> getPoseSupplier() {
    return () -> {
      return getPose();
    };
  }

  /** Resets drive encoder distance and starts odometry from the given pose. */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getRawGyroHeading(), getModulePositions(), pose);
  }

  /** Refreshes all StatusSignal<T>s accross all swerve modules */
  private void refreshOdometryStatusSignals() {
    m_frontLeft.refreshOdometryStatusSignals();
    m_frontRight.refreshOdometryStatusSignals();
    m_backLeft.refreshOdometryStatusSignals();
    m_backRight.refreshOdometryStatusSignals();
  }

  /** Calculates the average timestamp between updates from all modules */
  private double getOdometryTimestampAvgSeconds() {
    return (
      m_frontLeft.getOdometryTimestampSeconds()
      + m_frontRight.getOdometryTimestampSeconds()
      + m_backLeft.getOdometryTimestampSeconds()
      + m_backRight.getOdometryTimestampSeconds()
    ) / 4.0;
  }

  //#endregion





  
  /* -------------------------------------------------- Simulation --------------------------------------------------
  * As said in the module's description for its job during simulation, each module holds "fake" values of data - turn position, distance, and velocity
  * The false data of the gyroscope is the simulation heading
  * The Field2d is used for both the actual robot movement out of simulation and the simulated movement during simulation
  * The best way to think of simulation is the ideal movement of the robot from input
  * Helped me to fix a lot of bugs lol
  */
  //#region

  /** Advances the simulated heading and module positions each simulation loop. */
  @Override
  public void simulationPeriodic() {
    double delay = 0.02;
    
    m_simHeading = m_simHeading.plus(Rotation2d.fromRadians(m_lastRobotRelativeSpeeds.omega * delay));
    OnboardIMUSim.setYaw(m_simHeading.getRadians());

    m_frontLeft.updateSim(delay);
    m_frontRight.updateSim(delay);
    m_backLeft.updateSim(delay);
    m_backRight.updateSim(delay);
  }

  /** Lets me flip the alliance the autos are based on dynamically in simulation but still works for real use */
  public boolean shouldFlipPathForAlliance() {
    if (RobotBase.isSimulation()) 
      return m_simToggleTunable.get(); 
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
  }

  /** Updates the simulation field (should be called after pose estimation is updated) */
  private void updateSimField() {
    Pose2d robotPose = getPose();

    m_field.setRobotPose(robotPose);

    m_frontLeftObject2d.setPose(robotPose.transformBy(
      new Transform2d(
        new Translation2d(SwerveConstants.kWheelBase / 2,  SwerveConstants.kTrackWidth / 2), 
        m_frontLeft.getState().angle
      )
    ));
    m_frontRighObject2d.setPose(robotPose.transformBy(
      new Transform2d(
        new Translation2d(SwerveConstants.kWheelBase / 2,  -SwerveConstants.kTrackWidth / 2), 
        m_frontRight.getState().angle
      )
    ));
    m_backLeftObject2d.setPose(robotPose.transformBy(
      new Transform2d(
        new Translation2d(-SwerveConstants.kWheelBase / 2,  SwerveConstants.kTrackWidth / 2), 
        m_backLeft.getState().angle
      )
    ));
    m_backRightObject2d.setPose(robotPose.transformBy(
      new Transform2d(
        new Translation2d(-SwerveConstants.kWheelBase / 2,  -SwerveConstants.kTrackWidth / 2), 
        m_backRight.getState().angle
      )
    ));

    Telemetry.log("Field", m_field);
  }

  //#endregion





  
  /* ------------------------------------------------ Auto-alignment ------------------------------------------------
  * Reducing the load on the drivers is very important during the game!!!! This means assisting movement through code
  * Auto-alignment makes it easier for a driver to move to a certain destination
  * Auto-alignment can be based on odometry or Limelight vision
  * Using a profiled PID is better than a PID, because of the relationship between error and distance; farther = more aggresive on the motors = bad
  */
  //#region

  /** Resets the auto-align PID for the commands */
  public void setupAutoAlignPID(Translation2d m_targetPoint) {
    Pose2d position = getPose();
    Rotation2d targetHeading = m_targetPoint.minus(position.getTranslation()).getAngle().orElse(new Rotation2d()); // simple fix for Optational<Rotation2d> change
    m_autoAlignPID.reset(position.getRotation().getRadians());
    m_autoAlignPID.setGoal(targetHeading.getRadians()); // this does not need to be run constantly when stop is false (tested)
  }

  /** Calculates the PID output of the current angle to the targeted angle of the robot (should be used as omega input) */
  public double getAutoAlignOutput(Translation2d m_targetPoint, boolean asControllerInput) {
    Pose2d position = getPose();
    Translation2d fieldLocation = position.getTranslation();
    Translation2d vectorBetweenPoints = m_targetPoint.minus(fieldLocation);
    Rotation2d dynamicHeading = vectorBetweenPoints.getAngle().orElse(new Rotation2d()); // simple fix for Optational<Rotation2d> change

    // Second part - convert dynamic heading to be suitable with the drive method
    Rotation2d currentHeading = position.getRotation();
    double pidOutput = m_autoAlignPID.calculate(currentHeading.getRadians(), dynamicHeading.getRadians());
    pidOutput = Math.clamp(pidOutput, -SwerveConstants.kMaxAngularSpeed, SwerveConstants.kMaxAngularSpeed);
    
    // Third part- check if distance from target is valid (to protect from auto-align freaking out)
    double distance = m_targetPoint.getDistance(fieldLocation);
    if (distance < 0.5) return 0.0;
    
    // Fourth part - normalize PID output between 0.0 - 1.0 to act as a controller if necessary
    if (asControllerInput) return -(pidOutput / SwerveConstants.kMaxAngularSpeed); // normalize and flip for getDriveCommand in RobotContainer
    return pidOutput;
    
  }

  /** Checks if the auto-align command is aligned */
  public boolean isAutoAligned() {
    return m_autoAlignPID.atGoal();
  }

  /** Resets the auto-distance PID for the commands */
  public void setupAutoDistancePID(Translation2d m_targetPoint) {
    m_autoDistancePID.reset(m_targetPoint.getDistance(getPose().getTranslation()));
    m_autoDistancePID.setGoal(SwerveConstants.kAutoDistanceTarget);
  }

  /** Calculates the PID output of the current distance to the targeted distance off a certain Translation2d of the robot (should be used as x input, robot relative) */
  public double getAutoDistanceOutput(Translation2d m_targetPoint, boolean asControllerInput) {
    // First part - calculate distance between robot and target
    Pose2d position = getPose();
    Translation2d fieldLocation = position.getTranslation();
    double distance = m_targetPoint.getDistance(fieldLocation);

    // Second part - feed distance into PID
    double pidOutput = m_autoDistancePID.calculate(distance, SwerveConstants.kAutoDistanceTarget);
    pidOutput = Math.clamp(pidOutput, -SwerveConstants.kMaxMetersPerSecond, SwerveConstants.kMaxMetersPerSecond);
    pidOutput = -pidOutput; // negative for some reason i dont remember
    
    // Third part - normalize PID output between 0.0 - 1.0 to act as a controller if necessary
    if (asControllerInput) return -(pidOutput / SwerveConstants.kMaxMetersPerSecond);
    return pidOutput; // negative for some reason i dont remember
  }

  //#endregion





  
  /* -------------------------------------------------- Pathplanner/Autonomous -------------------------------------------------
  * Pathplanner is used to prevent hard-coding autos and makes it much easier on programming
  * I'm gonna be so fr, a lot of the Pathplanner code was copied from a guide ty YASS (Yet Another Software Suite)
  * A lot of Pathplanner code is setting up something called the AutoBuilder which converts paths in the GUI into commands
  * NamedCommands are used to bridge Pathplanner GUI to the rest of the code so other subsystems or commands can be triggered by it
  * Yes there is another driving method because Pathplanner needs it. The difference with this one is that it's robot relative and not field relative
  * We rely on Pathplanner for the initial pose/pose after auto inside of the pose estimator
  * Another method for fetching the heading was created because of a bug that happens during an autonomous that is disabled abruptly
    * (explained more in the basic section)
  * stopAutonomousDrive() was created to another autonomous bug
    * Happens in the same circumstances but where the robot will continue to move if it was in the auto before disconnection
    * It is placed in the Robot class in autonomousExit() to ensure the robot stops when autonomous is exited in any way
  */
  //#region
  
  /** Drives the robot using robot-relative chassis speeds, as required by PathPlanner. */
  private void driveRobotRelative(ChassisVelocities speedsRobotRelative) {
    SwerveModuleVelocity[] moduleStates = m_kinematics.toSwerveModuleVelocities(speedsRobotRelative);

    moduleStates = SwerveDriveKinematics.desaturateWheelVelocities(
      moduleStates,
      SwerveConstants.kMaxMetersPerSecond
    );

    m_frontLeft.setDesiredState(moduleStates[0]);
    m_frontRight.setDesiredState(moduleStates[1]);
    m_backLeft.setDesiredState(moduleStates[2]);
    m_backRight.setDesiredState(moduleStates[3]);

    m_lastRobotRelativeSpeeds = m_kinematics.toChassisVelocities(moduleStates);
  }

  /** Returns the robot-relative chassis speed used by PathPlanner. */
  private ChassisVelocities getRobotVelocity() {
    return m_kinematics.toChassisVelocities(
      new SwerveModuleVelocity[] {
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
          // ChassisVelocities supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> driveRobotRelative(speedsRobotRelative),
          // Method that will drive the robot given ROBOT RELATIVE ChassisVelocities. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(SwerveConstants.kTranslationP, SwerveConstants.kTranslationI, SwerveConstants.kTranslationD),
              // Translation PID constants
              new PIDConstants(SwerveConstants.kRotationP, SwerveConstants.kRotationI, SwerveConstants.kRotationD)
              // Rotation PID constants
          ),
          config,
          this::shouldFlipPathForAlliance,
          this
          // Reference to this subsystem to set requirements
                           );
      m_pathPlannerAvailable = true;

    } catch (ExceptionInInitializerError e) {
      // PathPlanner can throw this Error while its static RobotConfig Alerts are
      // created. Do not let an optional autonomous library prevent teleop from
      // starting; RobotContainer will omit PathPlanner autos in this state.
      DriverStationErrors.reportError(
          "PathPlanner failed to initialize; autonomous paths are disabled. "
              + "Update the matching PathPlanner/WPILib alpha releases. Cause: "
              + e.getException(),
          false);
      return;
    } catch (Exception e) {
      DriverStationErrors.reportError(
          "PathPlanner failed to initialize; autonomous paths are disabled. Cause: " + e,
          false);
      return;
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  /** Whether PathPlanner initialized successfully and PathPlanner autos may be created. */
  public boolean isPathPlannerAvailable() {
    return m_pathPlannerAvailable;
  }

  /** Return Pathplanner auto command */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /** Resets odometry based on the starting position of a Pathplanner auto. Useful for when the robot cannot move during auto but still requires a pose. */
  public Command resetPoseFromAuto(String autoName) {
    PathPlannerAuto sourceAuto = new PathPlannerAuto(autoName);

    return Commands.runOnce(() -> {
      Pose2d startingPose = sourceAuto.getStartingPose();

      if (startingPose == null) {
        DriverStationErrors.reportError(
          "PathPlanner auto \"" + autoName + "\" does not have a starting pose.",
          false
        );
        return;
      }

      if (shouldFlipPathForAlliance()) {
        startingPose = FlippingUtil.flipFieldPose(startingPose);
      }

      resetOdometry(startingPose);
    }, this);
}

/** Optional overload that resets odometry based off a given pose. Useful for when the robot cannot move but still requires a pose */
public Command resetPoseFromAuto(Pose2d poseBlueRelative) {
  return Commands.runOnce(() -> {
    if (shouldFlipPathForAlliance()) {
      resetOdometry(FlippingUtil.flipFieldPose(poseBlueRelative));
      return;
    }
    
    resetOdometry(poseBlueRelative);
  }, this);
}

  /** Will stop the robot if an auto is stopped prematurely. (To prevent a bug) SHOULD BE CALLED IN ROBOT.JAVA */
  public void stopAutonomousDrive() {
    driveRobotRelative(new ChassisVelocities());
  }

  //#endregion





  
  /* -------------------------------------------------- Extraneous --------------------------------------------------
  * These methods were also leftover from when I started to modify the REBUILT drivebase code
  * tunePIDControllers() is the golden child in this section as it lets us tune PID dynamically during operation without having to recompile the code 
  */
  //#region

  /** Dynamically tune the PID controllers of the drivebase */
  public void tunePIDControllers () {
    double driveP = m_pDriveTunable.get(); 
    double driveV = m_vDriveTunable.get(); 
    double turnP = m_pTurnTunable.get(); 
    double turnD = m_dTurnTunable.get(); 
    double autoAlignP = m_pAutoAlignTunable.get(); 
    double autoAilgnI = m_iAutoAlignTunable.get(); 
    double autoAlignD = m_dAutoAlignTunable.get(); 
    double autoDistanceP = m_pAutoDistanceTunable.get(); 
    double autoDistanceI = m_iAutoDistanceTunable.get(); 
    double autoDistanceD = m_dAutoDistanceTunable.get(); 

    if (m_pDriveTunable.hasChanged() || m_vDriveTunable.hasChanged()) { 
      m_frontLeft.configDrivePID(driveP, driveV);
      m_frontRight.configDrivePID(driveP, driveV);
      m_backLeft.configDrivePID(driveP, driveV);
      m_backRight.configDrivePID(driveP, driveV);
    }

    if (m_pTurnTunable.hasChanged() || m_dTurnTunable.hasChanged()) { 
      m_frontLeft.configTurnPID(turnP, turnD);
      m_frontRight.configTurnPID(turnP, turnD);
      m_backLeft.configTurnPID(turnP, turnD);
      m_backRight.configTurnPID(turnP, turnD);
    }

    if (m_pAutoAlignTunable.hasChanged() || m_iAutoAlignTunable.hasChanged() || m_dAutoAlignTunable.hasChanged()) { 
      m_autoAlignPID.setP(autoAlignP);
      m_autoAlignPID.setI(autoAilgnI);
      m_autoAlignPID.setD(autoAlignD);
    }

    if (m_pAutoDistanceTunable.hasChanged() || m_iAutoDistanceTunable.hasChanged() || m_dAutoDistanceTunable.hasChanged()) { 
      m_autoDistancePID.setP(autoDistanceP);
      m_autoDistancePID.setI(autoDistanceI);
      m_autoDistancePID.setD(autoDistanceD);
    }
  }

  /** Logs gyro and module telemetry to SmartDashboard. */
  public void logData() {
    Telemetry.log("gyro", getRawGyroHeading().getDegrees());
    m_frontLeft.logData("Front Left");
    m_frontRight.logData("Front Right");
    m_backLeft.logData("Back Left");
    m_backRight.logData("Back Right");
  }

  /** Stops all four swerve modules. */
  public void stopModules() { 
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  /** Exposes the camera for other subsystems to use */
  public Limelight getLimelightCamera() {
    return m_camera;
  }

  //#endregion
  
}

class SwerveModule {

    // ---------------------------------------------------- Fields ----------------------------------------------------
    //#region
    private TalonFX m_driveMotor; // basic section
    private TalonFX m_turnMotor; 
    private CANcoder m_CANcoder;
    private VelocityVoltage m_driveRequest;
    private PositionVoltage m_turnRequest;
    private SwerveModuleVelocity m_desiredState = new SwerveModuleVelocity();

    private StatusSignal<Angle> m_drivePositionSignal; // odometry section
    private StatusSignal<AngularVelocity> m_driveVelocitySignal;
    private StatusSignal<Angle> m_turnPositionSignal;

    private double m_simDriveDistanceMeters = 0.0; // simulation section
    private double m_simDriveVelocityMetersPerSecond = 0.0;
    private Rotation2d m_simTurnPosition = new Rotation2d();

    //#endregion






    /* ---------------------------------------------------- Basic ----------------------------------------------------
    * The basic methods needed for the drivebase to work
    * The drivebase controls each module by setting a desired state for it to follow 
    * Each swerve module actually uses both an absolute encoder and relative encoder 
        * The absolute encoder is used as the known origin for the relative encoder
        * The relative encoder is the main encoder as it works faster and better for when the robot is driving
    */
    //#region

    /** Creates a swerve module and applies the starting hardware configuration. */
    public SwerveModule(int driveMotorPort, int turnMotorPort, int CANCoderPort, double magnetOffset) {
        m_driveMotor = new TalonFX(driveMotorPort, new CANBus(SwerveConstants.kCANBus));
        m_turnMotor  = new TalonFX(turnMotorPort, new CANBus(SwerveConstants.kCANBus));
        m_CANcoder = new CANcoder(CANCoderPort, new CANBus(SwerveConstants.kCANBus));

        m_driveRequest = new VelocityVoltage(0);
        m_turnRequest = new PositionVoltage(0);

        configMotors(CANCoderPort);
        zeroDriveEncoder();
        // calibrateTurnEncoder();
        configMagnets(-magnetOffset);

        m_drivePositionSignal = m_driveMotor.getPosition();
        m_driveVelocitySignal = m_driveMotor.getVelocity();
        m_turnPositionSignal = m_turnMotor.getPosition();
    }

    /** Sets the target wheel speed and angle for this module. */
    public void setDesiredState(SwerveModuleVelocity targetState) {
        Rotation2d currentRotation = Rotation2d.fromRotations(getTurnEncoderValueRotations());
        SwerveModuleVelocity optimizedTargetState = targetState.optimize(currentRotation);

        m_driveMotor.setControl(m_driveRequest.withVelocity(optimizedTargetState.velocity));
        m_turnMotor.setControl(m_turnRequest.withPosition(optimizedTargetState.angle.getRotations()));

        m_desiredState = optimizedTargetState; 
    }

    /** Applies the absolute encoder magnet offset in rotations. */
    private void configMagnets(double kCANCoderMagnetOffset) {
        m_CANcoder.getConfigurator().apply(SwerveConfigs.magnetConfigs.withMagnetOffset(kCANCoderMagnetOffset));
    }

    /** Applies the drive and turn motor configurations, including the remote CANcoder feedback ID. */
    private void configMotors(int CANCoderID) {
        m_driveMotor.getConfigurator().apply(SwerveConfigs.driveConfigs);
        m_turnMotor.getConfigurator().apply(SwerveConfigs.turnConfigs);
        m_turnMotor.getConfigurator().apply(SwerveConfigs.turnConfigs.Feedback.withFeedbackRemoteSensorID(CANCoderID));
    }

    /** Returns the current turn encoder position in rotations. */
    private double getTurnEncoderValueRotations() {
        if (RobotBase.isSimulation()) return m_simTurnPosition.getRotations();
        return m_turnPositionSignal.getValueAsDouble();
    }

    /** Resets the drive encoder distance to zero. */
    public void zeroDriveEncoder() {
        if (RobotBase.isSimulation()) m_simDriveDistanceMeters = 0.0;
        m_driveMotor.setPosition(0);
    }

    //#endregion






    /* --------------------------------------------------- Odometry ---------------------------------------------------
    * Most of the explanation and actual odometry/pose estimation is in SwerveDrivebase
    * The purpose of these methods are to return the data from each module so it can be fed into the pose estimator
    * StatusSignal<T>s are used to return module data over the motors returning them directly for tracking the timestamp between updates
        * They cache their data as well for less memory usage (I think) instead of repeatedly getting data from the motors
    */
    //#region

    /** Returns the current drive velocity in meters per second. */
    private double getDriveVelocityMetersPerSecond() {
        if (RobotBase.isSimulation()) return m_simDriveVelocityMetersPerSecond;
        return m_driveVelocitySignal.getValueAsDouble();
    }

    /** Returns the total drive distance traveled in meters. */
    public double getDriveDistance() {
        if (RobotBase.isSimulation()) return m_simDriveDistanceMeters;
        return m_drivePositionSignal.getValueAsDouble();
    }

    /** Returns the module position used for swerve odometry. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRotations(getTurnEncoderValueRotations()));
    }

    /** Returns the module state used for kinematics and telemetry. */
    public SwerveModuleVelocity getState() {
        return new SwerveModuleVelocity(getDriveVelocityMetersPerSecond(), Rotation2d.fromRotations(getTurnEncoderValueRotations()));
    }

    /** Updates all of the status signals in a singular method. */
    public void refreshOdometryStatusSignals() {
        BaseStatusSignal.refreshAll(
            m_drivePositionSignal,
            m_driveVelocitySignal,
            m_turnPositionSignal
        );
    }

    /** Returns the timstamp of the drive position status signal. */
    public double getOdometryTimestampSeconds() {
        return m_drivePositionSignal.getTimestamp().getTime();
    }

    //#endregion






    /* -------------------------------------------------- Simulation --------------------------------------------------
    * Simulatting the robot means keeping track of fake/not real information of each swerve module, including turn position, distance, and velocity
    * A few methods may start with a conditional that checks if simulation is active, and then will return the "fake" value if so
    * updateSim() is the main method for updating the all of the values based on the desired positioning of the module
    * The best way to think of simulation is that it is the ideal movement based on the fact that it uses the desired position object (and represents it)
    */
    //#region

    /** Updates the simulated module position and velocity over the given loop delay. */
    public void updateSim(double delay) {
        m_simDriveVelocityMetersPerSecond = m_desiredState.velocity;
        m_simTurnPosition = m_desiredState.angle;
        m_simDriveDistanceMeters += m_simDriveVelocityMetersPerSecond * delay;
    }

    //#endregion






    /* -------------------------------------------------- Extraneous --------------------------------------------------
    * Extra methods leftover from when I started to modify the swerve module code
    * Nothing else really needed to be said
    */
    //#region

    /** Logs key encoder and target values for this module to SmartDashboard. */
    public void logData(String motor){
        Telemetry.log(motor + " CANcoder", m_CANcoder.getAbsolutePosition().getValueAsDouble());
        Telemetry.log(motor + " actual turn position", getTurnEncoderValueRotations());
        Telemetry.log(motor + " desired turn position", m_desiredState.angle.getRotations());
    }

    /** Stops both the drive and turn motors. */
    public void stop() {
        m_driveMotor.setThrottle(0.0);
        m_turnMotor.setThrottle(0.0);
    }

    /** Updates the drive motor PID and feedforward constants. */
    public void configDrivePID(double kP, double kV){
        Slot0Configs m_driveConfig = new Slot0Configs();
        m_driveConfig.kP = kP;
        m_driveConfig.kV = kV;
        m_driveMotor.getConfigurator().apply(m_driveConfig);
    }

    /** Updates the turn motor PID constants. */
    public void configTurnPID(double kP, double kD){ 
        Slot0Configs m_turnConfig = new Slot0Configs();
        m_turnConfig.kP = kP;
        m_turnConfig.kD = kD;
        m_turnMotor.getConfigurator().apply(m_turnConfig);
    }

    //#endregion

}
