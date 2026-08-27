
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Configs.SwerveConfigs;
import frc.robot.utils.Constants.SwerveConstants;



public class SwerveModule {

    // ---------------------------------------------------- Fields ----------------------------------------------------
    //#region
    private TalonFX m_driveMotor; // basic section
    private TalonFX m_turnMotor; 
    private CANcoder m_CANcoder;
    private VelocityVoltage m_driveRequest;
    private PositionVoltage m_turnRequest;
    private SwerveModuleState m_desiredState = new SwerveModuleState();

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
        m_driveMotor = new TalonFX(driveMotorPort, SwerveConstants.kCANBus);
        m_turnMotor  = new TalonFX(turnMotorPort, SwerveConstants.kCANBus);
        m_CANcoder = new CANcoder(CANCoderPort, SwerveConstants.kCANBus);

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
    public void setDesiredState(SwerveModuleState targetState) {
        Rotation2d currentRotation = Rotation2d.fromRotations(getTurnEncoderValueRotations());
        targetState.optimize(currentRotation);

        m_driveMotor.setControl(m_driveRequest.withVelocity(targetState.speedMetersPerSecond));
        m_turnMotor.setControl(m_turnRequest.withPosition(targetState.angle.getRotations()));

        m_desiredState = targetState; 
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
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), Rotation2d.fromRotations(getTurnEncoderValueRotations()));
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
        m_simDriveVelocityMetersPerSecond = m_desiredState.speedMetersPerSecond;
        m_simTurnPosition = m_desiredState.angle;
        m_simDriveDistanceMeters += m_simDriveVelocityMetersPerSecond * delay;
    }

    //#endregion






    /* -------------------------------------------------- Extraneous --------------------------------------------------
    * Extra methods leftover from when I started to modify the swerve module code
    * It is not certain whether these are 100%, absolutely necessary in the module class
    * Nothing else really needed to be said
    */
    //#region

    /** Returns the absolute CANcoder position in rotations. */
    private double getCANCoderPosition() {
        if (RobotBase.isSimulation()) return m_simTurnPosition.getRotations();
        return m_CANcoder.getAbsolutePosition().getValueAsDouble();
    }

    /** Logs key encoder and target values for this module to SmartDashboard. */
    public void logData(String motor){
        SmartDashboard.putNumber(motor + " CANcoder", m_CANcoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber(motor + " actual turn position", getTurnEncoderValueRotations());
        SmartDashboard.putNumber(motor + " desired turn position", m_desiredState.angle.getRotations());
    }

    /** Aligns the turn motor encoder with the absolute CANcoder reading. */
    private void calibrateTurnEncoder() {
        m_turnMotor.setPosition(m_CANcoder.getAbsolutePosition().getValueAsDouble());
    }

    /** Sets the drive encoder distance to the given value in meters. */
    public void setEncoderDistance(double distance) {
        if (RobotBase.isSimulation()) m_simDriveDistanceMeters = distance;
        m_driveMotor.setPosition(distance);
    }

    /** Stops both the drive and turn motors. */
    public void stop() {
        m_driveMotor.set(0);
        m_turnMotor.set(0);
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
