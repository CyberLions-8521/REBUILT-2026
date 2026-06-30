
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Configs.SwerveConfigs;
import frc.robot.utils.Constants.SwerveConstants;

/** Represents one swerve module's drive motor, turn motor, and absolute encoder. */
public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_turnMotor; 
    private CANcoder m_CANcoder;

    private VelocityVoltage m_driveRequest;
    private PositionVoltage m_turnRequest;

    private SwerveModuleState m_desiredState = new SwerveModuleState();

    private double m_simDriveDistanceMeters = 0.0;
    private double m_simDriveVelocityMetersPerSecond = 0.0;
    private Rotation2d m_simTurnPosition = new Rotation2d();

    // ---------------------------------------------------- Basic ----------------------------------------------------

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
        return m_turnMotor.getPosition().getValueAsDouble();
    }

    /** Resets the drive encoder distance to zero. */
    public void zeroDriveEncoder() {
        if (RobotBase.isSimulation()) m_simDriveDistanceMeters = 0.0;
        m_driveMotor.setPosition(0);
    }

    // --------------------------------------------------- Odometry ---------------------------------------------------

    /** Returns the current drive velocity in meters per second. */
    private double getDriveVelocityMetersPerSecond() {
        if (RobotBase.isSimulation()) return m_simDriveVelocityMetersPerSecond;
        return m_driveMotor.getVelocity().getValueAsDouble();
    }

    /** Returns the total drive distance traveled in meters. */
    public double getDriveDistance() {
        if (RobotBase.isSimulation()) return m_simDriveDistanceMeters;
        return m_driveMotor.getPosition().getValueAsDouble();
    }

    /** Returns the module position used for swerve odometry. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRotations(getTurnEncoderValueRotations()));
    }

    /** Returns the module state used for kinematics and telemetry. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), Rotation2d.fromRotations(getTurnEncoderValueRotations()));
    }

    // -------------------------------------------------- Simulation --------------------------------------------------

    /** Updates the simulated module position and velocity over the given loop delay. */
    public void updateSim(double delay) {
        m_simDriveVelocityMetersPerSecond = m_desiredState.speedMetersPerSecond;
        m_simTurnPosition = m_desiredState.angle;
        m_simDriveDistanceMeters += m_simDriveVelocityMetersPerSecond * delay;
    }

    // -------------------------------------------------- Extraneous --------------------------------------------------

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

}
