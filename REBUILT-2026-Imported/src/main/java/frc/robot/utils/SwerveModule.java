
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.utils.Configs.SwerveConfigs;
import frc.robot.utils.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_turnMotor; 
    private CANcoder m_CANcoder;

    private VelocityVoltage m_driveRequest;
    private PositionVoltage m_turnRequest;

    private SwerveModuleVelocity m_desiredState = new SwerveModuleVelocity();


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
    }

    public void configMagnets(double kCANCoderMagnetOffset) {
        m_CANcoder.getConfigurator().apply(SwerveConfigs.magnetConfigs.withMagnetOffset(kCANCoderMagnetOffset));
    }

    public void configMotors(int CANCoderID) {
        m_driveMotor.getConfigurator().apply(SwerveConfigs.driveConfigs);
        m_turnMotor.getConfigurator().apply(SwerveConfigs.turnConfigs);
        m_turnMotor.getConfigurator().apply(SwerveConfigs.turnConfigs.Feedback.withFeedbackRemoteSensorID(CANCoderID));
    }

    public double getDriveDistance() {
        return m_driveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveVelocityMetersPerSecond() {
        return m_driveMotor.getVelocity().getValueAsDouble();
    }

    public double getTurnEncoderValueRotations() {
        return m_turnMotor.getPosition().getValueAsDouble();
    }

    public double getCANCoderPosition() {
        return m_CANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public void logData(String motor){
        SmartDashboard.putNumber(motor + " CANcoder", m_CANcoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber(motor + " actual turn position", getTurnEncoderValueRotations());
        SmartDashboard.putNumber(motor + " desired turn position", m_desiredState.angle.getRotations());
    }

    public SwerveModuleVelocity getState() {
        return new SwerveModuleVelocity(getDriveVelocityMetersPerSecond(), Rotation2d.fromDegrees(getTurnEncoderValueRotations()));
    }

    public void calibrateTurnEncoder() {
        m_turnMotor.setPosition(m_CANcoder.getAbsolutePosition().getValueAsDouble());
    }

    public void zeroDriveEncoder() {
        m_driveMotor.setPosition(0);
    }

    public void resetTurnEncoder() {
        
    }

    public void setDesiredState(SwerveModuleVelocity targetState) {
        Rotation2d currentRotation = Rotation2d.fromRotations(getTurnEncoderValueRotations());
        targetState = targetState.optimize(currentRotation);

        m_driveMotor.setControl(m_driveRequest.withVelocity(targetState.velocity));
        m_turnMotor.setControl(m_turnRequest.withPosition(targetState.angle.getRotations()));

        m_desiredState = targetState; 
    }

    public void setEncoderDistance(double distance) {
        m_driveMotor.setPosition(distance);
    }

    public void stop() {
        m_driveMotor.setThrottle(0);
        m_turnMotor.setThrottle(0);
    }

    public void configDrivePID(double kP, double kV){
          Slot0Configs m_driveConfig = new Slot0Configs();
          m_driveConfig.kP = kP;
          m_driveConfig.kV = kV;
          m_driveMotor.getConfigurator().apply(m_driveConfig);
     }

    public void configTurnPID(double kP, double kD){ 
          Slot0Configs m_turnConfig = new Slot0Configs();
          m_turnConfig.kP = kP;
          m_turnConfig.kD = kD;
          m_turnMotor.getConfigurator().apply(m_turnConfig);
     }


}