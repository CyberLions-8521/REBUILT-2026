
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Configs.SwerveConfigs;
import frc.robot.utils.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_turnMotor; 
    private CANcoder m_CANcoder;

    private VelocityVoltage m_driveRequest;
    private PositionVoltage m_turnRequest;

    private SwerveModuleState m_desiredState = new SwerveModuleState();


    public SwerveModule(int driveMotorPort, int turnMotorPort, int CANCoderPort, double magnetOffset) {
        m_driveMotor = new TalonFX(driveMotorPort, SwerveConstants.kCANCoderBus);
        m_turnMotor  = new TalonFX(turnMotorPort, SwerveConstants.kCANCoderBus);
        m_CANcoder = new CANcoder(CANCoderPort, SwerveConstants.kCANCoderBus);

        m_driveRequest = new VelocityVoltage(0);
        m_turnRequest = new PositionVoltage(0);

        configMotors(CANCoderPort);
        zeroDriveEncoder();
        // calibrateTurnEncoder();
        configMagnets(-magnetOffset);
    }

    public void configMagnets(double kCANCoderMagnetOffset) {
        m_CANcoder.getConfigurator().apply(SwerveConfigs.m_magnetConfigs.withMagnetOffset(kCANCoderMagnetOffset));
    }

    public void configMotors(int CANCoderID) {
        m_driveMotor.getConfigurator().apply(SwerveConfigs.m_driveConfig);
        m_turnMotor.getConfigurator().apply(SwerveConfigs.m_turnConfig);
        m_turnMotor.getConfigurator().apply(SwerveConfigs.m_turnConfig.Feedback.withFeedbackRemoteSensorID(CANCoderID));
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

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), Rotation2d.fromDegrees(getTurnEncoderValueRotations()));
    }

    public void calibrateTurnEncoder() {
        m_turnMotor.setPosition(m_CANcoder.getAbsolutePosition().getValueAsDouble());
    }

    public void zeroDriveEncoder() {
        m_driveMotor.setPosition(0);
    }

    public void resetTurnEncoder() {
        
    }

    public void setDesiredState(SwerveModuleState targetState) {
        Rotation2d currentRotation = Rotation2d.fromRotations(getTurnEncoderValueRotations());
        targetState.optimize(currentRotation);

        m_driveMotor.setControl(m_driveRequest.withVelocity(targetState.speedMetersPerSecond));
        m_turnMotor.setControl(m_turnRequest.withPosition(targetState.angle.getRotations()));

        m_desiredState = targetState; 
    }

    public void setEncoderDistance(double distance) {
        m_driveMotor.setPosition(distance);
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turnMotor.set(0);
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