
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Configs.SwerveConfigs;
import frc.robot.utils.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_turnMotor; 
    private DutyCycleEncoder m_SRXEncoder;

    private VelocityVoltage m_driveRequest;
    private PositionVoltage m_turnRequest;

    private SwerveModuleState m_desiredState = new SwerveModuleState();


    public SwerveModule(int driveMotorPort, int turnMotorPort, int AbsEncoderPort, double magnetOffset) {
        m_driveMotor = new TalonFX(driveMotorPort);
        m_turnMotor  = new TalonFX(turnMotorPort);
        m_SRXEncoder = new DutyCycleEncoder(AbsEncoderPort, 1, magnetOffset);

        m_driveRequest = new VelocityVoltage(0);
        m_turnRequest = new PositionVoltage(0);

        configMotors();
        zeroDriveEncoder();
        resetTurnEncoder();
    }

    public void configMotors() {
        m_driveMotor.getConfigurator().apply(SwerveConfigs.m_driveConfig);
        m_turnMotor.getConfigurator().apply(SwerveConfigs.m_turnConfig);
    }

    public double getDriveDistance() {
        return m_driveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveVelocityMetersPerSecond() {
        return m_driveMotor.getVelocity().getValueAsDouble();
    }

    public double getTurnEncoderValueDegrees() {
        return m_turnMotor.getPosition().getValueAsDouble();
    }

    public double getAbsoluteEncoderPosition() {
        return m_SRXEncoder.get(); //rotations
    }

    public void logData(String motor){
        SmartDashboard.putNumber(motor + "abs encoder position", m_SRXEncoder.get());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), Rotation2d.fromDegrees(getTurnEncoderValueDegrees()));
    }

    public void zeroTurnEncoder() {
        m_turnMotor.setPosition(0);
    }

    public TalonFX getDriveMotor() {
        return m_driveMotor;
    }

    public void zeroDriveEncoder() {
        m_driveMotor.setPosition(0);
    }

    public void resetTurnEncoder() {
        m_turnMotor.setPosition((m_SRXEncoder.get()) * (SwerveConstants.kAngleConversion));  //degrees
    }

    public void setDesiredState(SwerveModuleState targetState) {
        Rotation2d currentRotation = Rotation2d.fromDegrees(getTurnEncoderValueDegrees());
        targetState.optimize(currentRotation);

        m_driveMotor.setControl(m_driveRequest.withVelocity(targetState.speedMetersPerSecond));
        m_turnMotor.setControl(m_turnRequest.withPosition(targetState.angle.getDegrees()));

        m_desiredState = targetState; 
    }

    public void setEncoderDistance(double distance) {
        m_driveMotor.setPosition(distance);
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turnMotor.set(0);
    }

    public void configDrivePID(double kP, double kD, double kV){
          Slot0Configs m_driveConfig = new Slot0Configs();
          m_driveConfig.kP = kP;
          m_driveConfig.kD = kD;
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