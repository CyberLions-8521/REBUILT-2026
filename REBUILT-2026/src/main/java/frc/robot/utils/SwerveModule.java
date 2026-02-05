
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Configs.SwerveConfigs;
import frc.robot.utils.Constants.SwerveConstants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_turnMotor; 
    private CANcoder m_CANcoder;

    private VelocityVoltage m_drivePID;
    private PositionVoltage m_turnPID;

    private SwerveModuleState m_desiredState = new SwerveModuleState();


    public SwerveModule(int driveMotorPort, int turnMotorPort, int CANCoderPort, double magnetOffset) {
        m_driveMotor = new TalonFX(driveMotorPort);
        m_turnMotor  = new TalonFX(turnMotorPort);
        m_CANcoder = new CANcoder(CANCoderPort, SwerveConstants.kCANCoderBus);

        m_drivePID = new VelocityVoltage(0);
        m_turnPID = new PositionVoltage(0);

        configure(SwerveConfigs.m_driveConfig, SwerveConfigs.m_driveConfig, CANCoderPort);
        resetDriveEncoder();
        configMagnets(magnetOffset);
    }  

    public double getDriveDistance() {
        return m_driveMotor.getPosition().getValueAsDouble();
    }

    public double getTurnEncoderValueDegrees() {
        return m_turnMotor.getPosition().getValueAsDouble();
    }

    public double getDriveVelocityMetersPerSecond() {
        return m_driveMotor.getVelocity().getValueAsDouble();
    }

    public void resetDriveEncoder() {
        m_driveMotor.setPosition(0);
    }

    public void logData(String motor){
        SmartDashboard.putNumber(motor + " turn position", m_turnMotor.getPosition().getValue().baseUnitMagnitude() % 360 - 180);
        SmartDashboard.putNumber(motor + " CANcoder", m_CANcoder.getAbsolutePosition().getValueAsDouble()*SwerveConstants.kAngleConversion);
        SmartDashboard.putNumber(motor + " desired position", m_desiredState.angle.getDegrees());

        SmartDashboard.putNumber(motor + " drive position", m_driveMotor.getPosition().getValue().baseUnitMagnitude());
        SmartDashboard.putNumber(motor + " turn position", m_turnMotor.getPosition().getValue().baseUnitMagnitude());

        SmartDashboard.putNumber(motor + " drive velocity", m_driveMotor.getVelocity().getValue().baseUnitMagnitude());
        SmartDashboard.putNumber(motor + " turn velocity", m_turnMotor.getVelocity().getValue().baseUnitMagnitude());

        // SmartDashboard.putNumber(motor + " drive current", m_driveMotor.getOutputCurrent());
        // SmartDashboard.putNumber(motor + " turn current", m_turnMotor.getOutputCurrent());

        // SmartDashboard.putNumber(motor + " drive voltage", m_driveMotor.getAppliedOutput());
        // SmartDashboard.putNumber(motor + " turn voltage", m_turnMotor.getAppliedOutput());

        // SmartDashboard.putNumber(motor + " drive temp", m_driveMotor.getMotorTemperature());
        // SmartDashboard.putNumber(motor + " turn temp", m_turnMotor.getMotorTemperature());

        // SmartDashboard.putNumber(motor + " drive voltage", m_driveMotor.getBusVoltage());
        // SmartDashboard.putNumber(motor + " turn voltage", m_turnMotor.getBusVoltage());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), Rotation2d.fromDegrees(getTurnEncoderValueDegrees()));
    }
    
    //extra tuning if needed
    public void updateFromDashboard(String motor) {
        double driveSpeed = SmartDashboard.getNumber(motor + " drive speed", 0.0);
        double turnAngle = SmartDashboard.getNumber(motor + " turn angle", 0.0);

        SwerveModuleState targetState = new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(turnAngle));
        setDesiredState(targetState);
    }

    public void zeroTurnEncoder() {
        m_driveMotor.setPosition(0);
    }

    public void zeroDriveEncoder() {
        m_driveMotor.setPosition(0);
    }


    public void configure(TalonFXConfiguration driveConfig, TalonFXConfiguration turnConfig, int CANCoderID) {
        m_driveMotor.getConfigurator().apply(driveConfig);
        m_turnMotor.getConfigurator().apply(turnConfig.Feedback.withFeedbackRemoteSensorID(CANCoderID));
    }

    public void setDesiredState(SwerveModuleState targetState) {
        Rotation2d currentRotation = Rotation2d.fromDegrees(getTurnEncoderValueDegrees());
        targetState.optimize(currentRotation);

        m_driveMotor.setControl(m_drivePID.withVelocity(targetState.speedMetersPerSecond));
        m_turnMotor.setControl(m_turnPID.withPosition(targetState.angle.getDegrees()));

        // m_drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
        // m_turnPID.setReference(targetState.angle.getDegrees(), ControlType.kPosition);

        m_desiredState = targetState; 
    }

    public void setEncoderDistance(double distance) {
        m_driveMotor.setPosition(distance);
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turnMotor.set(0);
    }

    public void turnMotors(double speed, double steer) {
        m_driveMotor.set(speed);
        m_turnMotor.set(steer);
    }

    //for smartdashboard logging purposes
    public double getCANCoderPosition() {
        return m_CANcoder.getAbsolutePosition().getValueAsDouble(); // rotations 
    }

    public void configMagnets(double kCANCoderMagnetOffset) {

        m_CANcoder.getConfigurator().apply(SwerveConfigs.m_magnetConfigs.withMagnetOffset(kCANCoderMagnetOffset));
        
    }

         public void configGains(){
          
          Slot0Configs m_driveConfig = new Slot0Configs();
          Slot0Configs m_turnConfig = new Slot0Configs();
          
          //USE ELASTIC LATER
          m_driveConfig.kP = SmartDashboard.getNumber("Drive P", 0);
          m_driveConfig.kD = SmartDashboard.getNumber("Drive D", 0);
          m_driveConfig.kV = SmartDashboard.getNumber("Drive FF", 0);

          m_turnConfig.kP = SmartDashboard.getNumber("Drive P", 0);
          m_turnConfig.kD = SmartDashboard.getNumber("Drive D", 0);
          m_turnConfig.kV = SmartDashboard.getNumber("Drive FF", 0);


          m_driveMotor.getConfigurator().apply(m_driveConfig);
          m_turnMotor.getConfigurator().apply(m_turnConfig);
     }

}