package frc.robot;

//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor; 
//import com.revrobotics.spark.config.closedLoop; 
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.PathPlannerConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.auto.HolonomicDriveController;
import com.pathplanner.lib.auto.ReplanningConfig;
import com.pathplanner.lib.config.PIDConstants;



public class Configs {
    public static class SwerveModuleConfigs{
        public static final SparkMaxConfig m_configDrive = new SparkMaxConfig();
        public static final SparkMaxConfig m_configTurn = new SparkMaxConfig();

        static {
            m_configDrive
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(SwerveConstants.driveMotorStallLimit, SwerveConstants.driveMotorFreeLimit);

            m_configTurn
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(SwerveConstants.turnMotorStallLimit, SwerveConstants.turnMotorFreeLimit);

            m_configDrive.encoder
                .positionConversionFactor(SwerveConstants.kDriveConversionFactor)           // meters
                .velocityConversionFactor(SwerveConstants.kDriveConversionFactor / 60.0);   // meters per second

            m_configTurn.encoder
                .positionConversionFactor(SwerveConstants.kTurnConversionFactor)            // degrees
                .velocityConversionFactor(SwerveConstants.kTurnConversionFactor / 60.0);    // degrees per second

            /*
            m_configDrive.closedLoop
                .pidf(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD, SwerveConstants.driveFF)
                .outputRange(-1, 1)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(false); 
                
            m_configTurn.closedLoop
                .pid(SwerveConstants.driveP, SwerveConstants.driveI, SwerveConstants.driveD)
                .outputRange(-1, 1)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-SwerveConstants.kAngleConversion / 2.0, SwerveConstants.kAngleConversion / 2.0); 
        
            */
            
                
        }
    }

    public static class PathPlannerConfigs{
        public static final HolonomicDriveController kPathFollowerConfig = 
            new HolonomicDriveController(
                new PIDConstants(
                        PathPlannerConstants.translationP, 
                        PathPlannerConstants.translationI, 
                        PathPlannerConstants.translationD
                    ),
                new PIDConstants(
                        PathPlannerConstants.rotationP,
                        PathPlannerConstants.rotationI,
                        PathPlannerConstants.rotationD
                    ),
                SwerveConstants.kMaxMetersPerSecond,
                SwerveConstants.kDriveBaseRadius,
                new ReplanningConfig()
            );
    }
}