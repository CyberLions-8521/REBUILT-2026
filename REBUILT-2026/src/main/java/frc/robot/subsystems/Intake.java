package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs.IntakeConfigs;
import frc.robot.utils.Constants.CANBusConstants;
import frc.robot.utils.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    
    private TalonFX m_intake;
    private TalonFX m_pivot;

    public Intake(){
        m_intake = new TalonFX(IntakeConstants.kIntakeID, CANBusConstants.subsystemCANBus);
        m_pivot = new TalonFX(IntakeConstants.kPivotID, CANBusConstants.subsystemCANBus);

        m_intake.getConfigurator().apply(IntakeConfigs.kIntakeConfig);
        m_pivot.getConfigurator().apply(IntakeConfigs.kPivotConfig);

        resetPivotEncoders();
    }

    private void logData(){
        SmartDashboard.putNumber("Pivot Position", getPivotPosition());
    }

    public Command getIntakeCommand(double speed) {
        return new RunCommand(() -> setIntakeSpeed(speed));
    }

    public Command setPivotIn() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                setPivotSpeed(0.5);
            },
            interrupted -> setPivotSpeed(0.0),
            () -> m_pivot.getPosition().getValueAsDouble() <= 0.1,
            this);
    }

    public Command setPivotOut() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                setPivotSpeed(-0.7);
            },
            interrupted -> setPivotSpeed(0.0),
            () -> m_pivot.getPosition().getValueAsDouble() >= IntakeConstants.extendedEncoderPosition, //get actual encoder value later
            this);
    }

    public void setPivotSpeed(double speed){
        m_pivot.set(speed);
    }

    public void setIntakeSpeed(double speed){
        m_intake.set(speed);
    }

    public void resetPivotEncoders(){
        m_pivot.setPosition(0);
    }
    
    public double getPivotPosition(){
        return m_pivot.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        logData();
    }
}