package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;

public class Intake extends SubsystemBase {
    
    private TalonFX m_intake;
    private TalonFX m_pivot;

    public Intake(int intakeID, int pivotID){
        m_intake = new TalonFX(intakeID);
        m_pivot = new TalonFX(pivotID);

        m_intake.getConfigurator().apply(IntakeConfigs.kIntakeConfig);
        m_pivot.getConfigurator().apply(IntakeConfigs.kPivotConfig);
    }

    private void logData(){
        SmartDashboard.putNumber("Intake Position", m_intake.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Position", m_intake.getPosition().getValueAsDouble());
    }

    public Command getIntakeCommand(double speed) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                setIntakeSpeed(speed);
            },
            interrupted -> setIntakeSpeed(0.0),
            () -> false,
            this);
    }

    public Command getPivotCommand(double speed) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                setPivotSpeed(speed);
            },
            interrupted -> setPivotSpeed(0.0),
            () -> false,
            this);
    }

    public void setPivotSpeed(double speed){
        m_pivot.set(speed);
    }

    public void setIntakeSpeed(double speed){
        m_intake.set(speed);
    }

    public void resetIntakeEncoders(){
        m_intake.setPosition(0);
    }

    public void resetPivotEncoders(){
        m_pivot.setPosition(0);
    }
    


    @Override
    public void periodic() {
        logData();
    }
}
