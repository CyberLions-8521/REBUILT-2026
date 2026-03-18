package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs.IntakeConfigs;
import frc.robot.utils.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    
    private TalonFX m_roller;
    private TalonFX m_pivot;

    public Intake(){
        m_roller = new TalonFX(IntakeConstants.kIntakeID, IntakeConstants.kCanbusName);
        m_pivot = new TalonFX(IntakeConstants.kPivotID, IntakeConstants.kCanbusName);

        m_roller.getConfigurator().apply(IntakeConfigs.rollerConfigs);
        m_pivot.getConfigurator().apply(IntakeConfigs.pivotConfigs);

        resetPivotEncoders();
    }

    private void logData(){
        SmartDashboard.putNumber("Pivot Position", getPivotPosition());
    }

    public Command getResetEncoderPosition() {
        return new InstantCommand(() -> resetPivotEncoders(), this);
    }

    public Command getIntakeCommand(double speed) {
        return new RunCommand(() -> setIntakeSpeed(speed), this);
    }

    public Command getPivotCommand(double speed) {
        return new RunCommand(() -> setPivotSpeed(speed), this);
    }

    public Command setPivotIn() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                setPivotSpeed(0.15);
            },
            interrupted -> setPivotSpeed(0.0),
            () -> m_pivot.getPosition().getValueAsDouble() >= IntakeConstants.kRetractedEncoderPosition,
            this);
    }

    public Command setPivotOut() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                setPivotSpeed(-0.15);
            },
            interrupted -> setPivotSpeed(0.0),
            () -> m_pivot.getPosition().getValueAsDouble() <= IntakeConstants.kExtendedEncoderPosition, //get actual encoder value later
            this);
    }

    public void setPivotSpeed(double speed){
        m_pivot.set(speed);
    }

    public void setIntakeSpeed(double speed){
        m_roller.set(speed);
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