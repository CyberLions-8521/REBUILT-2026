package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
    
    private TalonFX m_intake;
    private TalonFX m_pivot;

    private VelocityVoltage m_intakeController;
    private PositionVoltage m_pivotController;

    public Intake(){
        m_intake = new TalonFX(IntakeConstants.kIntakeID, IntakeConstants.kCanbusName);
        m_pivot = new TalonFX(IntakeConstants.kPivotID, IntakeConstants.kCanbusName);

        m_intake.getConfigurator().apply(IntakeConfigs.kIntakeConfig);
        m_pivot.getConfigurator().apply(IntakeConfigs.kPivotConfig);

        m_intakeController = new VelocityVoltage(0);
        m_pivotController = new PositionVoltage(0);

        resetPivotEncoders();

        SmartDashboard.putNumber("pivot P", IntakeConstants.pivotP);
        SmartDashboard.putNumber("pivot D", IntakeConstants.pivotD);
        SmartDashboard.putNumber("pivot G", IntakeConstants.pivotG);
        SmartDashboard.putNumber("intake P", IntakeConstants.rollerP);
        SmartDashboard.putNumber("intake V", IntakeConstants.rollerV);
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

    public Command setPivotPositionCommand(double position) {
        return new FunctionalCommand(
            () -> {}, 
            () -> {
                setPivotPosition(position);
            }, 
            interrupted -> m_pivot.set(0), 
            () -> m_pivot.getPosition().getValueAsDouble() <= position - 0.1 || m_pivot.getPosition().getValueAsDouble() >= position + 0.1,
            this);
    }

    public void setPivotPosition(double position){
        m_pivot.setControl(m_pivotController.withPosition(position));
    }

    public void setIntakeSpeed(double speed){
        m_intake.setControl(m_intakeController.withVelocity(speed));
    }

    public void resetPivotEncoders(){
        m_pivot.setPosition(0);
    }
    
    public double getPivotPosition(){
        return m_pivot.getPosition().getValueAsDouble();
    }

    public void tunePID() {
        Slot0Configs m_pivotConfig = new Slot0Configs();
        Slot0Configs m_intakeConfig = new Slot0Configs();
        double pivotP = SmartDashboard.getNumber("pivot P", IntakeConstants.pivotP);
        double pivotD = SmartDashboard.getNumber("pivot D", IntakeConstants.pivotD);
        double pivotG = SmartDashboard.getNumber("intake P", IntakeConstants.pivotG);
        double intakeP = SmartDashboard.getNumber("intake P", IntakeConstants.rollerP);
        double intakeV = SmartDashboard.getNumber("intake V", IntakeConstants.rollerV);

        if (pivotP != IntakeConstants.pivotP || pivotD != IntakeConstants.pivotD || pivotG != IntakeConstants.pivotG) { 
            m_pivotConfig.kP = pivotP;
            m_pivotConfig.kV = pivotD;
            m_pivotConfig.kG = pivotG;
            m_pivot.getConfigurator().apply(m_pivotConfig);
            IntakeConstants.pivotP = pivotP;
            IntakeConstants.pivotD = pivotD;
            IntakeConstants.pivotG = pivotG;
        }

        if (intakeP != IntakeConstants.rollerP || intakeV != IntakeConstants.rollerV) {
            m_intakeConfig.kP = intakeP;
            m_intakeConfig.kV = intakeV;
            m_intake.getConfigurator().apply(m_intakeConfig);
            IntakeConstants.rollerP = intakeP;
            IntakeConstants.rollerV = intakeV;
        }

    }

    @Override
    public void periodic() {
        logData();
        tunePID();
    }
}