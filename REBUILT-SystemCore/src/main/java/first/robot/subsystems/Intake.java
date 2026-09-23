package first.robot.subsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import first.robot.utils.Configs.IntakeConfigs;
import first.robot.utils.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    
    private TalonFX m_intake;
    private TalonFX m_pivot;

    private VelocityVoltage m_intakeController;
    private PositionVoltage m_pivotController;

    public Intake(){
        m_intake = new TalonFX(IntakeConstants.kIntakeID, new CANBus(IntakeConstants.kCanbusName));
        m_pivot = new TalonFX(IntakeConstants.kPivotID, new CANBus(IntakeConstants.kCanbusName));

        m_intake.getConfigurator().apply(IntakeConfigs.rollerConfigs);
        m_pivot.getConfigurator().apply(IntakeConfigs.pivotConfigs);

        m_intakeController = new VelocityVoltage(0);
        m_pivotController = new PositionVoltage(0);

        resetPivotEncoders();

        Telemetry.log("pivot P", IntakeConstants.pivotP);
        Telemetry.log("pivot D", IntakeConstants.pivotD);
        Telemetry.log("pivot G", IntakeConstants.pivotG);
        Telemetry.log("intake P", IntakeConstants.rollerP);
        Telemetry.log("intake V", IntakeConstants.rollerV);
    }

    private void logData(){
        Telemetry.log("Pivot Position", getPivotPosition());
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
            interrupted -> m_pivot.setThrottle(0), 
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
        var table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        Slot0Configs m_pivotConfig = new Slot0Configs();
        Slot0Configs m_intakeConfig = new Slot0Configs();
        double pivotP = table.getEntry("pivot P").getDouble(IntakeConstants.pivotP);
        double pivotD = table.getEntry("pivot D").getDouble(IntakeConstants.pivotD);
        double pivotG = table.getEntry("intake P").getDouble(IntakeConstants.pivotG);
        double intakeP = table.getEntry("intake P").getDouble(IntakeConstants.rollerP);
        double intakeV = table.getEntry("intake V").getDouble(IntakeConstants.rollerV);

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