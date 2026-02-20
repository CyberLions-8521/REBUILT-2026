package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Configs.OutputConfigs;

public class Output extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  private TalonFX m_motorBotLeader;
  private Follower m_motorBotFollower;

  private TalonFX m_motorTopLeader;
  private Follower m_motorTopFollower;

  private double m_botEnc = m_motorBotLeader.getPosition().getValueAsDouble();
  private double m_topEnc = m_motorTopLeader.getPosition().getValueAsDouble();

  private OutputConfigs m_configs = new OutputConfigs();
  
  public Output(int motorBotLeadID, int motorBotFolID, int motorTopLeadID, int motorTopFolID) {
    m_motorBotLeader = new TalonFX(motorBotLeadID);
    m_motorBotFollower = new Follower(motorBotFolID, MotorAlignmentValue.Opposed);
    m_motorTopLeader = new TalonFX(motorTopLeadID);
    m_motorTopFollower = new Follower(motorTopFolID, MotorAlignmentValue.Opposed);

    m_motorBotLeader.getConfigurator().apply(m_configs.kKrakenLeaderConfig);
    m_motorTopLeader.getConfigurator().apply(m_configs.kKrakenLeaderConfig);
  }

  public void runMotors(double speed) {
    m_motorBotLeader.set(speed);
    m_motorTopLeader.set(speed);
  }





  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
