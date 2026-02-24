package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Configs.HoodConfigs;

public class Shooter2 extends SubsystemBase {

  //v = (R/cos)(sqrt(((R^2)g)/(2(Rtan+h))))

  private LimelightHelpers m_helpers;

  private TalonFX m_motor;
  private double m_enc;
  
  private HoodConfigs m_configs = new HoodConfigs();
  

  public Shooter2(int motorID) {
    m_motor = new TalonFX(motorID);
    m_motor.getConfigurator().apply(m_configs.kKrakenLeaderConfig);

    m_enc = m_motor.getPosition().getValueAsDouble();

    m_helpers = new LimelightHelpers();
  }

  public double getVelocity(double angle, double R, double h) {
    return (R / Math.cos(angle)) * (Math.sqrt((((R * R * 10) / (2 * (R * Math.tan(angle) + h)))))); //Henry's equation
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
