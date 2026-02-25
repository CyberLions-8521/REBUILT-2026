package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Configs.HoodConfigs;
import frc.robot.Configs.ShooterConfigs;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  private TalonFX m_motorShooterLeader;
  private Follower m_motorShooterFollower;
  private TalonFX m_motorBottom;
  private TalonFX m_motorHood;

  private double m_leaderEnc = m_motorShooterLeader.getPosition().getValueAsDouble();

  private ShooterConfigs m_shooterConfigs = new ShooterConfigs();
  private HoodConfigs m_hoodConfigs = new HoodConfigs();
  
  public Shooter(int motorShooterLeadID, int motorShooterFolID, int motorBottomID, int motorHoodID) {

    m_motorShooterLeader = new TalonFX(motorShooterLeadID);
    m_motorShooterFollower = new Follower(motorShooterFolID, MotorAlignmentValue.Opposed);
    m_motorShooterFollower.LeaderID = motorShooterLeadID;
    m_motorBottom = new TalonFX(motorBottomID);
    m_motorHood = new TalonFX(motorHoodID);

    m_motorShooterLeader.getConfigurator().apply(m_shooterConfigs.kKrakenLeaderConfig);
  }

  public void runShooterMotors(double leaderSpeed, double bottomSpeed) {
    m_motorShooterLeader.set(leaderSpeed);
    m_motorBottom.set(bottomSpeed);
  }

  public void runHoodMotor(double speed) {
    m_motorHood.set(speed);
  }

  public double getVelocity(double angle, double R, double h) {
    return (R / Math.cos(angle)) * (Math.sqrt((((R * R * 10) / (2 * (R * Math.tan(angle) + h)))))); //Henry's equation (for static)
  }

  public Command hoodLift(double speed) {
    return new FunctionalCommand(
    () -> {},
    () -> {
      runHoodMotor(speed);
    },
    interrupted -> runHoodMotor(0.0),
    () -> false,
    this);
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
