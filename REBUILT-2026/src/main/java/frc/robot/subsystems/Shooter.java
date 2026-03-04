package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DifferentialDutyCycle;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Configs.HoodConfigs;
import frc.robot.Configs.ShooterConfigs;

import frc.robot.Constants.ShooterConstants;

import frc.robot.LimelightHelpers;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  private TalonFX m_motorShooterLeader;
<<<<<<< HEAD
  private Follower m_motorShooterFollower;
  private Follower m_motorShooterBottomFollower;
  private TalonFX m_motorHood;

  private ShooterConfigs m_shooterConfigs = new ShooterConfigs();
  private HoodConfigs m_hoodConfigs = new HoodConfigs();
  private Slot0Configs slot0 = new Slot0Configs();

  // c_ for control mode
  final DifferentialDutyCycle c_motorShooterLeader;
  final Follower c_motorShooterFollower;

  public Shooter(int motorShooterLeadID, int motorShooterFolID, int motorBottomFolID, int motorHoodID) {

    c_motorShooterLeader = new DifferentialDutyCycle(0.0, 0);
    c_motorShooterFollower = new Follower(motorShooterLeadID, MotorAlignmentValue.Opposed);

    m_motorShooterLeader = new TalonFX(motorShooterLeadID);
    m_motorShooterLeader.getConfigurator().apply(ShooterConfigs.kKrakenLeaderConfig);

    m_motorShooterFollower = new Follower(motorShooterFolID, MotorAlignmentValue.Opposed);
    m_motorShooterFollower.LeaderID = motorShooterLeadID;

    m_motorShooterBottomFollower = new Follower(motorBottomFolID, MotorAlignmentValue.Opposed);

    m_motorHood = new TalonFX(motorHoodID);

    slot0.kP = 0;
    slot0.kI = 0;
    slot0.kD = 0;

    m_motorHood.getConfigurator().apply(slot0);
    
  }
=======
  private TalonFX m_motorShooterFollower;
  private TalonFX m_motorBottom;
  private TalonFX m_motorHood;
  private Follower followShooter;
  
  public Shooter(int motorShooterLeadID, int motorShooterFolID, int motorBottomID, int motorHoodID) {

    m_motorShooterLeader = new TalonFX(motorShooterLeadID);
    m_motorShooterFollower = new TalonFX(motorShooterFolID);

    followShooter = new Follower(motorShooterLeadID, MotorAlignmentValue.Opposed);
    m_motorShooterFollower.setControl(followShooter.withUpdateFreqHz(50));

    m_motorBottom = new TalonFX(motorBottomID);
    m_motorHood = new TalonFX(motorHoodID);
    
    m_motorShooterLeader.getConfigurator().apply(ShooterConfigs.kKrakenLeaderConfig);
    m_motorShooterFollower.getConfigurator().apply(ShooterConfigs.kKrakenLeaderConfig);
}
>>>>>>> 6445bf703dbf6aebea4e9e0c02df50f586dcf264

  public void runShooterMotors(double leaderSpeed, double bottomSpeed) {
    m_motorShooterLeader.set(leaderSpeed);
    // m_motorShooterBottomFollower.set(bottomSpeed); it's supposed to follow at a lower speed
  }

<<<<<<< HEAD
  // disabled until PID is set up
  // public void runHoodMotor(double deg) {
  //   m_motorHood.setPosition((ShooterConstants.hoodMobilityRatio * deg) /*- insert motor encoder value variable here*/ );
  // }
=======
  public void runHoodMotor(double solDeg) {
    m_motorHood.setPosition((ShooterConstants.hoodMobilityRatio * solDeg) - ShooterConstants.kHoodOffset); // 0.0 right now
  }
>>>>>>> 6445bf703dbf6aebea4e9e0c02df50f586dcf264

  public double velocityToMotor(double velocity){
    return ((velocity + ShooterConstants.kB) / ShooterConstants.kA);
  }

  // for static angle
  public double getVelocity(double angleDegrees, double R) {
    double angleRads = Math.toRadians(angleDegrees);
    double g = ShooterConstants.kGravity;
    double h = ShooterConstants.deltaHeight;
    
    double cosTheta = Math.cos(angleRads);
    double denominator = 2 * (R * Math.tan(angleRads) + h);
    
    if (denominator <= 0) return Double.NaN; 

    return (R / cosTheta) * Math.sqrt(g / denominator);
  }

  public double getAngle(double velocity, double R) {
    double g = ShooterConstants.kGravity;
    double h = ShooterConstants.deltaHeight;
    double v2 = velocity * velocity;
    double v4 = v2 * v2;

    if (withinBounds(velocity, R)) {
        double discriminant = v4 - g * (g * R * R + 2 * h * v2);
        double root = Math.sqrt(discriminant);

        // only high-angle solution
        double sol = Math.atan((v2 + root) / (g * R));
        double deg = Math.toDegrees(sol);
        return deg;
    } else {
        return Double.NaN;
    }
  }

  public boolean withinBounds(double velocity, double R){
    double g = ShooterConstants.kGravity;
    double h = ShooterConstants.deltaHeight;
    double v2 = velocity * velocity;
    double v4 = v2 * v2;

    // max horizontal distance
    double maxDist = Math.sqrt(v4 - 2 * g * h * v2) / g;

    // discriminant for high-angle solution
    double discriminant = v4 - g * (g * R * R + 2 * h * v2);

    return (R <= maxDist) && (discriminant >= 0);
  }

  public Command shoot(double range){
    return new FunctionalCommand(
      () -> {},
      () -> {
<<<<<<< HEAD
        double angle = getAngle(8.0, range);
        if(Double.isNaN(angle)){
          // fallback for when is invalid
          runShooterMotors(0.0, 0.0);
          // runHoodMotor(0.0);
        } else {
        
=======
        double angle = getAngle(ShooterConstants.kDefaultShooterSpeed, LimelightHelpers.getTargetPose3d_RobotSpace("").getX());
        if(!Double.isNaN(angle)){
          // try angle

>>>>>>> 6445bf703dbf6aebea4e9e0c02df50f586dcf264
          /* 
          given required angle:
          set hood motor position to that angle
          (account for offset and bounds)
          */ 



        } else {
          // try velocity

          // fallback for when is invalid
          runShooterMotors(0.0, 0.0);
          runHoodMotor(0.0);

        }
      },
      interrupted -> runShooterMotors(0.0, 0.0),
      () -> false,
      this);
  }

  public Command setHood(double pos) {
    return new FunctionalCommand(
    () -> {},
    () -> {
      runHoodMotor(pos);
    },
    interrupted -> runHoodMotor(0.0),
    () -> false,
    this);
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
