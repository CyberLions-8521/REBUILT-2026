package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.HoodConfigs;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private TalonFX m_motorShooterLeader;
    private TalonFX m_motorShooterFollower;
    private TalonFX m_motorShooterBottomFollower;
    private TalonFX m_motorHood;

    private Slot0Configs slot0 = new Slot0Configs();
    private PositionVoltage m_request;

    public Shooter(int motorShooterLeadID, int motorShooterFolID, int motorBottomFolID, int motorHoodID) {

        m_motorShooterLeader = new TalonFX(motorShooterLeadID, "Circus Circle");
        m_motorShooterLeader.getConfigurator().apply(ShooterConfigs.kKrakenLeaderConfig);

        m_motorShooterFollower = new TalonFX(motorShooterFolID, "Circus Circle");
        m_motorShooterFollower.setControl(
            new Follower(m_motorShooterLeader.getDeviceID(), MotorAlignmentValue.Opposed)
        );

        m_motorShooterBottomFollower = new TalonFX(motorBottomFolID, "Circus Circle");

        m_motorShooterFollower.getConfigurator().apply(ShooterConfigs.kKrakenFollowerConfig);
        m_motorShooterBottomFollower.getConfigurator().apply(ShooterConfigs.kKrakenFollowerConfig);

        m_motorHood = new TalonFX(motorHoodID, "Circus Circle");

        // create a position closed-loop request, voltage output, slot 0 configs     
        
        // set these constants after tuning hood PID
        slot0.kP = ShooterConstants.kHoodP;
        slot0.kI = ShooterConstants.kHoodI;
        slot0.kD = ShooterConstants.kHoodD;
        m_motorHood.getConfigurator().apply(slot0);

        m_request = new PositionVoltage(0);   

        m_motorHood.getConfigurator().apply(HoodConfigs.kKrakenHoodConfig);
        m_motorHood.setPosition(0.0);

        // for testing
        debugInit();
    }

    // -------------------- METHODS --------------------

    public void runShooterMotors(double leaderSpeed) {
        m_motorShooterLeader.setControl(new DutyCycleOut(leaderSpeed));
        double bottomSpeed = leaderSpeed * ShooterConstants.kBottomMotorRatio;
        m_motorShooterBottomFollower.setControl(new DutyCycleOut(bottomSpeed));
    }

    public void runHoodMotor(double deg) {
        double rotations = deg * ShooterConstants.kHoodDegsToRot;
        SmartDashboard.putNumber("Hood Position (rot) [DESIRED]", rotations);
        rotations = MathUtil.clamp(rotations, 0.0, ShooterConstants.kMaxRot);
        double feedForwardFlip = 1;
        double motorHoodPos = m_motorHood.getPosition().getValueAsDouble();
        if(motorHoodPos < rotations){
            feedForwardFlip = 1;
        } else {
            feedForwardFlip = -1.5;
        }
        m_motorHood.setControl(m_request.withPosition(rotations).withFeedForward(ShooterConstants.kHoodFeedForward * feedForwardFlip));
      
    }

    // public Command runHoodMotorButSimple(){
    //     return this.run(() -> m_motorHood.setControl(m_request.withPosition(5)));
    // }

    
    public Command runHood(double speed){
        return this.run(() -> m_motorHood.setControl(new DutyCycleOut(speed)));
    }
  
    // -------------------- MATH --------------------

    public double velocityToMotor(double velocity) {
        return ((velocity + ShooterConstants.kB) / ShooterConstants.kA);
    }

    public double getVelocity(double angleDegrees, double R) {
        double angleRads = Math.toRadians(angleDegrees);
        double g = ShooterConstants.kGravity;
        double h = ShooterConstants.deltaHeight;

        double cosTheta = Math.cos(angleRads);
        double denominator = 2 * (R * Math.tan(angleRads) + h);

        if (denominator <= 0) return Double.NaN;

        return ((R / cosTheta) * Math.sqrt(g / denominator));
    }

    public double getAngle(double velocity, double R) {
        double g = ShooterConstants.kGravity;
        double h = ShooterConstants.deltaHeight;
        double v2 = velocity * velocity;
        double v4 = v2 * v2;

        if (withinBounds(velocity, R)) {
            double discriminant = v4 - g * (g * R * R + 2 * h * v2);
            double root = Math.sqrt(discriminant);
            double sol = Math.atan((v2 + root) / (g * R));
            return Math.toDegrees(sol);
        } else {
            return Double.NaN;
        }
    }

    public boolean withinBounds(double velocity, double R) {
        double g = ShooterConstants.kGravity;
        double h = ShooterConstants.deltaHeight;
        double v2 = velocity * velocity;
        double v4 = v2 * v2;

        double maxDist = Math.sqrt(v4 - 2 * g * h * v2) / g;
        double discriminant = v4 - g * (g * R * R + 2 * h * v2);

        return (R <= maxDist) && (discriminant >= 0);
    }

    // -------------------- COMMANDS --------------------

    public Command zeroHood(){
      return new FunctionalCommand(
            () -> {},
            () -> {
              m_motorHood.setPosition(0.0);
              
            },
            interrupted -> m_motorHood.setPosition(0.0),
            () -> false,
            this
        );
    }

    public Command hoodOnlySmartDashboard(){
      return new FunctionalCommand(
            () -> {},
            () -> {
              double targetDeg = SmartDashboard.getNumber("Hood Target Deg", 0.0);
              runHoodMotor(targetDeg);
            },
            interrupted -> {},
            () -> false,
            this
        );
    }

    public Command shoot(double range) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                double angle = getAngle(ShooterConstants.kDefaultShooterSpeed, range);
                if (!Double.isNaN(angle)) {
                    // try angle
                } else {
                    runShooterMotors(0.0);
                    runHoodMotor(0.0);
                }
            },
            interrupted -> runShooterMotors(0.0),
            () -> false,
            this
        );
    }

    public Command setHood(double pos) {
        return new FunctionalCommand(
            () -> {},
            () -> runHoodMotor(pos),
            interrupted -> runHoodMotor(0.0),
            () -> false,
            this
        );
    }

    public void debugInit(){
        SmartDashboard.putNumber("Hood Target Deg", 0.0);
        SmartDashboard.putNumber("Hood Position (rot) [DESIRED]", 0.0);
        /* 
        SmartDashboard.putNumber("Hood P", 0.0);
        SmartDashboard.putNumber("Hood I", 0.0);
        SmartDashboard.putNumber("Hood D", 0.0);
        SmartDashboard.putNumber("Hood FF", 0.0);
        */
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {}
}