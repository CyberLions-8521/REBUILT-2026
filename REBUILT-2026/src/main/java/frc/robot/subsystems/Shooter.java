package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    private final TalonFX m_motorShooterLeader;
    private final TalonFX m_motorShooterFollower;
    private final TalonFX m_motorShooterBottomFollower;
    private final TalonFX m_motorHood;

    private final PositionVoltage m_request = new PositionVoltage(0);

    // debug state
    private double m_debugResolvedAngle = 0.0;
    private double m_debugResolvedVelocity = 0.0;
    private double m_debugHoodError = 0.0;
    private boolean m_debugValidShot = false;
    private boolean m_debugShooterReady = false;

    public Shooter(int motorShooterLeadID, int motorShooterFolID, int motorBottomFolID, int motorHoodID) {
        m_motorShooterLeader = new TalonFX(motorShooterLeadID, "Circus Circle");
        m_motorShooterLeader.getConfigurator().apply(ShooterConfigs.kKrakenLeaderConfig);

        m_motorShooterFollower = new TalonFX(motorShooterFolID, "Circus Circle");
        m_motorShooterFollower.getConfigurator().apply(ShooterConfigs.kKrakenFollowerConfig);
        m_motorShooterFollower.setControl(new Follower(m_motorShooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        m_motorShooterBottomFollower = new TalonFX(motorBottomFolID, "Circus Circle");
        m_motorShooterBottomFollower.getConfigurator().apply(ShooterConfigs.kKrakenFollowerConfig);

        m_motorHood = new TalonFX(motorHoodID, "Circus Circle");
        m_motorHood.getConfigurator().apply(HoodConfigs.kKrakenHoodConfig);

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = ShooterConstants.kHoodP;
        slot0.kI = ShooterConstants.kHoodI;
        slot0.kD = ShooterConstants.kHoodD;
        m_motorHood.getConfigurator().apply(slot0);
        m_motorHood.setPosition(0.0);

        debugInit();
    }

    // -------------------- METHODS --------------------

    public void runShooterMotors(double leaderSpeed) {
        leaderSpeed = MathUtil.clamp(leaderSpeed, 0.0, 0.85);
        m_motorShooterLeader.setControl(new DutyCycleOut(leaderSpeed));
        m_motorShooterBottomFollower.setControl(new DutyCycleOut(leaderSpeed * ShooterConstants.kBottomMotorRatio));
    }

    public void runHoodMotor(double deg) {
        double rotations = MathUtil.clamp(deg * ShooterConstants.kHoodDegsToRot, 0.0, ShooterConstants.kMaxRot);
        SmartDashboard.putNumber("Hood Position (rot) [DESIRED]", rotations);
        double motorHoodPos = m_motorHood.getPosition().getValueAsDouble();
        double feedForwardFlip = (motorHoodPos < rotations) ? 1.0 : -1.5;
        m_motorHood.setControl(m_request.withPosition(rotations).withFeedForward(ShooterConstants.kHoodFeedForward * feedForwardFlip));
    }

    public void stopHood() {
        m_motorHood.setControl(new DutyCycleOut(0.0));
    }

    // -------------------- MATH --------------------

    public double velocityToMotor(double velocity) {
        return (velocity - ShooterConstants.kA) / ShooterConstants.kB;
    }

    public double getVelocity(double angleDegrees, double R) {
        double angleRads = Math.toRadians(angleDegrees);
        double g = ShooterConstants.kGravity;
        double h = ShooterConstants.deltaHeight;
        double denominator = 2 * (R * Math.tan(angleRads) + h);
        if (denominator <= 0) return Double.NaN;
        return (R / Math.cos(angleRads)) * Math.sqrt(g / denominator);
    }

    public double getAngle(double velocity, double R) {
        if (!withinBounds(velocity, R)) return Double.NaN;
        double g = ShooterConstants.kGravity;
        double h = ShooterConstants.deltaHeight;
        double v2 = velocity * velocity;
        double v4 = v2 * v2;
        double discriminant = v4 - g * (g * R * R + 2 * h * v2);
        return Math.toDegrees(Math.atan((v2 + Math.sqrt(discriminant)) / (g * R)));
    }

    public boolean withinBounds(double velocity, double R) {
        double g = ShooterConstants.kGravity;
        double h = ShooterConstants.deltaHeight;
        double v2 = velocity * velocity;
        double v4 = v2 * v2;
        double inner = v4 - 2 * g * h * v2;
        if (inner < 0) return false;
        double maxDist = Math.sqrt(inner) / g;
        double discriminant = v4 - g * (g * R * R + 2 * h * v2);
        return (R <= maxDist) && (discriminant >= 0);
    }

    // -------------------- COMMANDS --------------------

    public Command zeroHood() {
        return new FunctionalCommand(
            () -> {},
            () -> m_motorHood.setPosition(0.0),
            interrupted -> m_motorHood.setPosition(0.0),
            () -> false,
            this
        );
    }

    public Command runHood(double speed) {
        return this.run(() -> m_motorHood.setControl(new DutyCycleOut(speed)));
    }

    public Command setHoodDeg(double deg) {
        return new FunctionalCommand(
            () -> {},
            () -> runHoodMotor(deg),
            interrupted -> stopHood(),
            () -> false,
            this
        );
    }

    /** Returns a {angle, velocity} pair that lands in the 40-70deg window, or null if impossible. */
    private double[] resolveShot(double range) {
        // try default speed first
        double angle = getAngle(ShooterConstants.kDefaultShooterSpeed, range);
        if (!Double.isNaN(angle) && angle > 40 && angle < 70) {
            return new double[]{angle, ShooterConstants.kDefaultShooterSpeed};
        }
        // sweep 7.0 to 9.0 m/s in 0.1 increments
        for (int i = 70; i <= 90; i++) {
            double v = i / 10.0;
            double a = getAngle(v, range);
            if (!Double.isNaN(a) && a > 40 && a < 70) {
                return new double[]{a, v};
            }
        }
        return null;
    }

    public Command shoot(double range) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                double[] shot = resolveShot(range);
                m_debugValidShot = shot != null;
                if (shot != null) {
                    double angle = shot[0];
                    double velocity = shot[1];
                    m_debugResolvedAngle = angle;
                    m_debugResolvedVelocity = velocity;
                    runHoodMotor(angle - ShooterConstants.kHoodLowDegFromHorizontal);
                    double hoodError = Math.abs(
                        m_motorHood.getPosition().getValueAsDouble()
                        - (angle - ShooterConstants.kHoodLowDegFromHorizontal) * ShooterConstants.kHoodDegsToRot
                    );
                    m_debugHoodError = hoodError;
                    m_debugShooterReady = hoodError < 0.05;
                    if (hoodError < 0.05) {
                        runShooterMotors(velocityToMotor(velocity));
                    }
                }
            },
            interrupted -> {
                runShooterMotors(0.0);
                stopHood();
            },
            () -> false,
            this
        );
    }

    // -------------------- DEBUG --------------------

    private void debugInit() {
        SmartDashboard.putNumber("Hood Target Deg", 0.0);
        SmartDashboard.putNumber("Hood Position (rot) [ACTUAL]", 0.0);
        SmartDashboard.putNumber("Hood Position (rot) [DESIRED]", 0.0);
        SmartDashboard.putNumber("Shot Resolved Angle (deg)", 0.0);
        SmartDashboard.putNumber("Shot Resolved Velocity (m/s)", 0.0);
        SmartDashboard.putNumber("Hood Error (rot)", 0.0);
        SmartDashboard.putBoolean("Valid Shot", false);
        SmartDashboard.putBoolean("Shooter Ready", false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position (rot) [ACTUAL]", m_motorHood.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Shot Resolved Angle (deg)", m_debugResolvedAngle);
        SmartDashboard.putNumber("Shot Resolved Velocity (m/s)", m_debugResolvedVelocity);
        SmartDashboard.putNumber("Hood Error (rot)", m_debugHoodError);
        SmartDashboard.putBoolean("Valid Shot", m_debugValidShot);
        SmartDashboard.putBoolean("Shooter Ready", m_debugShooterReady);
    }

    @Override
    public void simulationPeriodic() {}
}