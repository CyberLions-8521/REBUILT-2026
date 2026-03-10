package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Shooter extends SubsystemBase {

    private final TalonFX m_motorShooterLeader;
    private final TalonFX m_motorShooterFollower;
    private final TalonFX m_motorShooterBottomFollower;

    private final VelocityVoltage m_requestFlywheel = new VelocityVoltage(0).withSlot(1);
    private final VelocityVoltage m_requestFlywheelBottom = new VelocityVoltage(0).withSlot(1);

    private final InterpolatingDoubleTreeMap velocityTable = new InterpolatingDoubleTreeMap();
    private double m_currentRange = 0.0;
    private Pose3d targetPoseRobot;


    //global consts (for readability)
    private static final double g = ShooterConstants.kGravity;
    private static final double h = ShooterConstants.deltaHeight;

    public Shooter(int motorShooterLeadID, int motorShooterFolID, int motorBottomFolID, int motorHoodID) {
        
        // main motors
        m_motorShooterLeader = new TalonFX(motorShooterLeadID, "Ryan");
        m_motorShooterLeader.getConfigurator().apply(ShooterConfigs.kKrakenLeaderConfig);

        m_motorShooterFollower = new TalonFX(motorShooterFolID, "Ryan");
        m_motorShooterFollower.getConfigurator().apply(ShooterConfigs.kKrakenFollowerConfig);
        m_motorShooterFollower.setControl(new Follower(m_motorShooterLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        m_motorShooterBottomFollower = new TalonFX(motorBottomFolID, "Ryan");
        m_motorShooterBottomFollower.getConfigurator().apply(ShooterConfigs.kKrakenFollowerConfig);
        
        // slot 1 = flywheel
        // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
        Slot1Configs slot1 = new Slot1Configs();
        slot1.kP = ShooterConstants.kShooterP;
        slot1.kI = ShooterConstants.kShooterI;
        slot1.kD = ShooterConstants.kShooterD;
        slot1.kS = ShooterConstants.kShooterS;
        slot1.kV = ShooterConstants.kShooterV;
        m_motorShooterLeader.getConfigurator().apply(slot1);
        m_motorShooterBottomFollower.getConfigurator().apply(slot1);

        debugInit();
        createLookupTable();
    }

    // -------------------- METHODS --------------------

    // Lookup Tables
    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/interpolation/InterpolatingDoubleTreeMap.html

    public void createLookupTable(){
        // distance, velocity
        velocityTable.put(3.0, 75.0);
        velocityTable.put(3.6, 80.0);
    }

    public double lookupVelocity(double distance){
        // linear interpolation
        if(distance > ShooterConstants.kMaxShooterRange ||
           distance < ShooterConstants.kMinShooterRange){
            return 0.0;
        }
        return velocityTable.get(distance);
    }

    public double getDistance(){

        if (targetPoseRobot == null) return m_currentRange;

        double x = targetPoseRobot.getX();
        double z = targetPoseRobot.getZ();

        m_currentRange = Math.sqrt(
            Math.pow(x, 2) +
            Math.pow(z, 2)
        );

        return m_currentRange;
    }

    public void runShooterMotors(double leaderSpeed) {
        leaderSpeed = MathUtil.clamp(leaderSpeed, 
                                     ShooterConstants.kMinShooterVelocity, 
                                     ShooterConstants.kMaxShooterVelocity);

        SmartDashboard.putNumber("3) Requested Velocity", leaderSpeed);

        m_motorShooterLeader.setControl(m_requestFlywheel.withVelocity(leaderSpeed));
        m_motorShooterBottomFollower.setControl(m_requestFlywheelBottom.withVelocity(leaderSpeed * ShooterConstants.kBottomMotorRatio));
    }

    public void stopShooterMotors(){
        m_motorShooterLeader.setControl(new DutyCycleOut(0.0));
        m_motorShooterBottomFollower.setControl(new DutyCycleOut(0.0));
    }

    // -------------------- COMMANDS --------------------
    public Command runFlywheel(DoubleSupplier speed) {
        return new FunctionalCommand(
            () -> {},
            () -> runShooterMotors(speed.getAsDouble()),
            interrupted -> runShooterMotors(0.0),
            () -> false,
            this
        );
    }

    public Command runFlywheelDashboard() {
        return new FunctionalCommand(
            () -> {},
            () -> runShooterMotors(ShooterConstants.kFlywheelVelocityInput),
            interrupted -> runShooterMotors(0.0),
            () -> false,
            this
        );
    }

    public Command stopFlywheel(){
        return this.run(() -> stopShooterMotors());
    }

    public Command shoot() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                double rps = lookupVelocity(getDistance());
                runShooterMotors(rps);
            },
            interrupted -> {
                stopShooterMotors();
            },
            () -> false,
            this
        );
    }

    // -------------------- DEBUG --------------------

    private void debugInit() {
        
        // FLYWHEEL STATS
        SmartDashboard.putNumber("1) Real Velocity (Leader)", 0.0);
        SmartDashboard.putNumber("2) Real Velocity (Bottom)", 0.0);
        SmartDashboard.putNumber("3) Requested Velocity", 0.0);
        SmartDashboard.putNumber("4) Flywheel Velocity Input", 0.0);

        // LIMELIGHT STATS
        SmartDashboard.putNumber("LL - Target X (m)", 0.0);
        SmartDashboard.putNumber("LL - Target Y (m)", 0.0);
        SmartDashboard.putNumber("LL - Target Z (m)", 0.0);
        SmartDashboard.putNumber("LL - Distance (m)", 0.0);
        SmartDashboard.putBoolean("LL - Target Visible", false);
    }

    @Override
    public void periodic() {
        
        // FLYWHEEL STATS
        SmartDashboard.putNumber("1) Real Velocity (Leader)", m_motorShooterLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("2) Real Velocity (Bottom)", m_motorShooterBottomFollower.getVelocity().getValueAsDouble());
        ShooterConstants.kFlywheelVelocityInput = SmartDashboard.getNumber("4) Flywheel Velocity Input", 0.0);
        
        // LIMELIGHT STATS
        SmartDashboard.putBoolean("LL - Target Visible", LimelightHelpers.getTV("limelight"));
        if(LimelightHelpers.getTV("limelight")){
            targetPoseRobot = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");

            double x = targetPoseRobot.getX();
            double y = targetPoseRobot.getY();
            double z = targetPoseRobot.getZ();

            SmartDashboard.putNumber("LL - Target X (m)", x);
            SmartDashboard.putNumber("LL - Target Y (m)", y);
            SmartDashboard.putNumber("LL - Target Z (m)", z);
            SmartDashboard.putNumber("LL - Distance (m)", getDistance());
        }
    }

    @Override
    public void simulationPeriodic() {}
}