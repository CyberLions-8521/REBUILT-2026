package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.utils.Configs.ShooterConfigs;
import frc.robot.utils.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private final TalonFX m_upperFlywheelLeader;
    private final TalonFX m_upperFlywheelFollower;
    private final TalonFX m_lowerFlywheel;

    private final VelocityVoltage m_requestFlywheel = new VelocityVoltage(0).withSlot(1);
    private final VelocityVoltage m_requestFlywheelBottom = new VelocityVoltage(0).withSlot(1);

    private final InterpolatingDoubleTreeMap velocityTable = new InterpolatingDoubleTreeMap();
    private double m_currentRange = 0.0;
    private Pose3d targetPoseRobot;


    //global consts (for readability)
    private static final double g = ShooterConstants.kGravity;
    private static final double h = ShooterConstants.kDeltaHeight;

    private static final int[] validIDs = {2, 5, 4, 10, 18, 21, 20, 26};

    public Shooter() {
        LimelightHelpers.SetFiducialIDFiltersOverride("limeilght", validIDs);
        // main motors

        //top left motor
        m_upperFlywheelLeader = new TalonFX(ShooterConstants.kShooterTopLeftID, ShooterConstants.kCanbusName);
        m_upperFlywheelLeader.getConfigurator().apply(ShooterConfigs.upperFlywheelConfigs);

        //top right motor
        m_upperFlywheelFollower = new TalonFX(ShooterConstants.kShooterTopRightID, ShooterConstants.kCanbusName);
        m_upperFlywheelFollower.getConfigurator().apply(ShooterConfigs.lowerFlywheelConfigs);   
        m_upperFlywheelFollower.setControl(new Follower(m_upperFlywheelLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        //bottom right motor
        m_lowerFlywheel = new TalonFX(ShooterConstants.kShooterBottomRightID, ShooterConstants.kCanbusName);
        m_lowerFlywheel.getConfigurator().apply(ShooterConfigs.lowerFlywheelConfigs);
        
        // slot 1 = flywheel
        // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
        Slot1Configs slot1 = new Slot1Configs();
        slot1.kP = ShooterConstants.kShooterP;
        slot1.kI = ShooterConstants.kShooterI;
        slot1.kD = ShooterConstants.kShooterD;
        slot1.kS = ShooterConstants.kShooterS;
        slot1.kV = ShooterConstants.kShooterV;
        m_upperFlywheelLeader.getConfigurator().apply(slot1);
        m_lowerFlywheel.getConfigurator().apply(slot1);

        debugInit();
        createLookupTable();
    }

    // -------------------- METHODS --------------------

    // Lookup Tables
    // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/interpolation/InterpolatingDoubleTreeMap.html

    public void createLookupTable(){
        // distance, velocity
        velocityTable.put(5.8, 63.0);
        velocityTable.put(5.45, 62.4);
        velocityTable.put(4.85,60.0);
        velocityTable.put(4.35, 57.0);
        velocityTable.put(4.1, 55.0);
        velocityTable.put(3.9, 53.5);
        velocityTable.put(3.5, 52.5);
        velocityTable.put(3.25, 51.0);
        velocityTable.put(2.85, 50.0);
        velocityTable.put(2.585, 49.25);
        velocityTable.put(2.3, 47.5);  
        velocityTable.put(2.025, 46.0);
        velocityTable.put(1.8, 45.0);
        velocityTable.put(1.53, 44.0);
    }

    public double lookupVelocity(double distance){
        // linear interpolation
        if(distance > ShooterConstants.kMaxShooterRange ||
           distance < ShooterConstants.kMinShooterRange){
            return 0.0;
        }
        return velocityTable.get(distance);
    }

    public double calculateVelocity(double distance){
        double a = ShooterConstants.kA;
        double b = ShooterConstants.kB;
        return(a + b * distance);
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

    public void runUpperFlywheelMotors(double speed) {
        speed = MathUtil.clamp(speed,
            ShooterConstants.kMinShooterVelocity,
            ShooterConstants.kMaxShooterVelocity);

        m_upperFlywheelLeader.setControl(m_requestFlywheel.withVelocity(speed));
    }

    public void runLowerFlywheelMotors(double speed) {
        speed = MathUtil.clamp(speed,
            ShooterConstants.kMinShooterVelocity,
            ShooterConstants.kMaxShooterVelocity);

        m_lowerFlywheel.setControl(m_requestFlywheel.withVelocity(speed * ShooterConstants.kBottomMotorRatio));
    }

    public void stopUpperFlywheelMotors(){
        m_upperFlywheelLeader.setControl(new DutyCycleOut(0.0));
    }

    public void stopLowerFlywheelMotors(){
        m_lowerFlywheel.setControl(new DutyCycleOut(0.0));
    }

    public void stopBothFlywheelMotors(){
        stopUpperFlywheelMotors();
        stopLowerFlywheelMotors();
    }

    private boolean isUpperAtSpeed(double targetRPS) {
        double velocity = m_upperFlywheelLeader.getVelocity().getValueAsDouble();
        return MathUtil.isNear(targetRPS, velocity, 10);
    }
    

    // // -------------------- COMMANDS --------------------
    // public Command runFlywheel(DoubleSupplier speed) {
    //     return new FunctionalCommand(
    //         () -> {},
    //         () -> runShooterMotors(speed.getAsDouble()),
    //         interrupted -> runShooterMotors(0.0),
    //         () -> false,
    //         this
    //     );
    
    // We are so cooked and when i mean cooked i mean the gears keep getting cooked + they installed brass fly wheels so you have to alter the S and V values
// the reason the i hate wdoewrge wodswthe rewasdpmthe the prhrw asdwpthe rkwspthe wtkthe owthoewthsoyw eyhwe ythewa eeasdtwe dthaewsdwthwad sderrdsthwaesadwthasyuo
    // public Command runFlywheelDashboard() {
    //     return new FunctionalCommand(
    //         () -> {},
    //         () -> runShooterMotors(ShooterConstants.kFlywheelVelocityInput),
    //         interrupted -> runShooterMotors(0.0),
    //         () -> false,
    //         this
    //     );
    // }


    public Command StopUpperFlywheelCommand(){
        return run(this::stopUpperFlywheelMotors);
    }

    public Command StopLowerFlywheelCommand(){
        return run(this::stopLowerFlywheelMotors);
    }

    public Command stopBothFlywheelCommand(){
        return this.run(() -> stopBothFlywheelMotors());
    }

    

    public Command ShootWithAprilTagCommand() {
        return new FunctionalCommand(
            () -> {},
            () -> {
                double rps = lookupVelocity(getDistance());
                runUpperFlywheelMotors(rps);
                if (isUpperAtSpeed(rps)) {
                    runLowerFlywheelMotors(rps);
                }
            },
            interrupted -> {
                stopBothFlywheelMotors();
            },
            () -> false,
            this
        );
    }

    public Command ShootWithoutAprilTagCommand(double rps) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                runUpperFlywheelMotors(rps);
                if (isUpperAtSpeed(rps)) {
                    runLowerFlywheelMotors(rps);
                }
            },
            interrupted -> {
                stopBothFlywheelMotors();
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
        SmartDashboard.putNumber("LL - Target X (m)", 5.26);
        SmartDashboard.putNumber("LL - Target Y (m)", 0.0);
        SmartDashboard.putNumber("LL - Target Z (m)", 1.26);
        SmartDashboard.putNumber("LL - Distance (m)", 5.41);
        SmartDashboard.putBoolean("LL - Target Visible", false);
    }
        // 5.41^2 = 1.26^2 + X^2
    @Override
    public void periodic() {
        
        // FLYWHEEL STATS
        SmartDashboard.putNumber("1) Real Velocity (Leader)", m_upperFlywheelLeader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("2) Real Velocity (Bottom)", m_lowerFlywheel.getVelocity().getValueAsDouble());
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