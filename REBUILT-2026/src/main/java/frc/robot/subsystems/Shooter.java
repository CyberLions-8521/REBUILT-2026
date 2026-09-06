package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.limelightvision.Limelight;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.FunctionalCommand;
import org.wpilib.command2.SubsystemBase;
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
    private Limelight m_camera = SwerveDrivebase.getInstance().getLimelightCamera();

    //global consts (for readability)
    private static final double g = ShooterConstants.kGravity;
    private static final double h = ShooterConstants.kDeltaHeight;

    private static final int[] validIDs = {2, 5, 4, 10, 18, 21, 20, 26};

    public Shooter() {
        // main motors

        //top left motor
        m_upperFlywheelLeader = new TalonFX(ShooterConstants.kShooterTopLeftID, new CANBus(ShooterConstants.kCanbusName));
        m_upperFlywheelLeader.getConfigurator().apply(ShooterConfigs.upperFlywheelConfigs);

        //top right motor
        m_upperFlywheelFollower = new TalonFX(ShooterConstants.kShooterTopRightID, new CANBus(ShooterConstants.kCanbusName));
        m_upperFlywheelFollower.getConfigurator().apply(ShooterConfigs.lowerFlywheelConfigs);   
        m_upperFlywheelFollower.setControl(new Follower(m_upperFlywheelLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        //bottom right motor
        m_lowerFlywheel = new TalonFX(ShooterConstants.kShooterBottomRightID, new CANBus(ShooterConstants.kCanbusName));
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
    // https://github.wpilib.org/allwpilib/docs/release/java/org.wpilib.math/interpolation/InterpolatingDoubleTreeMap.html

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

    public double getDistance(Pose2d estimatedPose, Translation2d targetPoint) { // overloaded version for distance based on real odometry
        return targetPoint.getDistance(estimatedPose.getTranslation());
    }

    public Supplier<Double> getDynamicRPS(Supplier<Pose2d> estimatedPose, Supplier<Translation2d> targetPoint) {
        return () -> {
            double distance = getDistance(estimatedPose.get(), targetPoint.get());
            return lookupVelocity(distance);
        };
    }

    public void runUpperFlywheelMotors(double speed) {
        speed = Math.clamp(speed,
            ShooterConstants.kMinShooterVelocity,
            ShooterConstants.kMaxShooterVelocity);

        m_upperFlywheelLeader.setControl(m_requestFlywheel.withVelocity(speed));
    }

    public void runLowerFlywheelMotors(double speed) {
        speed = Math.clamp(speed,
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
        return MathUtil.isNear(targetRPS, velocity, 5);
    }

    public boolean isShooterAtSpeed(Supplier<Double> targetRPS) {
        double upperVelocity = m_upperFlywheelLeader.getVelocity().getValueAsDouble();
        double lowerVelocity = m_lowerFlywheel.getVelocity().getValueAsDouble();
        boolean isUpperAtSpeed = MathUtil.isNear(targetRPS.get(), upperVelocity, 5);
        boolean isLowerAtSpeed = MathUtil.isNear(targetRPS.get(), lowerVelocity, 5);
        return isUpperAtSpeed && isLowerAtSpeed;
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
                // m_LedLights.setLEDMode(LEDMode.Charging);
                if (isUpperAtSpeed(rps)) {
                    runLowerFlywheelMotors(rps);
                    // m_LedLights.setLEDMode(LEDMode.Shooting);

                }
            },
            interrupted -> {
                stopBothFlywheelMotors();
                // m_LedLights.setLEDMode(LEDMode.Off);
            },
            () -> false,
            this
        );
    }

    public Command ShootWithoutAprilTagCommand(double rps) { // for the subsystem controller commands
        return new FunctionalCommand(
            () -> {},
            () -> {
                runUpperFlywheelMotors(rps);
                // m_LedLights.setLEDMode(LEDMode.Charging);
                if (isUpperAtSpeed(rps)) {
                    runLowerFlywheelMotors(rps);
                    // m_LedLights.setLEDMode(LEDMode.Shooting);

                }
            },
            interrupted -> {
                stopBothFlywheelMotors();
                // m_LedLights.setLEDMode(LEDMode.Off);

            },
            () -> false,
            this
        );
    }

    public Command ShootWithoutAprilTagCommand(Supplier<Double> rpsSupplier) { // for the drivebase controller commands
        return new FunctionalCommand(
            () -> {},
            () -> {
                double rps = rpsSupplier.get();
                runUpperFlywheelMotors(rps);
                // m_LedLights.setLEDMode(LEDMode.Charging);
                if (isUpperAtSpeed(rps)) {
                    runLowerFlywheelMotors(rps);
                    // m_LedLights.setLEDMode(LEDMode.Shooting);

                }
            },
            interrupted -> {
                stopBothFlywheelMotors();
                // m_LedLights.setLEDMode(LEDMode.Off);

            },
            () -> false,
            this
        );
    }

    public Command WarmUpShooter(double rps) { // only warms up the upper motors lol
        return new FunctionalCommand(
            () -> {},
            () -> {
                runUpperFlywheelMotors(rps);
                // m_LedLights.setLEDMode(LEDMode.Charging);
            },
            interrupted -> {
                // m_LedLights.setLEDMode(LEDMode.Off);
            },
            () -> false,
            this
        );
    }

    public Command WarmUpShooter(Supplier<Double> rpsSupplier) { // only warms up the upper motors lol (but with dynamic rps)
        return new FunctionalCommand(
            () -> {},
            () -> {
                double rps = rpsSupplier.get();
                runUpperFlywheelMotors(rps);
                // m_LedLights.setLEDMode(LEDMode.Charging);
            },
            interrupted -> {
                // m_LedLights.setLEDMode(LEDMode.Off);
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
        SmartDashboard.putBoolean("LL - Target Visible", m_camera.hasTarget());
        if(m_camera.hasTarget()){
            // original LimelightHelpers method was Pose3d, will have to deal with Pose2d + LimelightLib for now
            targetPoseRobot = new Pose3d(m_camera.getPoseEstimate(Limelight.PoseEstimateType.MT2_WPIBLUE).pose);

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