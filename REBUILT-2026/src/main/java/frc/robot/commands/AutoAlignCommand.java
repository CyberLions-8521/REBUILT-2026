package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.SwerveDrivebaseConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;

public class AutoAlignCommand extends CommandBase {
    private final Swerve m_swerve;

    public AutoAlignCommand(Swerve swerve) {
        this.swerve = swerve; 
        addRequirements(swerve);
    }

    double getTargetingAngularVelocity() { // aiming control
        double tx = LimelightHelpers.getTX(LimelightConstants.kName);

        double targetingAngularVelocity = tx * LimelightConstants.kAimP;

        //conversion to radians/second
        targetingAngularVelocity *= SwerveDrivebaseConstants.kMaxAngularSpeed;
        //invert since tx is positive to the right
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    double getTargetingForwardSpeed() { // ranging control
        double targetingForwardSpeed = LimelightHelpers.getTY(LimelightConstants.kName) * LimelightConstants.kRangeP;
        targetingForwardSpeed *= SwerveDrivebaseConstants.kMaxMetersPerSecond;
        targetingForwardSpeed *= -1.0; //invert since ty is positive when target is above crosshair
        return targetingForwardSpeed;
    }

    @Override
    public void execute() {
        m_swerve.drive(getTargetingForwardSpeed(), 0, getTargetingAngularVelocity(), false);
    }

    @Override
    public void end(boolean interrupted){
        m_swerve.drive(0, 0, 0, false);
    }


}