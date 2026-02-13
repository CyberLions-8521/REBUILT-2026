package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.SwerveDrivebase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;

public class AutoAlignCommand extends Command {
    private final SwerveDrivebase m_swerve;

    public AutoAlignCommand(SwerveDrivebase swerve) {
        this.m_swerve = swerve; 
        addRequirements(swerve);
    }

    double getTargetingAngularVelocity() { // aiming control
        double tx = LimelightHelpers.getTX(LimelightConstants.kName);

        double targetingAngularVelocity = tx * LimelightConstants.kAimP;

        //conversion to radians/second
        targetingAngularVelocity *= SwerveConstants.kMaxAngularSpeed;
        //invert since tx is positive to the right
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    double getTargetingForwardSpeed() { // ranging control
        double targetingForwardSpeed = LimelightHelpers.getTY(LimelightConstants.kName) * LimelightConstants.kRangeP;
        targetingForwardSpeed *= SwerveConstants.kMaxMetersPerSecond;
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