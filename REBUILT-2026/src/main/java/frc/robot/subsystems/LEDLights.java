package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LEDLights.LEDmode;
import frc.robot.utils.Configs.CANdleConfigs;
import frc.robot.utils.Constants.CANdleConstants;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.Constants.LimitSwitchConstants;
import frc.robot.utils.Constants.ShooterConstants;


public class LEDLightsCANdle extends SubsystemBase {
 
  private final Shooter m_shooter;
  private final CANdle m_CANdle = new CANdle(CANdleConstants.CANdleID, CANdleConstants.kCanbusName);
  private boolean seeAT;
  private boolean centerAT;
  private boolean inRange;

public enum LEDMode {
    Off (new EmptyAnimation(0)),
    SeesApriltag (new StrobeAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(0, 255, 0))), //limelight can see the apriltag
    TargetingApriltag (new SolidColor(0, CANdleConstants.ledCount -1).withColor(new RGBWColor(0, 255, 0))), //limelight is targeting the tag
    Shooting (new FireAnimation(0, CANdleConstants.ledCount - 1).withBrightness(1.0)),
    Charging (new StrobeAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255, 35, 0))),
    Intaking (new TwinkleAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255,115,0))),
    LimitSwitchDetected (new SolidColor(0, CANdleConstants.ledCount -1).withColor(new RGBWColor(0,255,0)));
    
    //comment

    public final ControlRequest animation;

    private LEDMode (ControlRequest animation) {
      this.animation = animation;
    }
  }

    
  public LEDLightsCANdle(Shooter i_shooter) {
    this.m_shooter = i_shooter;
    m_CANdle.getConfigurator().apply(CANdleConfigs.CANdleConfig);
  }
  public Command setLEDCommand(LEDMode newMode) {
    return new RunCommand(() -> m_CANdle.setControl(newMode.animation), this);
  }

  public Command CheckLimeLights() {
    return new RunCommand(() ->
    {
      seeAT = LimelightHelpers.getTV(LimelightConstants.limelightName);
      centerAT = Math.abs(LimelightHelpers.getTX(LimelightConstants.limelightName)) < 1;
      inRange = m_shooter.getDistance() < ShooterConstants.kMinShooterRange;
      
      if (!seeAT) {
       setLEDCommand(LEDMode.Off);
      } 
      else if (!inRange) {
        setLEDCommand(LEDMode.Shooting);
      }
      else if (!centerAT) {
          setLEDCommand(LEDMode.SeesApriltag);
      } else {
        setLEDCommand(LEDMode.TargetingApriltag);
      }
    }, this);
  }

  @Override
  public void periodic() {
      CheckLimeLights();
  }
}
