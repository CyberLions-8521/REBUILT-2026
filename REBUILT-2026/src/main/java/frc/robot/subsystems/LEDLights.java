
package frc.robot.subsystems;

/* 
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
*/

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.utils.Configs.CANdleConfigs;
import frc.robot.utils.Constants.CANdleConstants;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.subsystems.Shooter;

public class LEDLights extends SubsystemBase {
  private final CANdle m_CANdle = new CANdle(CANdleConstants.CANdleID, CANdleConstants.kCanbusName);
  private static int m_LEDcount = 256;

  public boolean seeAprilTag;
  public boolean centeredAprilTag;
  public boolean inRange;

  public enum LEDMode {
    Off (new EmptyAnimation(0)),
    SeesAprilTag (new SolidColor(0, m_LEDcount - 1).withColor( new RGBWColor(255,150,0))),
    OnAprilTag (new SolidColor(0, m_LEDcount - 1).withColor( new RGBWColor(255,255,0))),
    InRange(new SolidColor(0, m_LEDcount - 1).withColor( new RGBWColor(0, 0, 255))),
    AllChecks(new SolidColor(0, m_LEDcount - 1).withColor( new RGBWColor(0, 255, 0)));

    public final ControlRequest animation;

    private LEDMode (ControlRequest animation) {
      this.animation = animation;
    }
  }

  public LEDLights() {
    m_CANdle.getConfigurator().apply(CANdleConfigs.CANdleConfig);
  }


  public void setLEDCommand( LEDMode newMode) {
    m_CANdle.setControl(newMode.animation);
  }

  // public Command setLEDCommandTimed(LEDMode newMode, double time) {
  //   return new RunCommand(() -> m_CANdle.setControl(newMode.animation), this)
  //                         .withTimeout(time)
  //                         .andThen(setLEDCommand(LEDMode.Off));
  // } 

  public void ForceOff() {
    m_CANdle.setControl(LEDMode.Off.animation);
  }

  public Command CheckLimelight() {
    return new RunCommand(
      () -> 
      {
      seeAprilTag = LimelightHelpers.getTV(LimelightConstants.limelightName);
      centeredAprilTag = Math.abs(LimelightHelpers.getTX(LimelightConstants.limelightName)) < 1;
      inRange = Shooter.getDistance() > 1.5;
      
      if (seeAprilTag){
        setLEDCommand(LEDMode.SeesAprilTag);
        if(!(inRange && centeredAprilTag)){
          if(inRange){
            setLEDCommand(LEDMode.InRange);
          }
          if (centeredAprilTag) 
          { 
            setLEDCommand(LEDMode.OnAprilTag);
          }
        } else {
          setLEDCommand(LEDMode.AllChecks);
        }
      }
      else { 
        ForceOff(); 
      }
      },this);
  }
  /* Sample Codew
   * if (LimelightHelpers.getTVA() && keycode.x is pressed down) {
   *  activate the red one
   * } 
   * if (LimelightHelpers.getTVA() && keycode.x is pressed down && its within tolerance) }
   *  activate the green one
   * }
   * activate off;
   */
  @Override
  public void periodic() {
  
  }
}
