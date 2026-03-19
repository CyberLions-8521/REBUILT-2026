
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

import frc.robot.utils.Configs.CANdleConfigs;
import frc.robot.utils.Constants.CANdleConstants;
import frc.robot.utils.Constants.LimitSwitchConstants;

public class LED extends SubsystemBase {
  private final CANdle m_CANdle = new CANdle(CANdleConstants.CANdleID, CANdleConstants.kCanbusName);
  private static int m_LEDcount = 256;

  public enum LEDMode {
    Off (new EmptyAnimation(0)),
    SeesAprilTag (new SolidColor(0, m_LEDcount - 1).withColor( new RGBWColor(255,0,0))),
    OnApirlTag (new SolidColor(0, m_LEDcount - 1).withColor( new RGBWColor(0,255,0)));
    
    public final ControlRequest animation;

    private LEDMode (ControlRequest animation) {
      this.animation = animation;
    }
  }

  public LED() {
    m_CANdle.getConfigurator().apply(CANdleConfigs.CANdleConfig);
  }

  public Command setLECommand( LEDMode newMode) {
    return new RunCommand(() ->m_CANdle.setControl(newMode.animation) ,this);
  }

  public Command setLECommandTimed(LEDMode newMode, double time) {
    return new RunCommand(() -> m_CANdle.setControl(newMode.animation), this)
                          .withTimeout(time)
                          .andThen(setLECommand(LEDMode.Off));
  } 
  public Command ForceOff() {
    return new RunCommand(() -> m_CANdle.setControl(LEDMode.Off.animation), this);
  }
  
  @Override
  public void periodic() {
    
  }
}
