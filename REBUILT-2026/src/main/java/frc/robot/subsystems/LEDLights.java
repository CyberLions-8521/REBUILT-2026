// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class LEDLights extends SubsystemBase {

  DigitalInput m_leftLimitSwitch = new DigitalInput(LimitSwitchConstants.kLeftLimitSwitchID);
  DigitalInput m_rightLimitSwith = new DigitalInput(LimitSwitchConstants.kRightLimitSwitchID);

  public enum LEDMode {
    Off (new EmptyAnimation(0)),
    SeesApriltag (new StrobeAnimation(0, CANdleConstants.kLedCount - 1).withColor(new RGBWColor(0, 255, 0))), //limelight can see the apriltag
    TargetingApriltag (new SolidColor(0, CANdleConstants.kLedCount -1).withColor(new RGBWColor(0, 255, 0))), //limelight is targeting the tag
    Shooting (new FireAnimation(0, CANdleConstants.kLedCount - 1).withBrightness(1.0)),
    Charging (new StrobeAnimation(0, CANdleConstants.kLedCount - 1).withColor(new RGBWColor(255, 35, 0))),
    Intaking (new TwinkleAnimation(0, CANdleConstants.kLedCount - 1).withColor(new RGBWColor(255,115,0))),
    LimitSwitchDetected (new SolidColor(0, CANdleConstants.kLedCount -1).withColor(new RGBWColor(0,255,0)));
    
    public final ControlRequest animation;

    private LEDMode (ControlRequest animation) {
      this.animation = animation;
    }
  }

  private final CANdle m_CANdle = new CANdle(CANdleConstants.kCANdleID, CANdleConstants.kCanbusName);


  public LEDLights() {
    m_CANdle.getConfigurator().apply(CANdleConfigs.CANdleConfigs);
  }

  public Command setLEDCommand(LEDMode newMode) {
    return new RunCommand(() -> m_CANdle.setControl(newMode.animation), this);
  }

  public boolean isLimitSwitchPressed(){
    return !m_leftLimitSwitch.get() && !m_rightLimitSwith.get();
  }

  public Command limitSwitchLEDCommand() {
    return new RunCommand(() -> {
      m_CANdle.setControl(isLimitSwitchPressed() ? LEDMode.LimitSwitchDetected.animation : LEDMode.Off.animation);
    }, this);
  }

 
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("LimitSwitchStatus", !m_leftLimitSwitch.get());

    if(!m_leftLimitSwitch.get() && !m_rightLimitSwith.get()){
      m_CANdle.setControl(LEDMode.Shooting.animation);
    } 
    else {
       m_CANdle.setControl(LEDMode.Off.animation);
    }
    


  }
}