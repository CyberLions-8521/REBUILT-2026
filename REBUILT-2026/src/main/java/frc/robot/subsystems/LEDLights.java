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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.CANdleConfigs;
import frc.robot.Constants.CANdleConstants;

public class LEDLights extends SubsystemBase {

  public enum LEDMode {
    Off (new EmptyAnimation(0)),
    SeesApriltag (new StrobeAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(0, 255, 0))), //limelight can see the apriltag
    TargetingApriltag (new SolidColor(0, CANdleConstants.ledCount -1).withColor(new RGBWColor(0, 255, 0))), //limelight is targeting the tag
    Shooting (new FireAnimation(0, CANdleConstants.ledCount - 1).withBrightness(1.0)),
    Charging (new StrobeAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255, 35, 0))),
    Intaking (new TwinkleAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255,115,0))),
    LimitSwitchDetected (new SolidColor(0, CANdleConstants.ledCount -1).withColor(new RGBWColor(0,255,0)));

    
    // ColorFlow(new ColorFlowAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255, 0 ,0)).withSlot(0)),
    // RGBFade(new RgbFadeAnimation(0, CANdleConstants.ledCount-1)),
    // SingleFade(new SingleFadeAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255, 0 ,0))),
    // Twinkle( new TwinkleAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255, 0 ,0))),
    // RedSolid( new SolidColor(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255,0,0))),
    // BlueSolid( new SolidColor(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(0,255,0))),
    // GreenSolid( new SolidColor(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(0,0,255)));
    
    public final ControlRequest animation;

    private LEDMode (ControlRequest animation) {
      this.animation = animation;
    }
  }

  // void letsGetLitty(int strength) {
  //   for (int i = CANdleConstants.ledCount - 1; i > 0; i--) {
  //     strength = strength / (CANdleConstants.ledCount - 1);
  //     new ColorFlowAnimation(0, strength).withColor(new RGBWColor(255, 0, 0));
  //   }

  // }

  private final CANdle m_CANdle = new CANdle(CANdleConstants.CANdleID, new CANBus("Ryan"));


  public LEDLights() {
    m_CANdle.getConfigurator().apply(CANdleConfigs.CANdleConfig);
  }

  public Command setLEDCommand(LEDMode newMode) {
    return new RunCommand(() -> m_CANdle.setControl(newMode.animation), this);
  }

  
  @Override
  public void periodic() {
   
  }
}