// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;
import frc.robot.Configs.CANdleConfigs;
import frc.robot.Constants.CANdleConstants;
import com.ctre.phoenix6.controls.RainbowAnimation;

public class LEDLights extends SubsystemBase {

  public enum LEDMode {
    Idle, //nothing is happening
    SeesApriltag, //limelight can see the apriltag
    TargetingApriltag, //limelight is targeting the tag
    Charging, //buidling up speed before shooting;
    Shooting, //fuel is being shot
    ShootFail //the shooter can't find a possible combo of velocity/angle to shoot into the target. may or may not get implemented
  }

  CANdle m_CANdle = new CANdle(CANdleConstants.CANdleID, new CANBus("rio"));
  LEDMode currentMode;

  public LEDLights() {
    currentMode = LEDMode.Idle;
    m_CANdle.getConfigurator().apply(CANdleConfigs.CANdleConfig);
  }

  public void setMode(LEDMode mode) {
    currentMode = mode;
  }
  
  public void updateMode() {
    switch (currentMode) {
      case Idle: 
        m_CANdle.setControl(new FireAnimation(0, CANdleConstants.ledCount - 1));
        break;
      case SeesApriltag:
        m_CANdle.setControl(new StrobeAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(124, 252, 0)));
        break;
      case TargetingApriltag:
        m_CANdle.setControl(new SolidColor(0, CANdleConstants.ledCount -1).withColor(new RGBWColor(124, 252, 0)));
        break;
      case Charging:
        m_CANdle.setControl(new SolidColor(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255, 0, 255)));
      case Shooting:
        m_CANdle.setControl(new StrobeAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255, 0, 255)));
      case ShootFail:
        m_CANdle.setControl(new StrobeAnimation(0, CANdleConstants.ledCount - 1).withColor(new RGBWColor(255, 36, 0)));
    }
  }
  
  @Override
  public void periodic() {
    updateMode();
  }
}