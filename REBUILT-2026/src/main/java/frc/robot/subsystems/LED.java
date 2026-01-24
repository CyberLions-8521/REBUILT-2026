// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  private static final int kID = 0;
  private CANdle candle = new CANdle(kID, new CANBus("Ryan")); // change ID later

  //colors
  private static final RGBWColor kGreen = new RGBWColor(255, 0, 0);
  private static final RGBWColor kOff = new RGBWColor();

  //animations
  private final SolidColor solid = new SolidColor(0, 120);

  public LED() {
    // Configure the CANdle for basic use
    CANdleConfiguration configs = new CANdleConfiguration();
    // Set the LED strip type and brightness
    configs.LED.withStripType(StripTypeValue.RGB)
            .withBrightnessScalar(0.5);

    // Disable status LED when being controlled
    configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    candle.getConfigurator().apply(configs);
  }

  @Override
  public void periodic() {
    /* This method will be called once per scheduler run */
    
    if(LimelightHelpers.getTV("")) {
      turnOn();
    } else {
      turnOff();
    }
  }

  public void turnOn() {
    candle.setControl(solid.withColor(kGreen));
  }

  public void turnOff() {
    candle.setControl(solid.withColor(kOff));
  }
}
