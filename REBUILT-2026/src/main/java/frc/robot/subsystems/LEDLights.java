// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.Elastic;
import frc.robot.util.LimelightHelpers;

public class LEDLights extends SubsystemBase {

  CANdle m_leds = new CANdle(0, new CANBus("Ryan"));
  Elastic.Notification notification = new Elastic.Notification();

  public LEDLights() {
    CANdleConfiguration CANdleConfigs = new CANdleConfiguration();
    CANdleConfigs.LED.withStripType(StripTypeValue.GRB)
                     .withBrightnessScalar(0.5);
    m_leds.getConfigurator().apply(CANdleConfigs);
  }

  @Override
  public void periodic() {
    if (LimelightHelpers.getTV(LimelightConstants.limelightName)) {
      m_leds.setControl(new SolidColor(0, 120).withColor(new RGBWColor(0, 255,0)));

      Elastic.sendNotification(notification
             .withLevel(Elastic.NotificationLevel.INFO)
             .withTitle("LED Status")
             .withDescription("April Tag Detected! (green)")
             .withDisplaySeconds(6.7)
      );

    } else {
      m_leds.setControl(new SolidColor(0, 120).withColor(new RGBWColor(255, 0, 0)));

      Elastic.sendNotification(notification
             .withLevel(Elastic.NotificationLevel.INFO)
             .withTitle("LED Status")
             .withDescription("April Tag Not Detected! (red)")  
             .withDisplaySeconds(2)
      );
    }
  }
}