// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {

  CANdle lights = new CANdle(0);

  public LimelightSubsystem() {
    CANdleConfiguration CANdleConfigs = new CANdleConfiguration();
    CANdleConfigs.LED.withStripType(StripTypeValue.RGB)
                     .withBrightnessScalar(0.5);
    lights.getConfigurator().apply(CANdleConfigs);
  }

  @Override
  public void periodic() {
    if (LimelightHelpers.getTV("")) {
      lights.setControl(new SolidColor(0, 120).withColor(new RGBWColor(124, 252, 0)));
    } else {
      lights.setControl(new SolidColor(0, 120).withColor(new RGBWColor(255, 0, 0)));
    }
    SmartDashboard.putBoolean("TX", LimelightHelpers.getTV(""));
  }
}
