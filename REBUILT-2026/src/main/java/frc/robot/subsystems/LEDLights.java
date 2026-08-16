// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.Configs.CANdleConfigs;
import frc.robot.utils.Constants.CANdleConstants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class LEDLights extends SubsystemBase {

  // DigitalInput m_leftLimitSwitch = new DigitalInput(LimitSwitchConstants.kLeftLimitSwitchID);
  // DigitalInput m_rightLimitSwith = new DigitalInput(LimitSwitchConstants.kRightLimitSwitchID);

  public enum LEDMode {
    Off (new EmptyAnimation(0)),
    SeesAprilTag (new TwinkleAnimation(0, CANdleConstants.kLedCount - 1).withColor(new RGBWColor(255,0,0))),
    AlignedToApriLTag (new SolidColor(0, CANdleConstants.kLedCount -1).withColor(new RGBWColor(0, 255, 0))),  
    Intaking (new TwinkleAnimation(0, CANdleConstants.kLedCount - 1).withColor(new RGBWColor(255,115,0)));

    public final ControlRequest animation;

    private LEDMode (ControlRequest animation) {
      this.animation = animation;
    }
  }

  private final CANdle m_CANdle = new CANdle(CANdleConstants.kCANdleID, CANdleConstants.kCanbusName);
  private LEDMode currentMode = LEDMode.Off;
  private Shooter m_shooter;
  


  public LEDLights(Shooter i_shooter) {
    this.m_shooter = i_shooter;
    m_CANdle.getConfigurator().apply(CANdleConfigs.CANdleConfigs);
  }

  // public void turnOffLEDs(){
  //   m_CANdle.setControl(LEDMode.Off.animation);
  // }

  // public Command turnOffLEDsCommand() {
  //    return run(this::turnOffLEDs);
  // }
  public void setLEDMode(LEDMode newMode){
    currentMode = newMode;
  }

  public Command setLEDCommand(LEDMode newMode) {
    return new RunCommand(() -> m_CANdle.setControl(newMode.animation), this);
  }

  @Override
  public void periodic() {

    boolean seeAT = LimelightHelpers.getTV(LimelightConstants.limelightName); //checks if the april tag is visible
    boolean centerAT = Math.abs(LimelightHelpers.getTX(LimelightConstants.limelightName)) < 1; //checks if it is aligned
    boolean inRange = m_shooter.getDistance() < ShooterConstants.kMinShooterRange; //not in deadzone

    if (!seeAT) {
      currentMode = LEDMode.Off;
    } else if (!inRange) {
      currentMode = LEDMode.Off;
    } else if (!centerAT) {
      currentMode = LEDMode.SeesAprilTag;
    } else {
      currentMode = LEDMode.AlignedToApriLTag;
    }

    m_CANdle.setControl(currentMode.animation);
  }
}