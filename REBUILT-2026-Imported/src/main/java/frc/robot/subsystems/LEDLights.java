// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.limelightvision.Limelight;

import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.SubsystemBase;
import frc.robot.utils.Configs.CANdleConfigs;
import frc.robot.utils.Constants.CANdleConstants;
import frc.robot.utils.Constants.ShooterConstants;

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

  private final CANdle m_CANdle = new CANdle(CANdleConstants.kCANdleID, new CANBus(CANdleConstants.kCanbusName));
  private LEDMode currentMode = LEDMode.Off;
  private Shooter m_shooter;
  private Limelight m_Limelight;
  private SwerveDrivebase m_drivebase;

  public LEDLights(Shooter i_shooter, Limelight i_Limelight, SwerveDrivebase i_drivebase) {
    this.m_shooter = i_shooter;
    this.m_Limelight = i_Limelight;
    this.m_drivebase = i_drivebase;
    m_CANdle.getConfigurator().apply(CANdleConfigs.CANdleConfigs);
    
  }

  public void setLEDMode(LEDMode newMode){
    currentMode = newMode;
  }

  public Command setLEDCommand(LEDMode newMode) {
    return new RunCommand(() -> m_CANdle.setControl(newMode.animation), this);
  }

  @Override
  public void periodic() { 
    boolean hasTarget = m_Limelight.hasTarget();
    boolean isAutoAligned = m_drivebase.isAutoAligned();
    boolean isWithinShooterRange = m_shooter.getDistance() > ShooterConstants.kMinShooterRange && m_shooter.getDistance() < ShooterConstants.kMaxShooterRange;
    
    if (!hasTarget || !isWithinShooterRange) {
      setLEDMode(LEDMode.Off);
    } else {
      if (isAutoAligned) {
        setLEDMode(LEDMode.AlignedToApriLTag);
      } else {
        setLEDMode(LEDMode.SeesAprilTag);
      }
    }
    m_CANdle.setControl(currentMode.animation);
  }
   
}