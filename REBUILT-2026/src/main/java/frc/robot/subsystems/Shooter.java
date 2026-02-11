// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Shooter;


public class Shooter extends SubsystemBase {

  private TalonFX m_masterShooter;
  private TalonFX m_helperShooter;

  /** Creates a new Shooter. */
  public Shooter() {
    // remember to edit the port later
    m_masterShooter = new TalonFX(-1);
    m_helperShooter = new TalonFX(-1);
  }

  public void move(double speed) {
    m_masterShooter.set(speed);
    m_helperShooter.set(speed);
  }
  
  public void stop() {
    m_masterShooter.stopMotor();
    m_helperShooter.stopMotor();
  }

  // for the move(double speed) command, change to any double within the range [0,1] for testing 
  public Command shoot() { return new RunCommand( () -> { move(1); } ).finallyDo(interrupted -> stop()); }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}