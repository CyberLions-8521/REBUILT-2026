// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.Shooter;


public class Shooter extends SubsystemBase {

  private SparkMax m_shooter;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooter = new SparkMax(0, MotorType.kBrushless);
  }

  public void move() { m_shooter.set(1); }
  public void stop() { m_shooter.stopMotor(); }

  public Command shoot() { return new RunCommand( () -> { move(); } ).finallyDo(interrupted -> stop()); }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}