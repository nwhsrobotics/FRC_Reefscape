// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralEjection extends SubsystemBase {

  private final Servo ejectorMotor; 
  /** Creates a new CoralEjection. */
  public CoralEjection() {
    ejectorMotor = new Servo(5);

  }

  public void eject(){
    ejectorMotor.set(0.67); // 120 degrees
  }

  public void home(){
    ejectorMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
