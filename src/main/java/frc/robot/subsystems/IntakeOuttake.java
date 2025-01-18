// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;

public class IntakeOuttake extends SubsystemBase {
  private PWM servoMoter;
  /** Creates a new IntakeOuttake. */
  public IntakeOuttake() {
    servoMoter = new PWM(0);
    servoMoter.setBoundsMicroseconds(1500, 1260, 1250, 1240, 1000);
  }

  public void outtakeOpen() {
    servoMoter.setPosition(0.5);
  }

  public void outtakeClose() {
    servoMoter.setPosition(-0.5);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
