// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class IntakeOuttake extends SubsystemBase {
  private Servo servoMoter; //Initiative variable for Servo object
  /** Creates a new IntakeOuttake. */
  public IntakeOuttake() {
    servoMoter = new Servo(0); // Initialize servo on PWM port 0
    
  }
  /** Opens the outake mechanism toa present position */

  public void outtakeOpen() {
    servoMoter.set(0.5);
    System.out.println("Outtakeopen"); // Mid-point or fully open
  }
  /** CLoses the outtake mechanism to a present position */

  public void outtakeClose() {
    servoMoter.set(0.0);
    System.out.println("Outtakeopen");  // Fully closed
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
