// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeOuttake extends SubsystemBase {
  private Servo servoMoter; //Initiative variable for Servo object
  private boolean isIntakeOpen; 
  


  public IntakeOuttake() {
    servoMoter = new Servo(0); // Initialize servo on PWM port 0
    isIntakeOpen = false; 
    SmartDashboard.putBoolean("Is Intake Open", isIntakeOpen);
    // Logger.recordOutput("Is On", isIntakeOpen);
  }
  /** Opens the outake mechanism toa present position */

  public void outtakeOpen() {
    if(!isIntakeOpen){
      isIntakeOpen = true;
      servoMoter.set(0.3);
      System.out.println("Outtakeopen"); //open 90 dagrees 
    }else{
      System.out.println("Outtake already open");
    }
    
  }
  /** CLoses the outtake mechanism to a present position */
  public void outtakeClose() {
    if(isIntakeOpen){
      isIntakeOpen = false;
      servoMoter.set(0.0);
      System.out.println("Outtakeclosed");
    }else{
      System.out.println("Outtake already closed");
    }
      
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Is Intake Open", isIntakeOpen);
  }
}
