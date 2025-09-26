// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANAssignments;
import frc.robot.util.RobotCANUtils.CANSparkMaxController;
import frc.robot.util.RobotCANUtils.MotorKind;


 
public class GroundIntakeSubsystem extends SubsystemBase {

  private final CANSparkMaxController LeftGroundIntakeMotor;
  private final CANSparkMaxController RightGroundIntakeMotor;
  private final CANSparkMaxController GroundPivot;





  public GroundIntakeSubsystem() {

    LeftGroundIntakeMotor = new CANSparkMaxController(CANAssignments.LEFT_GROUND_INTAKE_MOTOR_ID, MotorKind.NEO30AMP, IdleMode.kBrake); //Change the motor ID
    RightGroundIntakeMotor = new CANSparkMaxController(CANAssignments.RIGHT_GROUND_INTAKE_MOTOR_ID, MotorKind.NEO30AMP, IdleMode.kBrake); // Chcange the motor ID
    
  }

  public void Intake(){
    LeftGroundIntakeMotor.set(0.5);
    RightGroundIntakeMotor.set(-0.5);
  }
  public void Outtake(){
    LeftGroundIntakeMotor.set(-0.5);
    RightGroundIntakeMotor.set(0.5);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
  */