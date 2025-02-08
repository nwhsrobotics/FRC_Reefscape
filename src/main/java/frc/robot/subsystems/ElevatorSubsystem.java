// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ImprovedCanSpark;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {

  private double kS = 0.0;
  private double kG = 0.0;  
  private double kV = 0.0; 
  private double kA = 0.0;  
  
  // Create ele(vator) motors
  private final SparkMax leftElevatorMotor = new ImprovedCanSpark(25, ImprovedCanSpark.MotorKind.NEO550, null, SparkBaseConfig.IdleMode.kBrake);
  private final SparkMax rightElevatorMotor = new ImprovedCanSpark(26, ImprovedCanSpark.MotorKind.NEO550, null, SparkBaseConfig.IdleMode.kBrake);
  // set up absolute encoders for elevator
  public final CANcoder absoluteEncoderLeft = new CANcoder(27);
  public final CANcoder absoluteEncoderRight = new CANcoder(28);
  
  // set up relative encoders for elevator
  public RelativeEncoder relativeEncoderLeft = leftElevatorMotor.getEncoder();
  public RelativeEncoder relativeEncoderRight = rightElevatorMotor.getEncoder();

  //PID controllers 
  public PIDController pigControllerLeft = new PIDController(0, 0, 0); 
  public PIDController pigControllerRight = new PIDController(0, 0, 0);
  
  //Create ElevatorFeedForward
  


  public ElevatorSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftElevatorMotor.setVoltage(pigControllerLeft.calculate(relativeEncoderLeft.getPosition(), setpoint) + feedforward);
    rightElevatorMotor.setVoltage(pigControllerRight.calculate(relativeEncoderRight.getPosition(), setpoint) + feedforward);
  }
}
