// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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
  
  // k values
  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
  ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

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
  public PIDController pidControllerLeft = new PIDController(0, 0, 0);
  public PIDController pidControllerRight = new PIDController(0, 0, 0);

  // setpoint
  private double setpointNum;
  
  // create L(#) preset methods
  public void L1_Preset() {
    setpointNum = 1.0;
  }
  public void L2_Preset() {
    setpointNum = 2.0;
  }
  public void L3_Preset() {
    setpointNum = 3.0;
  }
  public void L4_Preset() {
    setpointNum = 4.0;
  }


  public ElevatorSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftElevatorMotor.setVoltage(pidControllerLeft.calculate(relativeEncoderLeft.getPosition(), setpointNum) + feedforward.calculate(0.0));
    rightElevatorMotor.setVoltage(pidControllerRight.calculate(relativeEncoderRight.getPosition(), setpointNum) + feedforward.calculate(0.0));
  }
}
