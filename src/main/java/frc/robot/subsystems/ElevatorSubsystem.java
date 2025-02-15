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
import frc.robot.Constants.CANAssignments;
import frc.robot.Constants.ElevatorConstants;
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
  private final SparkMax leftElevatorMotor = new ImprovedCanSpark(CANAssignments.LEFT_ELEVATOR_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, null, SparkBaseConfig.IdleMode.kBrake);
  private final SparkMax rightElevatorMotor = new ImprovedCanSpark(CANAssignments.RIGHT_ELEVATOR_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, null, SparkBaseConfig.IdleMode.kBrake);
  // set up absolute encoders for elevator
  public final CANcoder absoluteEncoderLeft = new CANcoder(CANAssignments.CLIMB_ABSOLUTE_ENCODER_LEFT_ID);
  public final CANcoder absoluteEncoderRight = new CANcoder(CANAssignments.CLIMB_ABSOLUTE_ENCODER_RIGHT_ID);
  
  // set up relative encoders for elevator
  public RelativeEncoder relativeEncoderLeft = leftElevatorMotor.getEncoder();
  public RelativeEncoder relativeEncoderRight = rightElevatorMotor.getEncoder();

  //PID controllers 
  public PIDController pidControllerLeft = new PIDController(0, 0, 0);
  public PIDController pidControllerRight = new PIDController(0, 0, 0);

  // setpoint
  private double setPointRotations;

  public double metersToRotations(double setpointNumMeters){
    return setpointNumMeters * ElevatorConstants.ELEVATOR_GEAR_RATIO;

  }
  
  // create L(#) preset methods

  //Drive bace in 3.8cm off the ground  
  public void L1_Preset() {
    setPointRotations = metersToRotations(0.0); // 46cm >> 42.2cm >> 0.224m
  }
  public void L2_Preset() {
    setPointRotations = metersToRotations(0.7973);   // 81cm >> 77.2cm  >> 0.772m  
  }
  public void L3_Preset() {
    setPointRotations = metersToRotations(1.1973);  //121cm  >> 117.2cm  >>  1.172m
  }
  public void L4_Preset() {
    setPointRotations = metersToRotations(1.8173);  //183cm >>  179.3cm  >>   1.793m 
  }
  public void loadStation_Preset(){
    setPointRotations = metersToRotations(0.0); //needs to be found 
  }


  public ElevatorSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftElevatorMotor.setVoltage(pidControllerLeft.calculate(relativeEncoderLeft.getPosition(), setPointRotations) + feedforward.calculate(0.0));
    rightElevatorMotor.setVoltage(pidControllerRight.calculate(relativeEncoderRight.getPosition(), setPointRotations) + feedforward.calculate(0.0));
  }
}
