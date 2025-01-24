// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ImprovedCanSpark;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {
  
  // BREAKING CHANGE: null parameter is a bandaid fix - FIND OUT WHAT IT DOES
  private final SparkMax leftElevatorMotor = new ImprovedCanSpark(1, ImprovedCanSpark.MotorKind.NEO550, null, SparkBaseConfig.IdleMode.kBrake);
  private final SparkMax rightElevatorMotor = new ImprovedCanSpark(2, ImprovedCanSpark.MotorKind.NEO550, null, SparkBaseConfig.IdleMode.kBrake);


  public ElevatorSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
