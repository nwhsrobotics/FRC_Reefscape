// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SysIdConstants;
import frc.robot.util.ImprovedCanSpark;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class SysId extends SubsystemBase {
  private SparkMax sysMotorLeft;
  private SparkMax sysMotorRight;
  private RelativeEncoder sysMotorLeftEncoder;
  private final SparkBaseConfig sysMotorLeftConfig;
  
  private SysIdRoutine m_sysIdRoutine;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  
  /** Creates a new SysIdRoutine. */
  public SysId() {
    
    

    sysMotorLeftConfig = new SparkMaxConfig();
    sysMotorLeftConfig.encoder.positionConversionFactor(SysIdConstants.SYSIDENCOCERROT2METER);
    sysMotorLeftConfig.encoder.velocityConversionFactor(SysIdConstants.SYSIDENCODERMETERPERSECONDS);
    this.sysMotorLeft = new ImprovedCanSpark(41, ImprovedCanSpark.MotorKind.NEO, sysMotorLeftConfig, IdleMode.kBrake);
    sysMotorLeftEncoder = sysMotorLeft.getEncoder();


    //this.sysMotorRight = new SparkMax(80, MotorType.kBrushless);
//creates sysid routine in the actual code
    m_sysIdRoutine = new SysIdRoutine(
      //set ramp rate voltage and duration
        new SysIdRoutine.Config(),
        //lets you record the data and define the subsystem that sysid is in
                  new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              voltage -> {
                sysMotorLeft.setVoltage(voltage);
                //m_rightMotor.setVoltage(voltage);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("sysid-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            sysMotorLeft.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(sysMotorLeftEncoder.getPosition(), Meters))
                    .linearVelocity(
                      //CHANGE THE UNITS OF VELOCITY USING THE SETVELOCITYCONVERSIONFACTOR
                        m_velocity.mut_replace(sysMotorLeftEncoder.getVelocity() , MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                /*log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
                        */
              } 
              ,
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
  }

//running the quasistic test when executed
public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return m_sysIdRoutine.quasistatic(direction);
}
//running the dynamic test when executed
public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  return m_sysIdRoutine.dynamic(direction);
}

  @Override
  public void periodic() {
    System.out.println(sysMotorLeft.getAppliedOutput());
    System.out.println(RobotController.getBatteryVoltage());
    // This method will be called once per scheduler run
  }



}
