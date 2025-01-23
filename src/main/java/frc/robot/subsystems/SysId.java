// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class SysId extends SubsystemBase {
  private CANSparkMax Sysmotor;
  private SysIdRoutine m_sysIdRoutine;
  /** Creates a new SysIdRoutine. */
  public SysId() {
    this.Sysmotor = new CANSparkMax(1, MotorType.kBrushless);
//creates sysid routine in the actual code
    m_sysIdRoutine = new SysIdRoutine(
      //set ramp rate voltage and duration
        new SysIdRoutine.Config(),
        //lets you record the data and define the subsystem that sysid is in
        new SysIdRoutine.Mechanism(null, null, null)
        );
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
    // This method will be called once per scheduler run
  }



}
