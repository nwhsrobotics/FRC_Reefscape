// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.SysId;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SysIDCommand extends Command {
  private SysId sysIdSubsystem;
  private XboxController gunner;
  /** Creates a new SysIDCommand. */
  public SysIDCommand(SysId sysIdSubsystem, XboxController gunner) {
    this.sysIdSubsystem  = sysIdSubsystem;
    this.gunner = gunner;
    addRequirements(sysIdSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(gunner.getAButton()){
      sysIdSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
      System.out.println("Button A pressed");
    }
    else if(gunner.getBButton()){
      sysIdSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse);
    }
    else if(gunner.getXButton()){
      sysIdSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward);
    }
    else if(gunner.getYButton()){
      sysIdSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
