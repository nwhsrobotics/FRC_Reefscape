// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class stationSelector extends Command {

private final SwerveSubsystem swerve;
private final XboxController drive;

  /** Creates a new stationSelector. */
  public stationSelector(SwerveSubsystem swerve, XboxController drive ) {
    this.swerve = swerve;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drive.getPOV() == 0){
      swerve.autonavigator.navigateTo(Constants.Positions.STATION_LEFT);
    }
    if(drive.getLeftTriggerAxis() >= 0.2 && drive.getPOV() == 0){
      swerve.autonavigator.navigateTo(Constants.Positions.STATION_RIGHT);
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
