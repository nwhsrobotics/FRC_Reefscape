package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeOuttake;


public class IntakeOuttakeCommand extends Command {

  private IntakeOuttake intakeOuttake;
  private XboxController gunner;



  public IntakeOuttakeCommand(IntakeOuttake intakeOuttake, XboxController gunner) {
    this.intakeOuttake = intakeOuttake;
    this.gunner = gunner;
    addRequirements(intakeOuttake);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
