package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorCommand extends Command {
  
  private ElevatorSubsystem elevator;
  private XboxController gunner;


  public ElevatorCommand(ElevatorSubsystem elevator, XboxController gunner) {
    this.elevator = elevator;
    this.gunner = gunner;
    addRequirements(elevator);
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
