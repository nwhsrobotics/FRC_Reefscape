package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class L2CMD extends Command {
  
  private final ElevatorSubsystem elevator;
  private XboxController gunner;

  public L2CMD(ElevatorSubsystem initElevator , XboxController initGunner ) {
    this.elevator = initElevator;
    this.gunner = initGunner; 
    addRequirements(elevator);
  }


  @Override
  public void initialize() {}

 
  @Override
  public void execute() {
    elevator.L2_Preset();
  }

 
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
