package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;


public class ClimbCommand extends Command {

  private final ClimbSubsystem climb;
  private XboxController gunner;

  public ClimbCommand(ClimbSubsystem climb, XboxController gunner) {
    this.climb = climb;
    this.gunner = gunner;
    addRequirements(climb); 
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
