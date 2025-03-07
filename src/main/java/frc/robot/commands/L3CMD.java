package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class L3CMD extends Command {

  private final ElevatorSubsystem elevator;
  private XboxController gunner;

  public L3CMD(ElevatorSubsystem initElevator , XboxController initGunner ) {
    this.elevator = initElevator;
    this.gunner = initGunner; 
    addRequirements(elevator);
  }

  public boolean isAtLocation(){
    
    if (elevator.setPointRotations == 0.0){
      return ((elevator.rotationsToMeters(elevator.setPointRotations)-elevator.relativeEncoderLeft.getPosition())>-0.2);
      
    }
    return (Math.abs(elevator.rotationsToMeters(elevator.setPointRotations)-elevator.relativeEncoderLeft.getPosition())<0.2);

  }

  @Override
  public void initialize() {
    elevator.L3_Preset();
  }

  
  @Override
  public void execute() {
    //elevator.L3_Preset();
  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return elevator.isNearTargetPosition();
  }
}
