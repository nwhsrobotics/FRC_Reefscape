package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class L1CMD extends Command {

  private final ElevatorSubsystem elevator;
  private XboxController gunner;

  public L1CMD(ElevatorSubsystem initElevator , XboxController initGunner ) {
    this.elevator = initElevator;
    this.gunner = initGunner; 
    addRequirements(elevator);
  }

  //TODO: Move this method in the elevator subsystem
  // so don't need to repeatedly change/copy paste for all commands
  public boolean isAtLocation(){
    
    //TODO: There is a logic error here (units), its meters-rotations, should either be meters-meters or rotations-rotations
    // or use setPositionConversionFactor
    if (elevator.setPointRotations == 0.0){
      return ((elevator.rotationsToMeters(elevator.setPointRotations)-elevator.relativeEncoderLeft.getPosition())>-0.2);
      
    }
    return (Math.abs(elevator.rotationsToMeters(elevator.setPointRotations)-elevator.relativeEncoderLeft.getPosition())<0.2);

  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    elevator.L1_Preset();
  }

 
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return isAtLocation();
  }
}
