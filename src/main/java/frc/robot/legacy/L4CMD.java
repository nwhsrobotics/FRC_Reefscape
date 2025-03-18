package frc.robot.legacy;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class L4CMD extends Command {

    private final ElevatorSubsystem elevator;
    private final XboxController gunner;

    public L4CMD(ElevatorSubsystem initElevator, XboxController initGunner) {
        this.elevator = initElevator;
        this.gunner = initGunner;
        addRequirements(elevator);
    }

    public boolean isAtLocation() {

        if (elevator.setPointRotations == 0.0) {
            return ((elevator.rotationsToMeters(elevator.setPointRotations) - elevator.relativeEncoderLeft.getPosition()) > -0.2);

        }
        return (Math.abs(elevator.rotationsToMeters(elevator.setPointRotations) - elevator.relativeEncoderLeft.getPosition()) < 0.2);

    }

    @Override
    public void initialize() {
        elevator.L4_Preset();
    }


    @Override
    public void execute() {
        //elevator.L4_Preset();
    }


    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return elevator.isNearTargetPosition();
    }
}
