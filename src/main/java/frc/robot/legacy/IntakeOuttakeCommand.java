package frc.robot.legacy;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeOuttake;


public class IntakeOuttakeCommand extends Command {

    private final IntakeOuttake intakeOuttake;
    private final XboxController gunner;


    public IntakeOuttakeCommand(IntakeOuttake intakeOuttake, XboxController gunner) {
        this.intakeOuttake = intakeOuttake;
        this.gunner = gunner;
        addRequirements(intakeOuttake);
    }


    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        System.out.println("Executing intakeoutake command");
        if (gunner.getAButton()) {
            intakeOuttake.outtakeOpen();
        }

        if (gunner.getBButton()) {
            intakeOuttake.outtakeClose();
        }
    }


    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
