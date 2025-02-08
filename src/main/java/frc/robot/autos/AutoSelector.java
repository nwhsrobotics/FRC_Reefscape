package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    public AutoSelector (){
        SendableChooser<Command> autoChooser = new SendableChooser<>();
        autoChooser.addOption("A to 6A", new Auto(, null, null, null, 0, null))
        SmartDashboard.putData(autoChooser);
    }
}
