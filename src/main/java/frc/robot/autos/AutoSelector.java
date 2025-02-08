package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class AutoSelector {
    public AutoSelector (SwerveSubsystem swerve, Vision vision ){
        SendableChooser<Command> autoChooser = new SendableChooser<>();
        autoChooser.addOption("A -> 5&6", new Auto(swerve, vision, new ArrayList<String>(List.of("[6B]", "[5B]", "[6A]", "[5A]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("B -> 1", new Auto(swerve, vision, new ArrayList<String>(List.of("[1B]", "[1A]", "[1B]")), Constants.Positions.CAGE_B));
        autoChooser.addOption("C -> 3&4", new Auto(swerve, vision, new ArrayList<String>(List.of("[3A]", "[3B]", "[4A]", "[4B]")), Constants.Positions.CAGE_C));
        SmartDashboard.putData(autoChooser);
    }
}
