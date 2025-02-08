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
        autoChooser.addOption("A to 6A", new Auto(swerve, vision, new ArrayList<String>(List.of("[6A]", "[6B]", "[5A]", "[5B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("A1 to 6A", new Auto(swerve, vision, new ArrayList<String>(List.of("[5A]", "[6B]", "[5A]", "[5B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("B to 6A", new Auto(swerve, vision, new ArrayList<String>(List.of("[1A]", "[1B]", "[5A]", "[5B]")), Constants.Positions.CAGE_B));
        SmartDashboard.putData(autoChooser);
    }
}
