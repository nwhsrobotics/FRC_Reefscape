package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoSelector {
    public AutoSelector (SwerveSubsystem swerve, VisionSubsystem vision, VisionSubsystem vision1){
        //TODO: Add this code in RobotContainer.java and use the same autochooser (switch between pathplanner and this/manually comment out)
        SendableChooser<Command> autoChooser = new SendableChooser<>();
        autoChooser.addOption("A to 6A", new Auto(swerve, vision, vision1, new ArrayList<String>(List.of("[6A]", "[6B]", "[5A]", "[5B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("A1 to 6A", new Auto(swerve, vision,vision1, new ArrayList<String>(List.of("[5A]", "[6B]", "[5A]", "[5B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("A to 5A", new Auto(swerve, vision,vision1, new ArrayList<String>(List.of("[5A]", "[5B]", "[4A]", "[4B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("A1 to 5A", new Auto(swerve, vision,vision1, new ArrayList<String>(List.of("[4A]", "[5B]", "[4A]", "[4B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("B to 6A", new Auto(swerve, vision,vision1, new ArrayList<String>(List.of("[1A]", "[1B]", "[5A]", "[5B]")), Constants.Positions.CAGE_B));
        autoChooser.addOption("B1 to 6A", new Auto(swerve, vision,vision1, new ArrayList<String>(List.of("[1A]", "[5B]", "[1A]", "[5B]")), Constants.Positions.CAGE_B));
        autoChooser.addOption("C to 4A", new Auto(swerve, vision,vision1, new ArrayList<String>(List.of("[4A]", "[4B]", "[3A]", "[3B]")), Constants.Positions.CAGE_C));
        autoChooser.addOption("C1 to 4A", new Auto(swerve, vision,vision1, new ArrayList<String>(List.of("[3A]", "[4B]", "[3A]", "[3B]")), Constants.Positions.CAGE_C));

        SmartDashboard.putData(autoChooser);
    }
}
