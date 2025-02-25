package frc.robot;


import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Positions;
import frc.robot.autos.Auto;
import frc.robot.commands.IntakeOuttakeCommand;
import frc.robot.commands.L1CMD;
import frc.robot.commands.L2CMD;
import frc.robot.commands.L3CMD;
import frc.robot.commands.L4CMD;
import frc.robot.commands.LoadStation;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeOuttake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SysId;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Buttons;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    //private final ClimbSubsystem climbSubsystem = new ClimbSubsystem(); 

    private final IntakeOuttake intakeoutake = new IntakeOuttake();

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final VisionSubsystem limeLightForwards = new VisionSubsystem(Constants.LimelightConstants.llLocalizationNameForwards);

    public final SysId sysIdSubsystem = new SysId();
    private final VisionSubsystem limeLightBackwards = new VisionSubsystem(Constants.LimelightConstants.llLocalizationNameBackwards);

    

    private final SendableChooser<Command> autoChooser;

    public XboxController driver = new XboxController(0);
    public static XboxController gunner = new XboxController(1);

    public RobotContainer() {

        ParallelCommandGroup autoInit = new ParallelCommandGroup(); // new ParallelCommandGroup((new InstantCommand(() -> wristSubsystem.ampPreset(), wristSubsystem), (new InstantCommand(() -> armSubsystem.underStage(), armSubsystem));

        Command L4CMD = new L4CMD(elevatorSubsystem, gunner);
        Command L3CMD = new L3CMD(elevatorSubsystem, gunner);
        Command L2CMD = new L2CMD(elevatorSubsystem, gunner);
        Command L1CMD = new L1CMD(elevatorSubsystem, gunner);
        Command LoadStation = new LoadStation(elevatorSubsystem, gunner);

        NamedCommands.registerCommand("autoInit", autoInit);

        NamedCommands.registerCommand("L4CORAL",L4CMD);
        NamedCommands.registerCommand("L3CORAL",L3CMD);
        NamedCommands.registerCommand("L2CORAL",L2CMD);
        NamedCommands.registerCommand("L1CORAL",L1CMD);
        NamedCommands.registerCommand("LoadStation",LoadStation);


        //INIT after registering named commands
        NamedCommands.registerCommand("Intake", new InstantCommand());
        NamedCommands.registerCommand("Outtake", new InstantCommand());
        NamedCommands.registerCommand("MoveElevator", new InstantCommand());
        autoChooser = AutoBuilder.buildAutoChooser(); 
        //TODO: Move autochooser here
        autoChooser.addOption("A to 6A", new Auto(swerveSubsystem, limeLightForwards, limeLightBackwards, new ArrayList<String>(List.of("[6A]", "[6B]", "[5A]", "[5B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("A1 to 6A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<String>(List.of("[5A]", "[6B]", "[5A]", "[5B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("A to 5A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<String>(List.of("[5A]", "[5B]", "[4A]", "[4B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("A1 to 5A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<String>(List.of("[4A]", "[5B]", "[4A]", "[4B]")), Constants.Positions.CAGE_A));
        autoChooser.addOption("B to 6A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<String>(List.of("[1A]", "[1B]", "[5A]", "[5B]")), Constants.Positions.CAGE_B));
        autoChooser.addOption("B1 to 6A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<String>(List.of("[1A]", "[5B]", "[1A]", "[5B]")), Constants.Positions.CAGE_B));
        autoChooser.addOption("C to 4A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<String>(List.of("[4A]", "[4B]", "[3A]", "[3B]")), Constants.Positions.CAGE_C));
        autoChooser.addOption("C1 to 4A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<String>(List.of("[3A]", "[4B]", "[3A]", "[3B]")), Constants.Positions.CAGE_C));


        //Gunner controlls 
        new JoystickButton(gunner, Buttons.POV_UP).onTrue(new InstantCommand(() -> elevatorSubsystem.increaseCurrentLevel(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.POV_DOWN).onTrue(new InstantCommand(() -> elevatorSubsystem.decreaseCurrentLevel(), elevatorSubsystem));

        new JoystickButton(gunner, Buttons.RIGHT_BUMPER).onTrue(new InstantCommand(() -> intakeoutake.outtakeOpen(), intakeoutake));
        new JoystickButton(gunner, Buttons.LEFT_BUMPER).onTrue(new InstantCommand(() -> intakeoutake.outtakeClose(), intakeoutake));
        
        //Driver controlls 
        new JoystickButton(driver, Buttons.MENU).onTrue(new InstantCommand(swerveSubsystem::zeroGyro, swerveSubsystem));
        new JoystickButton(driver, Buttons.VIEW).onTrue(new InstantCommand(swerveSubsystem::switchFR, swerveSubsystem));
        // NEED TO ADD THE FAST/SLOW MODE TOGGLE (see controller diagram)
        // NEED TO ADD THE STATION 1/2 TOGGLE THING (see controller diagram)
        
        new JoystickButton(driver, Buttons.RIGHT_STICK_BUTTON).onTrue(new InstantCommand(swerveSubsystem.autonavigator::toggle));
        new JoystickButton(driver, Buttons.X).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_LEFT_REEF)));
        new JoystickButton(driver, Buttons.Y).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_MID_REEF)));
        new JoystickButton(driver, Buttons.A).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.BACK_MID_REEF)));
        new JoystickButton(driver, Buttons.B).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_RIGHT_REEF)));
        //TODO: This can work
        new JoystickButton(driver, Buttons.LEFT_BUMPER).whileTrue(AutoBuilder.pathfindToPose(Positions.BACK_LEFT_REEF, AutoConstants.kPathfindingConstraints, 0.0));
        
        new JoystickButton(gunner, Buttons.X).onTrue(sysIdSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        new JoystickButton(gunner, Buttons.Y).onTrue(sysIdSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        new JoystickButton(gunner, Buttons.A).onTrue(sysIdSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        new JoystickButton(gunner, Buttons.B).onTrue(sysIdSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));


        new JoystickButton(driver, Buttons.POV_RIGHT).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.BACK_RIGHT_REEF)));
        new JoystickButton(driver, Buttons.POV_LEFT).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_LEFT_REEF)));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        

        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driver));
        //intakeoutake.setDefaultCommand(new IntakeOuttakeCommand(intakeoutake, gunner));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}