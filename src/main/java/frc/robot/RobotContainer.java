package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Positions;
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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Buttons;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem(); 

    private final IntakeOuttake intakeoutake = new IntakeOuttake();

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final VisionSubsystem limeLightForwards = new VisionSubsystem("limelight1");
    private final VisionSubsystem limeLightBackwards = new VisionSubsystem("limelight2");


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
        autoChooser = AutoBuilder.buildAutoChooser();


        new JoystickButton(driver, Buttons.MENU).onTrue(new InstantCommand(swerveSubsystem::zeroGyro, swerveSubsystem));
        new JoystickButton(driver, Buttons.VIEW).onTrue(new InstantCommand(swerveSubsystem::switchFR, swerveSubsystem));

        // NEED TO ADD THE FAST/SLOW MODE TOGGLE (see controller diagram)
        // NEED TO ADD THE STATION 1/2 TOGGLE THING (see controller diagram)
        new JoystickButton(driver, Buttons.RIGHT_STICK_BUTTON).onTrue(new InstantCommand(swerveSubsystem.autonavigator::toggle, swerveSubsystem));
        new JoystickButton(driver, Buttons.X).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_LEFT_REEF), swerveSubsystem));
        new JoystickButton(driver, Buttons.Y).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_MID_REEF), swerveSubsystem));
        new JoystickButton(driver, Buttons.A).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.BACK_MID_REEF), swerveSubsystem));
        new JoystickButton(driver, Buttons.B).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_RIGHT_REEF), swerveSubsystem));
        new JoystickButton(driver, Buttons.POV_RIGHT).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.BACK_RIGHT_REEF), swerveSubsystem));
        new JoystickButton(driver, Buttons.POV_LEFT).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_LEFT_REEF), swerveSubsystem));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driver));
        intakeoutake.setDefaultCommand(new IntakeOuttakeCommand(intakeoutake, gunner));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
