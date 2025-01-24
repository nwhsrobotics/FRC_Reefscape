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
import frc.robot.commands.IntakeOuttakeCommand;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeOuttake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.Buttons;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem(); 

    private final IntakeOuttake intakeoutake = new IntakeOuttake();

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final Vision VisionSubsystem = new Vision();

    private final SendableChooser<Command> autoChooser;

    public XboxController driver = new XboxController(0);
    public static XboxController gunner = new XboxController(1);

    public RobotContainer() {

        ParallelCommandGroup autoInit = new ParallelCommandGroup(); // new ParallelCommandGroup((new InstantCommand(() -> wristSubsystem.ampPreset(), wristSubsystem), (new InstantCommand(() -> armSubsystem.underStage(), armSubsystem));

        NamedCommands.registerCommand("autoInit", autoInit);

        //INIT after registering named commands
        autoChooser = AutoBuilder.buildAutoChooser("[B]");


        new JoystickButton(driver, Buttons.MENU).onTrue(new InstantCommand(swerveSubsystem::zeroGyro, swerveSubsystem));
        new JoystickButton(driver, Buttons.VIEW).onTrue(new InstantCommand(swerveSubsystem::switchFR, swerveSubsystem));

        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driver));
        intakeoutake.setDefaultCommand(new IntakeOuttakeCommand(intakeoutake, gunner));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
