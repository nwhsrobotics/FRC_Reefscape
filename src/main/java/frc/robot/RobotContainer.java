package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Buttons;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final AutoAlign autoAlign = new AutoAlign(swerveSubsystem);
    public final AlgaeArm algaeArm = new AlgaeArm();
    public final IntakeOuttake intakeoutake = new IntakeOuttake();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final Field2d field;
    private final SendableChooser<Command> autoChooser;

    public final XboxController driver = new XboxController(0);
    public static final XboxController gunner = new XboxController(1);

    public RobotContainer() {
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogCurrentPoseCallback(pose -> field.setRobotPose(pose));
        PathPlannerLogging.setLogTargetPoseCallback(pose -> field.getObject("target pose").setPose(pose));
        PathPlannerLogging.setLogActivePathCallback(poses -> field.getObject("path").setPoses(poses));

        NamedCommands.registerCommand("LEDEleup", new InstantCommand(() -> LEDSubsystem.setStateUntil(LEDState.ELEUP, elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("LEDEledrop", new InstantCommand(() -> LEDSubsystem.setStateUntil(LEDState.ELEDROPPING, elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("LEDAutoAlign", new InstantCommand(() -> LEDSubsystem.setState(LEDState.AUTOALIGNRUNNING)));
        //NamedCommands.registerCommand("FPA", AutoNavigator.navigateTo(swerveSubsystem.getPose().nearest(Constants.Positions.allAutoPositions)));
        NamedCommands.registerCommand("ElevatorWait", NamedCommands.getCommand("LEDEleup").andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L4CORAL", new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem).andThen(NamedCommands.getCommand("ElevatorWait")));
        NamedCommands.registerCommand("L3CORAL", new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem).andThen(NamedCommands.getCommand("ElevatorWait")));
        NamedCommands.registerCommand("L2CORAL", new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem).andThen(NamedCommands.getCommand("ElevatorWait")));
        NamedCommands.registerCommand("L1CORAL", new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem).andThen(NamedCommands.getCommand("ElevatorWait")));
        NamedCommands.registerCommand("LoadStation", new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem).andThen(NamedCommands.getCommand("LEDEledrop")));
        NamedCommands.registerCommand("Intake", new WaitCommand(1));
        NamedCommands.registerCommand("OuttakeOpen", new InstantCommand(() -> intakeoutake.outtakeOpen(), intakeoutake));
        NamedCommands.registerCommand("OuttakeClose", new InstantCommand(() -> intakeoutake.outtakeClose(), intakeoutake));
        NamedCommands.registerCommand("Outtake", NamedCommands.getCommand("OuttakeOpen").andThen(new WaitCommand(0.7)).andThen(NamedCommands.getCommand("OuttakeClose")));
        NamedCommands.registerCommand("VibrateON", new InstantCommand(() -> gunner.setRumble(RumbleType.kBothRumble, 1)));
        NamedCommands.registerCommand("VibrateOFF", new InstantCommand(() -> gunner.setRumble(RumbleType.kBothRumble, 0)));
        NamedCommands.registerCommand("Vibration", new InstantCommand(() -> NamedCommands.getCommand("VibrateON").andThen(new WaitCommand(1)).andThen(NamedCommands.getCommand("VibrateOFF"))));
        NamedCommands.registerCommand("AlgaeHome", new InstantCommand(() -> algaeArm.Homeposition(), algaeArm));
        NamedCommands.registerCommand("AlgaeKnockout", new InstantCommand(() -> algaeArm.knockoutAlgae(), algaeArm));
        NamedCommands.registerCommand("AlgaeRemovalAuto", NamedCommands.getCommand("L1CORAL").alongWith(NamedCommands.getCommand("AlgaeKnockout")).andThen(NamedCommands.getCommand("AlgaeHome")).andThen(NamedCommands.getCommand("LoadStation")));
        NamedCommands.registerCommand("LeftReefAuto", new InstantCommand(() -> autoAlign.navigateTo(autoAlign.leftReef(swerveSubsystem.getPose()))).alongWith(NamedCommands.getCommand("LEDAutoAlign")));
        NamedCommands.registerCommand("RightReefAuto", new InstantCommand(() -> autoAlign.navigateTo(autoAlign.rightReef(swerveSubsystem.getPose()))).alongWith(NamedCommands.getCommand("LEDAutoAlign")));

        // Control diagram: https://docs.google.com/drawings/d/1NsJOx6fb6   KYHW6L8ZeuNtpK3clnQnIA9CD2kQHFL0P0/edit?usp=sharing
        new POVButton(gunner, Buttons.POV_UP).onTrue(NamedCommands.getCommand("AlgaeHome"));
        new POVButton(gunner, Buttons.POV_DOWN).onTrue(NamedCommands.getCommand("AlgaeKnockout"));
        new JoystickButton(gunner, Buttons.Y).onTrue(NamedCommands.getCommand("L1CORAL"));
        new JoystickButton(gunner, Buttons.B).onTrue(NamedCommands.getCommand("L2CORAL"));
        new JoystickButton(gunner, Buttons.A).onTrue(NamedCommands.getCommand("L3CORAL"));
        new JoystickButton(gunner, Buttons.X).onTrue(NamedCommands.getCommand("L4CORAL"));
        new JoystickButton(gunner, Buttons.RIGHT_STICK_BUTTON).onTrue(NamedCommands.getCommand("LoadStation"));
        new POVButton(gunner, Buttons.POV_LEFT).onTrue(NamedCommands.getCommand("AlgaeRemovalAuto"));
        new JoystickButton(gunner, Buttons.RIGHT_BUMPER).whileTrue(NamedCommands.getCommand("OuttakeOpen").onlyIf(() -> !intakeoutake.isIntakeOpen));
        new JoystickButton(gunner, Buttons.RIGHT_BUMPER).onFalse(NamedCommands.getCommand("OuttakeClose").andThen(NamedCommands.getCommand("Vibration")));
        new POVButton(gunner, Buttons.POV_RIGHT).onTrue(new InstantCommand(() -> elevatorSubsystem.elevator_top()));
        new JoystickButton(gunner, Buttons.LEFT_BUMPER).onTrue(new InstantCommand(() -> elevatorSubsystem.elevator_zero()));
        new JoystickButton(driver, Buttons.MENU).onTrue(new InstantCommand(swerveSubsystem::switchFR, swerveSubsystem));
        new POVButton(driver, Buttons.POV_DOWN).onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometryWithVision()));
        new JoystickButton(driver, Buttons.X).onTrue(NamedCommands.getCommand("LeftReefAuto").andThen(NamedCommands.getCommand("Vibration")));
        new JoystickButton(driver, Buttons.B).onTrue(NamedCommands.getCommand("RightReefAuto").andThen(NamedCommands.getCommand("Vibration")));

        autoChooser = AutoBuilder.buildAutoChooser("Straight Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driver));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }


}
