package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.*;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public final AlgaeArm algaeArm = new AlgaeArm();

    public final IntakeOuttake intakeoutake = new IntakeOuttake();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    //public final LEDSubsystem led_sybsystem = new LEDSubsystem();

    public final VisionSubsystem limeLightForwards = new VisionSubsystem(LimelightConstants.llFront);

    public final VisionSubsystem limeLightBackwards = new VisionSubsystem(LimelightConstants.llBack);

    private final Field2d field;
    private final SendableChooser<Command> autoChooser;

    public XboxController driver = new XboxController(0);
    public static XboxController gunner = new XboxController(1);

    public RobotContainer() {
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        NamedCommands.registerCommand("finalPreciseAlignment", swerveSubsystem.autonavigator.finalPreciseAllingment(false));
        NamedCommands.registerCommand("L4CORAL", new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L3CORAL", new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L2CORAL", new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L1CORAL", new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("LoadStation", new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem));
        NamedCommands.registerCommand("Boost", new InstantCommand(() -> elevatorSubsystem.boost(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("Intake", new WaitCommand(1));
        //               .andThen(new InstantCommand(() -> recordAttempt()))
        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> intakeoutake.outtakeOpen(), intakeoutake)
                .andThen(new WaitCommand(0.7))
                .andThen(new InstantCommand(() -> intakeoutake.outtakeClose(), intakeoutake)));

        autoChooser = AutoBuilder.buildAutoChooser("Straight Auto");

        // Control diagram: https://docs.google.com/drawings/d/1NsJOx6fb6KYHW6L8ZeuNtpK3clnQnIA9CD2kQHFL0P0/edit?usp=sharing
        new POVButton(gunner, Buttons.POV_UP).onTrue(new InstantCommand(() -> algaeArm.Homeposition(), algaeArm));
        new POVButton(gunner, Buttons.POV_DOWN).onTrue(new InstantCommand(() -> algaeArm.knockoutAlgae(), algaeArm));
        new JoystickButton(gunner, Buttons.Y).onTrue(new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem).andThen(new InstantCommand(() -> LEDSubsystem.state = LEDSubsystem.LEDState.ELEUP)));
        new JoystickButton(gunner, Buttons.B).onTrue(new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem).andThen(new InstantCommand(() -> LEDSubsystem.state = LEDSubsystem.LEDState.ELEUP)));
        new JoystickButton(gunner, Buttons.A).onTrue(new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem).andThen(new InstantCommand(() -> LEDSubsystem.state = LEDSubsystem.LEDState.ELEUP)));
        new JoystickButton(gunner, Buttons.X).onTrue(new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.RIGHT_STICK_BUTTON).onTrue(new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem));
        new POVButton(gunner, Buttons.POV_LEFT).onTrue((NamedCommands.getCommand("L1CORAL")
                .alongWith(new InstantCommand(() -> algaeArm.knockoutAlgae(), algaeArm)))
                .andThen(new InstantCommand(() -> algaeArm.Homeposition(), algaeArm))
                .andThen(NamedCommands.getCommand("LoadStation")));
        new JoystickButton(gunner, Buttons.RIGHT_BUMPER).whileTrue(((new InstantCommand(() -> intakeoutake.outtakeOpen(), intakeoutake))).onlyIf(() -> !intakeoutake.isIntakeOpen));
        new JoystickButton(gunner, Buttons.RIGHT_BUMPER).onFalse(new InstantCommand(() -> intakeoutake.outtakeClose(), intakeoutake).andThen(
                new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1))
                        .andThen(new WaitCommand(1))
                        .andThen(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0)))
        ));
        new POVButton(gunner, Buttons.POV_RIGHT).onTrue(new InstantCommand(() -> elevatorSubsystem.elevator_top()));
        new JoystickButton(gunner, Buttons.LEFT_BUMPER).onTrue(new InstantCommand(() -> elevatorSubsystem.elevator_zero()));
        // don't need reset odometry with vision because you can fake angle it (for rotation) + the april tag corrects odometry live
        new JoystickButton(driver, Buttons.VIEW).onTrue(new InstantCommand(swerveSubsystem::switchFR, swerveSubsystem));
        // new POVButton(driver, Buttons.POV_DOWN).onTrue(new InstantCommand(()
        //         -> swerveSubsystem.resetOdometry(new Pose2d(swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY(), limeLightForwards.getFakeAngle(swerveSubsystem.getPose())))));
        new POVButton(driver, Buttons.POV_DOWN).onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometryWithVision()));
        // new POVButton(driver, Buttons.POV_UP).onTrue(new InstantCommand(()
        //         -> swerveSubsystem.resetOdometry(new Pose2d(3.829, 5.143, Rotation2d.fromDegrees(-60)))));

        new JoystickButton(driver, Buttons.X).onTrue(
                new InstantCommand(() -> {
                    Pose2d target = limeLightForwards.leftReef(swerveSubsystem.getPose());
                    swerveSubsystem.autonavigator.navigateTo(target);
                }).andThen(
                        new InstantCommand(() -> gunner.setRumble(RumbleType.kBothRumble, 1))
                                .andThen(new InstantCommand(() -> LEDSubsystem.state = LEDSubsystem.LEDState.AUTOALINERUNNING))
                                .andThen(new WaitCommand(1))
                                .andThen(new InstantCommand(() -> gunner.setRumble(RumbleType.kBothRumble, 0)))
                )
        );

        new JoystickButton(driver, Buttons.B).onTrue(
                new InstantCommand(() -> {
                    Pose2d target = limeLightForwards.rightReef(swerveSubsystem.getPose());
                    swerveSubsystem.autonavigator.navigateTo(target);
                }).andThen(
                        new InstantCommand(() -> gunner.setRumble(RumbleType.kBothRumble, 1))
                                .andThen(new InstantCommand(() -> LEDSubsystem.state = LEDSubsystem.LEDState.AUTOALINERUNNING))
                                .andThen(new WaitCommand(1))
                                .andThen(new InstantCommand(() -> gunner.setRumble(RumbleType.kBothRumble, 0)))
                )
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driver));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }


}
