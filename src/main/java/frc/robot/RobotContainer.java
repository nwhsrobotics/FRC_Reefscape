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
import frc.robot.Constants.AprilTagOffsets;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TagOffset;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeOuttake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Buttons;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public int coralsAttempted = 0;
    public int coralsScored = 0;

    private final AlgaeArm algaeArm = new AlgaeArm();

    public final IntakeOuttake intakeoutake = new IntakeOuttake();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    //public final LED_Subsystem led_sybsystem = new LED_Subsystem();

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

        NamedCommands.registerCommand("L4CORAL", new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L3CORAL", new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L2CORAL", new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L1CORAL", new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("LoadStation", new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem));
        NamedCommands.registerCommand("Boost", new InstantCommand(() -> elevatorSubsystem.boost(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("Intake", new WaitCommand(0.75));
        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> intakeoutake.outtakeOpen(), intakeoutake)
                .andThen(new InstantCommand(() -> recordAttempt()))
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(() -> intakeoutake.outtakeClose(), intakeoutake)));

        autoChooser = AutoBuilder.buildAutoChooser("Straight Auto");

        // Control diagram: https://docs.google.com/drawings/d/1NsJOx6fb6KYHW6L8ZeuNtpK3clnQnIA9CD2kQHFL0P0/edit?usp=sharing
        new JoystickButton(gunner, Buttons.POV_UP).onTrue(new InstantCommand(() -> algaeArm.knockoutAlgae(), algaeArm));
        new JoystickButton(gunner, Buttons.POV_DOWN).onTrue(new InstantCommand(() -> algaeArm.Homeposition(), algaeArm));
        new JoystickButton(gunner, Buttons.Y).onTrue(new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.B).onTrue(new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.A).onTrue(new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.X).onTrue(new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem));
        new POVButton(gunner, Buttons.POV_LEFT).onTrue(new InstantCommand(() -> elevatorSubsystem.dynamic_L4_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.RIGHT_STICK_BUTTON).onTrue(new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.RIGHT_BUMPER).whileTrue(((new InstantCommand(() -> recordAttempt())).andThen(new InstantCommand(() -> intakeoutake.outtakeOpen(), intakeoutake))).onlyIf(() -> !intakeoutake.isIntakeOpen));
        new JoystickButton(gunner, Buttons.RIGHT_BUMPER).onFalse(new InstantCommand(() -> intakeoutake.outtakeClose(), intakeoutake).andThen(
                new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1))
                        .andThen(new WaitCommand(1))
                        .andThen(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0)))
        ));
        new JoystickButton(gunner, Buttons.LEFT_BUMPER).onTrue(NamedCommands.getCommand("L4CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("Boost")).andThen(NamedCommands.getCommand("LoadStation")));
        //new POVButton(gunner, Buttons.POV_UP).onTrue(NamedCommands.getCommand("L1CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("LoadStation")));
        new POVButton(gunner, Buttons.POV_RIGHT).onTrue(NamedCommands.getCommand("L2CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("LoadStation")));
        //new POVButton(gunner, Buttons.POV_DOWN).onTrue(NamedCommands.getCommand("L3CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("LoadStation")));
        new JoystickButton(gunner, Buttons.MENU).onTrue(new InstantCommand(() -> successfulAttempt()));
        new JoystickButton(gunner, Buttons.VIEW).onTrue(new InstantCommand(() -> successfulAttempt()));

        // don't need reset odometry with vision because you can fake angle it (for rotation) + the april tag corrects odometry live
        new JoystickButton(driver, Buttons.VIEW).onTrue(new InstantCommand(swerveSubsystem::switchFR, swerveSubsystem));
        new POVButton(driver, Buttons.POV_DOWN).onTrue(new InstantCommand(()
                -> swerveSubsystem.resetOdometry(new Pose2d(swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY(), limeLightForwards.getFakeAngle(swerveSubsystem.getPose())))));
        // new POVButton(driver, Buttons.POV_UP).onTrue(new InstantCommand(()
        //         -> swerveSubsystem.resetOdometry(new Pose2d(3.829, 5.143, Rotation2d.fromDegrees(-60)))));

        new JoystickButton(driver, Buttons.X).onTrue(
                new InstantCommand(() -> {
                    Pose2d target = limeLightForwards.leftReef(swerveSubsystem.getPose());
                    swerveSubsystem.autonavigator.navigateTo(target);
                }).andThen(
                        new InstantCommand(() -> gunner.setRumble(RumbleType.kBothRumble, 1))
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
                                .andThen(new WaitCommand(1))
                                .andThen(new InstantCommand(() -> gunner.setRumble(RumbleType.kBothRumble, 0)))
                )
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driver));
    }

    public Command getAutonomousCommand() {
        //return new AlternativePathfindAprilTag(1, swerveSubsystem, limeLightForwards, "");
        return autoChooser.getSelected();

    }

    public void recordAttempt() {
        Pose2d currentPose = swerveSubsystem.getPose();
        int currentTagId = limeLightForwards.getCurrentDetectedAprilTag(currentPose);
        String alignmentDir = limeLightForwards.getCurrentAlignment(currentPose);
        double offsetY = limeLightForwards.getOffsetY(currentPose);
        double offsetX = limeLightForwards.getOffsetX(currentPose);
        boolean isRight = "RIGHT".equals(alignmentDir);
        TagOffset offset = AprilTagOffsets.getOffset(currentTagId);
        double offsetYDifference = isRight ? (offset.right - offsetY) : (offset.left - offsetY);
        double offsetXDifference = offset.back - offsetX;

        Logger.recordOutput("attempt." + coralsAttempted + ".swerve", currentPose);
        Logger.recordOutput("attempt." + coralsAttempted + ".aprilTag", currentTagId);
        SmartDashboard.putNumber("attempt." + coralsAttempted + ".aprilTag", currentTagId);
        Logger.recordOutput("attempt." + coralsAttempted + ".alignmentDirection", alignmentDir);

        Logger.recordOutput("attempt." + coralsAttempted + ".offsetYRelative", offsetY);
        Logger.recordOutput("attempt." + coralsAttempted + ".offsetXRelative", offsetX);
        SmartDashboard.putNumber("attempt." + coralsAttempted + ".offsetYRelative", offsetY);
        SmartDashboard.putNumber("attempt." + coralsAttempted + ".offsetXRelative", offsetX);

        Logger.recordOutput("attempt." + coralsAttempted + ".isAllignRight", isRight);
        SmartDashboard.putBoolean("attempt." + coralsAttempted + ".isAllignRight", isRight);

        Logger.recordOutput("attempt." + coralsAttempted + ".offsetYRelativeDifference", offsetYDifference);
        Logger.recordOutput("attempt." + coralsAttempted + ".offsetXRelativeDifference", offsetXDifference);
        SmartDashboard.putNumber("attempt." + coralsAttempted + ".offsetYRelativeDifference", offsetYDifference);
        SmartDashboard.putNumber("attempt." + coralsAttempted + ".offsetXRelativeDifference", offsetXDifference);

        Logger.recordOutput("attempt." + coralsAttempted + ".wasScored", false);

        Logger.recordOutput("corals.attempted", coralsAttempted);

        coralsAttempted++;
    }

    public void successfulAttempt() {
        int attemptIndex = coralsAttempted - 1;

        Logger.recordOutput("attempt." + attemptIndex + ".wasScored", true);
        coralsScored++;

        double diffY = SmartDashboard.getNumber("attempt." + attemptIndex + ".offsetYRelativeDifference", 0.0);
        double diffX = SmartDashboard.getNumber("attempt." + attemptIndex + ".offsetXRelativeDifference", 0.0);
        double offsetY = SmartDashboard.getNumber("attempt." + attemptIndex + ".offsetYRelative", 0.0);
        double offsetX = SmartDashboard.getNumber("attempt." + attemptIndex + ".offsetXRelative", 0.0);
        boolean wasRight = SmartDashboard.getBoolean("attempt." + attemptIndex + ".isAllignRight", false);
        int tagID = (int) SmartDashboard.getNumber("attempt." + attemptIndex + ".aprilTag", -1);

        //limeLightForwards.correctTheOffset(diffY, diffX, wasRight, tagID);

        if (wasRight) {
            SmartDashboard.putNumber("at.ID." + tagID + ".right.Y", offsetY);
            Logger.recordOutput("at.ID." + tagID + ".right.Y", offsetY);
            SmartDashboard.putNumber("at.ID." + tagID + ".right.X", offsetX);
            Logger.recordOutput("at.ID." + tagID + ".right.X", offsetX);
            SmartDashboard.putNumber("at.ID." + tagID + ".right.YDiff", diffY);
            Logger.recordOutput("at.ID." + tagID + ".right.YDiff", diffY);
            SmartDashboard.putNumber("at.ID." + tagID + ".right.XDiff", diffX);
            Logger.recordOutput("at.ID." + tagID + ".right.XDiff", diffX);
        } else {
            SmartDashboard.putNumber("at.ID." + tagID + ".left.Y", offsetY);
            Logger.recordOutput("at.ID." + tagID + ".left.Y", offsetY);
            SmartDashboard.putNumber("at.ID." + tagID + ".left.X", offsetX);
            Logger.recordOutput("at.ID." + tagID + ".left.X", offsetX);
            SmartDashboard.putNumber("at.ID." + tagID + ".left.YDiff", diffY);
            Logger.recordOutput("at.ID." + tagID + ".left.YDiff", diffY);
            SmartDashboard.putNumber("at.ID." + tagID + ".left.XDiff", diffX);
            Logger.recordOutput("at.ID." + tagID + ".left.XDiff", diffX);
        }

        //account for auto
        Logger.recordOutput("corals.scored", coralsScored);
        Logger.recordOutput("corals.missRate", (1 - (coralsScored / coralsAttempted)) * 100 + "%");
    }


}
