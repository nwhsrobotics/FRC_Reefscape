package frc.robot;


import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.autos.PosePIDCommand;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.*;
import frc.robot.util.Buttons;

public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public int coralsAttempted = 0;
    public int coralsScored = 0;

    //private final AlgaeArm algaeArm = new AlgaeArm();

    public final IntakeOuttake intakeoutake = new IntakeOuttake();

    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public final LED_Subsystem led_sybsystem = new LED_Subsystem();

    //public final ElevatorSysID elevatorSysID = new ElevatorSysID();

    public final VisionSubsystem limeLightForwards = new VisionSubsystem(LimelightConstants.llFront);

    // public final SysId sysIdSubsystem = new SysId();
    public final VisionSubsystem limeLightBackwards = new VisionSubsystem(LimelightConstants.llBack);

    private final Field2d field;


    private final SendableChooser<Command> autoChooser;

    public XboxController driver = new XboxController(0);
    public static XboxController gunner = new XboxController(1);

    public RobotContainer() {

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

        //make these instantcommands and reuse them for auto and manual
        // Command L4CMD = new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem);
        // Command L3CMD = new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem);
        // Command L2CMD = new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem);
        // Command L1CMD = new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem);
        // Command LoadStation = new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem);

        //INIT after registering named commands
        //maybe wait until commands instead of separate classes?
        NamedCommands.registerCommand("L4CORAL", new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L3CORAL", new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L2CORAL", new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("L1CORAL", new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        NamedCommands.registerCommand("LoadStation", new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem));
        NamedCommands.registerCommand("Boost", new InstantCommand(() -> elevatorSubsystem.boost(), elevatorSubsystem).andThen(new WaitUntilCommand(elevatorSubsystem::isNearTargetPosition)));
        // NamedCommands.registerCommand("L4", new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem).andThen(new WaitCommand(2.0)));
        // NamedCommands.registerCommand("L3", new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem).andThen(new WaitCommand(1.7)));
        // NamedCommands.registerCommand("L2", new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem).andThen(new WaitCommand(1.4)));
        // NamedCommands.registerCommand("L1", new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem).andThen(new WaitCommand(1.1)));
        // NamedCommands.registerCommand("Load", new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem).andThen(new WaitCommand(0.9)));
        //Tune intake/outtake time for 3 corals, intake 1 second and outtake 0.5 seconds
        //TODO: Wait until command until ultrasonic (only if in intake not below that)
        NamedCommands.registerCommand("Intake", new WaitCommand(0.75));
        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> intakeoutake.outtakeOpen(), intakeoutake)
                .andThen(new InstantCommand(() -> recordAttempt()))
                .andThen(new WaitCommand(0.5))
                .andThen(new InstantCommand(() -> intakeoutake.outtakeClose(), intakeoutake)));
        // NamedCommands.registerCommand("MoveElevator", new InstantCommand());

        // autos (either PathPlanner or Auto with Vision) (cannot have both enabled, need to keep 1 commented out)
        // remove this comment below for pathplanner auto
        autoChooser = AutoBuilder.buildAutoChooser("Straight Auto");
        // remove all the autochooser comments below for Auto with Vision auto
        //autoChooser = new SendableChooser<>();
        //autoChooser.addOption("A*", new Auto(swerveSubsystem, limeLightForwards, limeLightBackwards, new ArrayList<>(List.of("[1A]", "[6B]", "[5A]", "[5B]")), Positions.START_A));
        //autoChooser.addOption("A1 to 5A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<>(List.of("[5A]", "[6B]", "[5A]", "[5B]")), Positions.START_A));
        //autoChooser.addOption("A to 5A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<>(List.of("[5A]", "[5B]", "[4A]", "[4B]")), Positions.START_A));
        //autoChooser.addOption("A1 to 4A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<>(List.of("[4A]", "[5B]", "[4A]", "[4B]")), Positions.START_A));
        //autoChooser.addOption("B*", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<>(List.of("[1A]", "[1B]", "[5A]", "[5B]")), Positions.START_B));
        //autoChooser.addOption("B1 to 1A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<>(List.of("[1A]", "[5B]", "[1A]", "[5B]")), Positions.START_B));
        //autoChooser.addOption("C*", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<>(List.of("[4A]", "[4B]", "[3A]", "[3B]")), Positions.START_C));
        //autoChooser.addOption("C1 to 4A", new Auto(swerveSubsystem, limeLightForwards,limeLightBackwards, new ArrayList<>(List.of("[3A]", "[4B]", "[3A]", "[3B]")), Positions.START_C));


        // Gunner controlls 
        // https://docs.google.com/drawings/d/1NsJOx6fb6KYHW6L8ZeuNtpK3clnQnIA9CD2kQHFL0P0/edit?usp=sharing
        //new POVButton(gunner, Buttons.POV_UP).onTrue(new InstantCommand(() -> elevatorSubsystem.increaseCurrentLevel(), elevatorSubsystem));
        //new POVButton(gunner, Buttons.POV_DOWN).onTrue(new InstantCommand(() -> elevatorSubsystem.decreaseCurrentLevel(), elevatorSubsystem));
        //new JoystickButton(gunner, Buttons.X).onTrue(new InstantCommand(() -> algaeArm.triggerAlgaeArm(), algaeArm));
        new JoystickButton(gunner, Buttons.Y).onTrue(new InstantCommand(() -> elevatorSubsystem.L1_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.B).onTrue(new InstantCommand(() -> elevatorSubsystem.L2_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.A).onTrue(new InstantCommand(() -> elevatorSubsystem.L3_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.X).onTrue(new InstantCommand(() -> elevatorSubsystem.L4_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.RIGHT_STICK_BUTTON).onTrue(new InstantCommand(() -> elevatorSubsystem.loadStation_Preset(), elevatorSubsystem));
        new JoystickButton(gunner, Buttons.RIGHT_BUMPER).whileTrue(((new InstantCommand(() -> recordAttempt())).andThen(new InstantCommand(() -> intakeoutake.outtakeOpen(), intakeoutake))).onlyIf(() -> !intakeoutake.isIntakeOpen));
        new JoystickButton(gunner, Buttons.RIGHT_BUMPER).onFalse(new InstantCommand(() -> intakeoutake.outtakeClose(), intakeoutake));
        //new JoystickButton(gunner, Buttons.LEFT_BUMPER).onTrue(NamedCommands.getCommand("L4CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("LoadStation"))); 

        new JoystickButton(gunner, Buttons.LEFT_BUMPER).onTrue(NamedCommands.getCommand("L4CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("Boost")).andThen(NamedCommands.getCommand("LoadStation")));


        //Add feedforward to elevator for higher accel, reduce allowed error
        new POVButton(gunner, Buttons.POV_UP).onTrue(NamedCommands.getCommand("L1CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("LoadStation")));
        new POVButton(gunner, Buttons.POV_RIGHT).onTrue(NamedCommands.getCommand("L2CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("LoadStation")));
        new POVButton(gunner, Buttons.POV_DOWN).onTrue(NamedCommands.getCommand("L3CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("LoadStation")));
        //new POVButton(gunner, Buttons.POV_LEFT).onTrue(NamedCommands.getCommand("L4CORAL").andThen(NamedCommands.getCommand("Outtake")).andThen(NamedCommands.getCommand("LoadStation")));
        new JoystickButton(gunner, Buttons.MENU).onTrue(new InstantCommand(() -> successfulAttempt()));
        new JoystickButton(gunner, Buttons.VIEW).onTrue(new InstantCommand(() -> successfulAttempt()));

        //Driver controlls 
        // https://docs.google.com/drawings/d/1NsJOx6fb6KYHW6L8ZeuNtpK3clnQnIA9CD2kQHFL0P0/edit?usp=sharing
        // don'ooglt need reset odometry with vision because you can fake angle it (for rotation) + the april tag corrects odometry live
        //new JoystickButton(driver, Buttons.MENU).onTrue(new InstantCommand(swerveSubsystem::resetOdometryWithVision, swerveSubsystem));
        new JoystickButton(driver, Buttons.VIEW).onTrue(new InstantCommand(swerveSubsystem::switchFR, swerveSubsystem));
        new POVButton(driver, Buttons.POV_DOWN).onTrue(new InstantCommand(()
        -> swerveSubsystem.resetOdometry(new Pose2d(swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY(), limeLightForwards.getFakeAngle(swerveSubsystem.getPose())))));
        //new JoystickButton(driver, Buttons.RIGHT_STICK_BUTTON).onTrue(new InstantCommand(swerveSubsystem.autonavigator::toggle));
        // new JoystickButton(driver, Buttons.X).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.STATION_LEFT)));
        // new JoystickButton(driver, Buttons.B).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.STATION_RIGHT)));
        // new JoystickButton(driver, Buttons.Y).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_REEF)));
        // new JoystickButton(driver, Buttons.A).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.BACK_REEF)));

        //Add PID based alligning, if pathplanner is inaccurate
        //new NavPID(swerveSubsystem, limeLightForwards, true);
        //Invert controls, left/right reef
        new JoystickButton(driver, Buttons.X).onTrue(
                new InstantCommand(() -> {
                    Pose2d target = limeLightForwards.rightReef(swerveSubsystem.getPose());
                    swerveSubsystem.autonavigator.navigateTo(target);
                })
        );


        //  new JoystickButton(driver, Buttons.Y).onTrue(
        //      new InstantCommand(() -> {
        //          Pose2d target = limeLightForwards.leftReef(swerveSubsystem.getPose());
        //          swerveSubsystem.autonavigator.enable();
        //          swerveSubsystem.autonavigator.navigateToWithElevator(target);
        //      })
        //  );

        // //  new JoystickButton(driver, Buttons.A).onTrue(
        // //      new InstantCommand(() -> {
        // //          Pose2d target = limeLightForwards.rightReef(swerveSubsystem.getPose());
        // //          swerveSubsystem.autonavigator.enable();
        // //          swerveSubsystem.autonavigator.navigateToWithElevator(target);
        // //      })
        // //  );

        //  new JoystickButton(driver, Buttons.A).onTrue(
        //     new PosePIDCommand(swerveSubsystem, limeLightForwards.rightReef(swerveSubsystem.getPose())
        // ));


        // //last alligning resort
        // new JoystickButton(driver, Buttons.Y).onTrue(
        //     new RunCommand(() -> swerveSubsystem.drive(0, 0.1, 0, false, true), swerveSubsystem)
        //         .withTimeout(0.32).andThen(new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem))
        // );

        // new JoystickButton(driver, Buttons.A).onTrue(
        //     new RunCommand(() -> swerveSubsystem.drive(0, -0.1, 0, false, true), swerveSubsystem)
        //         .withTimeout(0.32).andThen(new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem))
        // );

        new JoystickButton(driver, Buttons.B).onTrue(
                new InstantCommand(() -> {
                    Pose2d target = limeLightForwards.leftReef(swerveSubsystem.getPose());
                    swerveSubsystem.autonavigator.navigateTo(target);
                })
        );

        // new JoystickButton(driver, Buttons.Y).onTrue(
        //     new InstantCommand(() -> {
        //         Pose2d target = limeLightForwards.scootRight(
        //             VisionAprilTag.transformTargetLocation(
        //                 swerveSubsystem.getPose(), 
        //                 LimelightConstants.llFront
        //             ),
        //             0.1651
        //         );
        //         Command cmd = swerveSubsystem.pathfindToPosition(target)
        //             .andThen(
        //                 NamedCommands.getCommand("L4CORAL"),
        //                 NamedCommands.getCommand("Outtake"),
        //                 NamedCommands.getCommand("LoadStation")
        //             );
        //         cmd.addRequirements(swerveSubsystem);
        //         cmd.schedule();
        //     })
        // );

        // new JoystickButton(driver, Buttons.A).onTrue(
        //     new InstantCommand(() -> {
        //         Pose2d target = limeLightForwards.scootLeft(
        //             VisionAprilTag.transformTargetLocation(
        //                 swerveSubsystem.getPose(), 
        //                 LimelightConstants.llFront
        //             ),
        //             0.1651
        //         );
        //         Command cmd = swerveSubsystem.pathfindToPosition(target)
        //             .andThen(
        //                 NamedCommands.getCommand("L4CORAL"),
        //                 NamedCommands.getCommand("Outtake"),
        //                 NamedCommands.getCommand("LoadStation")
        //             );
        //         cmd.addRequirements(swerveSubsystem);
        //         cmd.schedule();
        //     })
        // );


        // new POVButton(driver, Buttons.POV_RIGHT).onTrue(new InstantCommand(() -> VisionAprilTag.offsetRight(Constants.LimelightConstants.llFront)));
        // new POVButton(driver, Buttons.POV_LEFT).onTrue(new InstantCommand(() -> VisionAprilTag.offsetLeft(Constants.LimelightConstants.llFront)));
        // new POVButton(driver, Buttons.POV_UP).onTrue(new InstantCommand(() -> VisionAprilTag.offsetCenter(Constants.LimelightConstants.llFront)));
        // new POVButton(driver, Buttons.POV_DOWN).onTrue(
        //     new InstantCommand(() -> {
        //         Pose2d target = VisionAprilTag.transformTargetLocation(swerveSubsystem.getPose(), LimelightConstants.llFront);
        //         swerveSubsystem.autonavigator.enable();
        //         swerveSubsystem.autonavigator.navigateTo(target);
        //     })
        // );


        // new POVButton(driver, Buttons.POV_DOWN).onTrue(
        //     new InstantCommand(() -> {
        //         Pose2d target = limeLightForwards.scootRight(swerveSubsystem.getPose(), 0.1651);
        //         swerveSubsystem.autonavigator.enable();
        //         swerveSubsystem.autonavigator.navigateTo(target);
        //     })
        // );

        //new JoystickButton(driver, Buttons.A).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.BACK_MID_REEF)));
        //new JoystickButton(driver, Buttons.B).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_RIGHT_REEF)));

        //new JoystickButton(driver, Buttons.B).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_RIGHT_REEF)));
        //new JoystickButton(driver, Buttons.B).onTrue(swerveSubsystem.pathFindThenFollowPath("[A1] [6A]"));
        //This can work
        //new JoystickButton(driver, Buttons.LEFT_BUMPER).whileTrue(AutoBuilder.pathfindToPose(Positions.BACK_LEFT_REEF, AutoConstants.kPathfindingConstraints, 0.0));
        /*
        new JoystickButton(gunner, Buttons.X).onTrue(elevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kForward));
        new JoystickButton(gunner, Buttons.Y).onTrue(elevatorSysID.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        new JoystickButton(gunner, Buttons.A).onTrue(elevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        new JoystickButton(gunner, Buttons.B).onTrue(elevatorSysID.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        */


        //new POVButton(driver, Buttons.POV_RIGHT).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.BACK_RIGHT_REEF)));
        //new POVButton(driver, Buttons.POV_LEFT).onTrue(new InstantCommand(() -> swerveSubsystem.autonavigator.navigateTo(Positions.FRONT_LEFT_REEF)));
        SmartDashboard.putData("Auto Chooser", autoChooser);


        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driver));


        //intakeoutake.setDefaultCommand(new IntakeOuttakeCommand(intakeoutake, gunner));
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
        Logger.recordOutput("corals.missRate", String.valueOf((1 - (coralsScored / coralsAttempted)) * 100) + "%");
    }
    
    
    
}
