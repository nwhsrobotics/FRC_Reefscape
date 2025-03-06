package frc.robot.autos;
import com.google.flatbuffers.FlexBuffers.Map;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionAprilTag;
import frc.robot.subsystems.VisionGamePiece;
import frc.robot.util.Elastic;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import static java.util.Map.entry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Auto extends SequentialCommandGroup {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem visionForwards;
    private final Pose2d initialPos;
    private final List<String> locationsToGo;
    private final VisionSubsystem visionBackwards;
    // add the dictionaries for red and blue alliance with respective tag IDs for locations
    
    public Auto(SwerveSubsystem swerve, VisionSubsystem visionForwards, VisionSubsystem visionBackwards, List<String> posToGo, Pose2d initialPos) {
        this.swerve = swerve;
        this.locationsToGo = posToGo;
        this.initialPos = initialPos;
        this.visionForwards = visionForwards;
        this.visionBackwards = visionBackwards;
        // LimelightHelpers.setLEDMode_ForceBlink(LimelightConstants.llFront);
        LimelightHelpers.setLEDMode_ForceOn(LimelightConstants.llFront);
        // Elastic.Notification.notif("Auto done");
        addCommands(
                // Reset robot odometry to a initial position.
                new InstantCommand(() -> flipResetOdometry(initialPos)),
                //new InstantCommand(() -> Elastic.Notification.notif("AUTO RUNNING")),
                NamedCommands.getCommand("autoInit"),
                NamedCommands.getCommand("Output"),
                scoreCoral()
                //new InstantCommand(() -> Elastic.Notification.notif("AUTO RUNNING"))
        );
    }

    // Contructs a sequential command group, essentially a set of actions (commands) the robot will perform 
    // Checks how many corals (for-loop) it needs to get and adds that many necessary actions (commands)
    // Depending on the start location (A,B,C) you have conditionals to make or follow a specific path
    /**
     * Generates a SequentialCommandGroup to get corals during autonomous.
     *
     * @return A SequentialCommandGroup containing commands to get corals.
     */
    public SequentialCommandGroup scoreCoral() {
        SequentialCommandGroup exitReturnCommands = new SequentialCommandGroup();
            //Checks if robot is at position A
            if (initialPos.equals(Positions.START_A)) {
                //Starts from position A and then goes to first position in list 
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[A] " + locationsToGo.get(0)).onlyWhile(() -> !visionForwards.isDetectingTargetID(locationsToGo.get(0))));
                //once the april tag is detected, pathFindAprilTag comes in and adjusts the robot to the april tag
                exitReturnCommands.addCommands(new AlternativePathfindAprilTag(visionForwards.getAprilTagId(locationsToGo.get(0)), swerve, visionForwards, locationsToGo.get(0)));
                exitReturnCommands.addCommands(NamedCommands.getCommand("L4CORAL"));
                exitReturnCommands.addCommands(NamedCommands.getCommand("Outtake"));
                //exitReturnCommands.addCommands(new ParallelCommandGroup(NamedCommands.getCommand("L4CORAL"), swerve.pathFindThenFollowPath("[A] " + locationsToGo.get(0)).onlyWhile(() -> !visionForwards.isDetectingTargetID(locationsToGo.get(0)))));

                
                for (int i = 0; i < locationsToGo.size()-1; i++) {
                    //Iterates through each position in the list to station 1
                    exitReturnCommands.addCommands(NamedCommands.getCommand("LoadStation"));
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath(locationsToGo.get(i) + " [S1]").onlyWhile(()-> !visionBackwards.isDetectingTargetID("[S1]")));
                    exitReturnCommands.addCommands(new AlternativePathfindAprilTag(visionBackwards.getAprilTagId(locationsToGo.get(i)), swerve, visionBackwards, locationsToGo.get(i)));
                   
                    //Move robot from station 1 to next station
                    int finalI = i;
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[S1] " + locationsToGo.get(i+1)).onlyWhile(()-> !visionForwards.isDetectingTargetID(locationsToGo.get(finalI +1))));
                    exitReturnCommands.addCommands(new AlternativePathfindAprilTag(visionForwards.getAprilTagId(locationsToGo.get(i+1)), swerve, visionForwards, locationsToGo.get(i+1)));
                    exitReturnCommands.addCommands(NamedCommands.getCommand("L4CORAL"));
                    exitReturnCommands.addCommands(NamedCommands.getCommand("Outtake"));
                }


            }
            //Checks if robot is at position B
            if (initialPos.equals(Positions.START_B)) {
                //exitReturnCommands.addCommands(new InstantCommand(() -> Elastic.Notification.notif("B AUTO")));
                //Starts from position B and then goes to first position in list
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[B] " + locationsToGo.get(0)).onlyWhile(()-> !visionForwards.isDetectingTargetID(locationsToGo.get(0))));
                exitReturnCommands.addCommands(new AlternativePathfindAprilTag(visionForwards.getAprilTagId(locationsToGo.get(0)), swerve, visionForwards, locationsToGo.get(0)));
                exitReturnCommands.addCommands(NamedCommands.getCommand("L4CORAL"));
                exitReturnCommands.addCommands(NamedCommands.getCommand("Outtake"));

                for (int i = 0; i < locationsToGo.size()-1; i++) {
                    //Iterates through each position in the list to station 1
                    exitReturnCommands.addCommands(NamedCommands.getCommand("LoadStation"));
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath(locationsToGo.get(i) + " [S1]").onlyWhile(()-> !visionBackwards.isDetectingTargetID("[S1]")));
                    exitReturnCommands.addCommands(new AlternativePathfindAprilTag(visionBackwards.getAprilTagId(locationsToGo.get(i)), swerve, visionBackwards, locationsToGo.get(i)));
                    
                    //Move robot from station 1 to next station
                    int finalI = i;
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[S1] " + locationsToGo.get(i+1)).onlyWhile(()-> !visionForwards.isDetectingTargetID(locationsToGo.get(finalI +1))));
                    exitReturnCommands.addCommands(new AlternativePathfindAprilTag(visionForwards.getAprilTagId(locationsToGo.get(i+1)), swerve, visionForwards, locationsToGo.get(i+1)));
                    exitReturnCommands.addCommands(NamedCommands.getCommand("L4CORAL"));
                    exitReturnCommands.addCommands(NamedCommands.getCommand("Outtake"));
                }
                
            }

            //Checks if robot is at position C
            if (initialPos.equals(Positions.START_C)) {
                //Starts from position C and then goes to first position in list
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[C] " + locationsToGo.get(0)).onlyWhile(()-> !visionForwards.isDetectingTargetID(locationsToGo.get(0))));
                exitReturnCommands.addCommands(new AlternativePathfindAprilTag(visionForwards.getAprilTagId(locationsToGo.get(0)), swerve, visionForwards, locationsToGo.get(0)));
                exitReturnCommands.addCommands(NamedCommands.getCommand("L4CORAL"));
                exitReturnCommands.addCommands(NamedCommands.getCommand("Outtake"));

                for (int i = 0; i < locationsToGo.size()-1; i++) {
                    //Iterates through each position in the list to station 2
                    exitReturnCommands.addCommands(NamedCommands.getCommand("LoadStation"));
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath(locationsToGo.get(i) + " [S2]").onlyWhile(()-> !visionBackwards.isDetectingTargetID("[S2]")));
                    exitReturnCommands.addCommands(new AlternativePathfindAprilTag(visionBackwards.getAprilTagId(locationsToGo.get(i)), swerve, visionBackwards, locationsToGo.get(i)));
                    
                    //Move robot from station 2 to next station
                    int finalI = i;
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[S2] " + locationsToGo.get(i+1)).onlyWhile(()-> !visionForwards.isDetectingTargetID(locationsToGo.get(finalI +1))));
                    exitReturnCommands.addCommands(new AlternativePathfindAprilTag(visionForwards.getAprilTagId(locationsToGo.get(i+1)), swerve, visionForwards, locationsToGo.get(i+1)));
                    exitReturnCommands.addCommands(NamedCommands.getCommand("L4CORAL"));
                    exitReturnCommands.addCommands(NamedCommands.getCommand("Outtake"));
                }
                
            ;
        }

        return exitReturnCommands;
    }

    /**
     * Resets robot odometry to a flipped position if the alliance is red.
     *
     * @param loc The position to reset the robot odometry to.
     */

    public void flipResetOdometry(Pose2d loc) {
        swerve.resetOdometry(getLocation(loc));
    }

    public Pose2d getLocation(Pose2d loc){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return FlippingUtil.flipFieldPose(loc);
        } else {
            return loc;
        }
    }
}