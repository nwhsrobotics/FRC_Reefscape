package frc.robot.autos;
import com.google.flatbuffers.FlexBuffers.Map;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionAprilTag;
import frc.robot.subsystems.VisionGamePiece;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import static java.util.Map.entry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Auto extends SequentialCommandGroup {
    private final SwerveSubsystem swerve;
    private final Vision vision;
    private final List<Pose2d> possibleLocations;
    private int coralLimit = 10;
    private final Pose2d initialPos;
    private final List<String> locationsToGo;

    //private ArrayList allPaths = new ArrayList<String>(List.of("[B][1A]","[1A] [S1]","[1B] [S1]","[2] [S1]","[2A] [S2]","[2B] [S2]",));
    // add the dictionaries for red and blue alliance with respective tag IDs for locations
    public static HashMap<String, Integer> blueAllianceIds = new HashMap<>();
    blueAllianceIds.put("[1A]", 1);
    blueAllianceIds.put("[1B]", 1);
    blueAllianceIds.put("[2A]", 2);
    blueAllianceIds.put("[2B]", 2);
    blueAllianceIds.put("[3A]", 3);
    blueAllianceIds.put("[3B]", 3);
    blueAllianceIds.put("[4A]", 4);
    blueAllianceIds.put("[4B]", 4);
    blueAllianceIds.put("[5A]", 5);
    blueAllianceIds.put("[5B]", 5);
    blueAllianceIds.put("[6A]", 6);
    blueAllianceIds.put("[6B]", 6);

    private ArrayList<String> occupiedStations = new ArrayList<String>();


    // 
    /**
     * Creates a new instance of the Auto class.
     *
     * @param swerve             The SwerveSubsystem object representing the robot's swerve drive.
     * @param blackListLocations A list of Pose2d objects representing locations to blacklist.
     * @param coralLimit          The limit on the number of notes to be obtained during autonomous + 1 preloaded.
     * @param initialPos         The initial position to reset the robot odometry to.
     */
    
    public Auto(SwerveSubsystem swerve, Vision vision, List<Pose2d> blackListLocations, List<String> posToGo, int coralLimit, Pose2d initialPos) {
        this.swerve = swerve;
        this.locationsToGo = posToGo;
        this.vision = vision;
        possibleLocations = Constants.AprilTags.aprilTags;
        blackList(blackListLocations);
        this.coralLimit = coralLimit;
        this.initialPos = initialPos;
        blueAllianceIds.put("1", 1);
        String s = "[1a]";
        s.substring(0, 1);
        addCommands(
                // Reset robot odometry to a initial position.
                new InstantCommand(() -> flipResetOdometry(initialPos)),
            
                NamedCommands.getCommand("autoInit"),
                NamedCommands.getCommand("Output"),
                scoreCoral()
        );
    }

    // Contructs a sequential command group, essentially a set of actions (commands) the robot will perform 
    // Checks how many corals (for-loop) it needs to get and adds that many necessary actions (commands)
    // Depending on the start location (A,B,C) you have conditionals to make or follow a specific path
    /**
     * Generates a SequentialCommandGroup to get notes during autonomous.
     *
     * @return A SequentialCommandGroup containing commands to get notes.
     */
    public SequentialCommandGroup scoreCoral() {
        SequentialCommandGroup exitReturnCommands = new SequentialCommandGroup();
        LimelightResults llr = VisionAprilTag.isValid("limelight");
        if (llr != null){
            double id = llr.targets_Fiducials[0].fiducialID;
        }
            //Checks if robot is at position A
            if (swerve.getPose().getY() > 7 && swerve.getPose().getY() < 7.50) {
                //Starts from position A and then goes to first position in list 
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[A] " + locationsToGo.get(0)).onlyWhile(() -> !VisionAprilTag.isValid("limelight").targets_Fiducials[0].fiducialID == blueAllianceIds.get("[6A]")));
                for (int i = 0; i < locationsToGo.size()-1; i++) {
                    //Iterates through each position in the list to station 1
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath(locationsToGo.get(i) + " [S1]").onlyWhile(()-> !VisionAprilTag.isValid("limelight").targets_Fiducials[0].fiducialID == blueAllianceIds.get("S1")));
                    //Move robot from station 1 to next station
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[S1] " + locationsToGo.get(i+1)).onlyWhile(()-> !VisionAprilTag.isValid("limelight").targets_Fiducials[0].fiducialID == blueAllianceIds.get("S1")));

                }

            }
            if (swerve.getPose().getY() > 5.90 && swerve.getPose().getY() < 6.50){
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[B] [1A]").onlyWhile(()-> !VisionAprilTag.isValid("limelight").targets_Fiducials[0].fiducialID == blueAllianceIds.get("[1A]")));
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[1A] [S1]").onlyWhile(()-> !VisionAprilTag.isValid("limelight").targets_Fiducials[0].fiducialID == blueAllianceIds.get("[S1]")));
            }

            if (swerve.getPose().getY() > 4.80 && swerve.getPose().getY() < 5.40){
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[C] [2A]").onlyWhile(()-> !VisionAprilTag.isValid("limelight").targets_Fiducials[0].fiducialID == blueAllianceIds.get("[2A]")));
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[2A] [S2]").onlyWhile(()-> !VisionAprilTag.isValid("limelight").targets_Fiducials[0].fiducialID == blueAllianceIds.get("[S2]")));
            exitReturnCommands.addCommands(
                    //currently only using 1 limelight
                    // Set the limelight pipeline index to 1 for vision processing.
                    new InstantCommand(() -> LimelightHelpers.setPipelineIndex(LimelightConstants.llObjectDetectionName, 1)),

                    //Starting from position A
                    //swerve.pathFindThenFollowPath("[A] [6A]").onlyWhile(()-> (swerve.getPose().getY() > 7.00) && (swerve.getPose().getY() < 7.50)),
                    //Starting from position B
                    //swerve.pathFindThenFollowPath("[B] [1A]").onlyWhile(()-> (swerve.getPose().getY() > 5.90) && (swerve.getPose().getY() < 6.50)),
                    //Starting from position C
                    //swerve.pathFindThenFollowPath("[C] [2A]").onlyWhile(()-> (swerve.getPose().getY() > 4.80) && (swerve.getPose().getY() < 5.40)),
                    
                    swerve.pathfindToPosition("").onlyWhile(() -> !LimelightHelpers.getTV(LimelightConstants.llObjectDetectionName),
                    



          
                    // Navigate the robot to the closest location without considering vision targeting.
                    swerve.pathfindToPosition(getClosestLocation()).onlyWhile(() -> !LimelightHelpers.getTV(LimelightConstants.llObjectDetectionName)),
                    // Navigate the robot to a specific location based on vision targeting.
                    swerve.pathfindToPosition(VisionGamePiece.visionTargetLocation),
                    //new PathFindVision(swerve, score, possibleLocations, getClosestLocation()),
                    //or command.repeatedly also works for single command
                    //Commands.repeatingSequence(new PathFindVision(swerve, score, possibleLocations, getClosestLocation()).until(() -> score.isNoteInside())),
                    // Set the limelight pipeline index back to 0 for april tag localization.
                    //note inside logic doesn't work currently but no current spike implementation done
                    new InstantCommand(() -> LimelightHelpers.setPipelineIndex(LimelightConstants.llObjectDetectionName, 0)),
                    // Navigate the robot to the initial position to shoot.
                    swerve.pathfindToPosition(initialPos),
                    //swerve.pathfindToPosition(getClosestLocation()).onlyWhile(() -> !(LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.llObjectDetectionName).rawFiducials[0].id == 7)),
                    // Remove the closest location from the list of possible locations.
                   new InstantCommand(() -> { possibleLocations.remove(getClosestLocation()); })
                    // Execute the shooting command.
                    //NamedCommands.getCommand("shoot")
                    //new AutoScoringCommand(score, ScoringState.FIRE, ScoringState.IDLE)
                
            ;
        }
        exitReturnCommands.addCommands();

        return exitReturnCommands;
    }

    /**
     * Blacklists specified locations from the list of possible locations.
     *
     * @param blackListLocations A list of Pose2d objects representing locations to blacklist.
     */
    public void blackList(List<Pose2d> blackListLocations) {
        for (Pose2d pos : blackListLocations) {
            possibleLocations.remove(pos);
        }
    }

    /**
     * Finds the closest location to the current robot pose from the list of possible locations.
     *
     * @return The closest Pose2d location.
     */
    public Pose2d getClosestLocation() {
        return swerve.getPose().nearest(possibleLocations);
    }

    /**
     * Resets robot odometry to a flipped position if the alliance is red.
     *
     * @param loc The position to reset the robot odometry to.
     */
    public void flipResetOdometry(Pose2d loc) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            swerve.resetOdometry(FlippingUtil.flipFieldPose(loc));
        } else {
            swerve.resetOdometry(loc);
        }
    }
}