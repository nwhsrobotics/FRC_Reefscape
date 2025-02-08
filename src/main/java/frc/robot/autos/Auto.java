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
    private final Pose2d initialPos;
    private final List<String> locationsToGo;

    //private ArrayList allPaths = new ArrayList<String>(List.of("[B][1A]","[1A] [S1]","[1B] [S1]","[2] [S1]","[2A] [S2]","[2B] [S2]",));
    // add the dictionaries for red and blue alliance with respective tag IDs for locations
    public static HashMap<String, Integer> blueAllianceIds = new HashMap<>();
    blueAllianceIds.put("[1A]", 20);
    blueAllianceIds.put("[1B]", 20);
    blueAllianceIds.put("[2A]", 21);
    blueAllianceIds.put("[2B]", 21);
    blueAllianceIds.put("[3A]", 22);
    blueAllianceIds.put("[3B]", 22);
    blueAllianceIds.put("[4A]", 17);
    blueAllianceIds.put("[4B]", 17);
    blueAllianceIds.put("[5A]", 18);
    blueAllianceIds.put("[5B]", 18);
    blueAllianceIds.put("[6A]", 19);
    blueAllianceIds.put("[6B]", 19);

    public static HashMap<String, Integer> redAllianceIds = new HashMap<>();
    redAllianceIds.put("[1A]", 11);
    redAllianceIds.put("[1B]", 11);
    redAllianceIds.put("[2A]", 10);
    redAllianceIds.put("[2B]", 10);
    redAllianceIds.put("[3A]", 9);
    redAllianceIds.put("[3B]", 9);
    redAllianceIds.put("[4A]", 8);
    redAllianceIds.put("[4B]", 8);
    redAllianceIds.put("[5A]", 7);
    redAllianceIds.put("[5B]", 7);
    redAllianceIds.put("[6A]", 6);
    redAllianceIds.put("[6B]", 16);


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
    
    public Auto(SwerveSubsystem swerve, Vision vision, List<String> posToGo, Pose2d initialPos) {
        this.swerve = swerve;
        this.locationsToGo = posToGo;
        this.vision = vision;
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
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[A] " + locationsToGo.get(0)).onlyWhile(() -> !isDetectingTargetID(locationsToGo.get(0)))));
                for (int i = 0; i < locationsToGo.size()-1; i++) {
                    //Iterates through each position in the list to station 1
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath(locationsToGo.get(i) + " [S1]").onlyWhile(()-> !isDetectingTargetID("[S1]")));
                    //Move robot from station 1 to next station
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[S1] " + locationsToGo.get(i+1)).onlyWhile(()-> !isDetectingTargetID(locationsToGo.get(i+1))));

                }

            }
            //Checks if robot is at position B
            if (swerve.getPose().getY() > 5.90 && swerve.getPose().getY() < 6.50) {
                //Starts from position B and then goes to first position in list
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[B] " + locationsToGo.get(0)).onlyWhile(()-> !isDetectingTargetID(locationsToGo.get(0))));
                for (int i = 0; i < locationsToGo.size()-1; i++) {
                    //Iterates through each position in the list to station 1
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath(locationsToGo.get(i) + " [S1]")).onlyWhile(()-> !isDetectingTargetID("[S1]"));
                    //Move robot from station 1 to next station
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[S1] " + locationsToGo.get(i+1)).onlyWhile(()-> !isDetectingTargetID(locationsToGo.get(i+1))));
                }
                
            }

            //Checks if robot is at position C
            if (swerve.getPose().getY() > 4.80 && swerve.getPose().getY() < 5.40) {
                //Starts from position C and then goes to first position in list
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[C] " + locationsToGo.get(0)).onlyWhile(()-> !isDetectingTargetID(locationsToGo.get(0))));
                for (int i = 0; i < locationsToGo.size()-1; i++) {
                    //Iterates through each position in the list to station 2
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath(locationsToGo.get(i) + " [S2]").onlyWhile(()-> !isDetectingTargetID("[S2]")));
                    //Move robot from station 2 to next station
                    exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[S2] " + locationsToGo.get(i+1)).onlyWhile(()-> !isDetectingTargetID(locationsToGo.get(i+1))));
                }
                
            ;
        }
        exitReturnCommands.addCommands();

        return exitReturnCommands;
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

    /**
     * 
     * @param targetLocation The location we are trying to go to (our convention)
     * @return The boolean is the April Tag ID for that location (field convention and respective alliance) is currently detected
     */
    public boolean isDetectingTargetID(String targetLocation){
        var alliance = DriverStation.getAlliance();
        int targetId = -1;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetId = redAllianceIds.get(targetLocation);
        } else {
            targetId = blueAllianceIds.get(targetLocation);
        }
        LimelightResults llr = VisionAprilTag.isValid("limelight");
        if (llr != null && llr.targets_Fiducials != null){
            return llr.targets_Fiducials[0].fiducialID == targetId;
        }
        return false;
    }
}