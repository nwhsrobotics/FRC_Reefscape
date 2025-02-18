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
import frc.robot.subsystems.VisionSubsystem;
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
     * Generates a SequentialCommandGroup to get corals during autonomous.
     *
     * @return A SequentialCommandGroup containing commands to get corals.
     */
    public SequentialCommandGroup scoreCoral() {
        SequentialCommandGroup exitReturnCommands = new SequentialCommandGroup();
        LimelightResults llr = VisionAprilTag.isValid("limelight");
        if (llr != null){
            double id = llr.targets_Fiducials[0].fiducialID;
        }
        for (int i = 0; i < coralLimit; i++) {  //amount of notes to get + 1 preloaded
            if (swerve.getPose().getY() > 7){
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath("[A] [1A]").onlyWhile(() -> !VisionAprilTag.isValid("limelight").targets_Fiducials[0].fiducialID == blueAllianceIds.get("[1A]")));
            }

            if (swerve.getPose().getY() > 7){
                exitReturnCommands.addCommands(swerve.pathFindThenFollowPath(""));
            }
            exitReturnCommands.addCommands(
                    //currently only using 1 limelight
                    // Set the limelight pipeline index to 1 for vision processing.
                    new InstantCommand(() -> LimelightHelpers.setPipelineIndex(LimelightConstants.llObjectDetectionName, 1)),

                    //Starting from position A
                    swerve.pathFindThenFollowPath("[A] [6A]").onlyWhile(()-> (swerve.getPose().getY() > 7.00) && (swerve.getPose().getY() < 7.50)),
                    //Starting from position B
                    swerve.pathFindThenFollowPath("[B] [1A]").onlyWhile(()-> (swerve.getPose().getY() > 5.90) && (swerve.getPose().getY() < 6.50)),
                    //Starting from position C
                    swerve.pathFindThenFollowPath("[C] [2A]").onlyWhile(()-> (swerve.getPose().getY() > 4.80) && (swerve.getPose().getY() < 5.40)),
                    
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
                    new InstantCommand(() -> possibleLocations.remove(getClosestLocation()))
                    // Execute the shooting command.
                    //NamedCommands.getCommand("shoot")
                    //new AutoScoringCommand(score, ScoringState.FIRE, ScoringState.IDLE)
                
            );
        }
            */
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

    public void addCommandActions(SequentialCommandGroup commands, String startLocation, String station){
        
    }
}