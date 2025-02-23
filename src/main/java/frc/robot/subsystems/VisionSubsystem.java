// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;

public class VisionSubsystem extends SubsystemBase {
    /**
     * Creates a new Vision.
     */
    private String limelightName;
    public VisionSubsystem(String limelightName) {
        this.limelightName = limelightName;
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run\
        /*Logger.recordOutput("ll.straightLineDist", VisionGamePiece.straightLineZDistance(limelightName));
        Logger.recordOutput("ll.verticalDist", VisionGamePiece.verticalYOffsetDistance(limelightName));
        Logger.recordOutput("ll.horizontalDist", VisionGamePiece.horizontalOffestXDistance(limelightName));
        Logger.recordOutput("ll.3dHypotnuese", VisionGamePiece.full3DDistance(limelightName));
        Logger.recordOutput("ll.2dHypotnuese", VisionGamePiece.hypotenuseDistanceXandZ(limelightName));
        Logger.recordOutput("ll.rot", VisionGamePiece.hypotenuseDistanceXandZ(limelightName));
        Logger.recordOutput("ll.detect", LimelightHelpers.getTY(limelightName));
        //1, 1, Rotation2d.fromDegrees(60)
        Logger.recordOutput("ll.target", VisionGamePiece.transformTargetLocation(new Pose2d(), limelightName).toString());
        Logger.recordOutput("ll.targetX", VisionGamePiece.transformTargetLocation(new Pose2d(), limelightName).getX());
        Logger.recordOutput("ll.targetY", VisionGamePiece.transformTargetLocation(new Pose2d(), limelightName).getY());
        Logger.recordOutput("ll.targetDegrees", VisionGamePiece.transformTargetLocation(new Pose2d(), limelightName).getRotation().getDegrees());
        Logger.recordOutput("ll.targetRadians", VisionGamePiece.transformTargetLocation(new Pose2d(), limelightName).getRotation().getRadians());
        Logger.recordOutput("ll.targetRotation", VisionGamePiece.transformTargetLocation(new Pose2d(), limelightName).getRotation().getRotations());
        Logger.recordOutput("ll.getOriginDistance", VisionGamePiece.transformTargetLocation(new Pose2d(), limelightName).getTranslation().getNorm());

        
        Logger.recordOutput("lla.straightLineDist", VisionAprilTag.straightLineZDistance(limelightName));
        Logger.recordOutput("lla.verticalDist", VisionAprilTag.verticalYOffsetDistance(limelightName));
        Logger.recordOutput("lla.horizontalDist", VisionAprilTag.horizontalOffsetXDistance(limelightName));
        Logger.recordOutput("lla.rot", VisionAprilTag.limelight_aimX_proportional(limelightName));
        Logger.recordOutput("lla.3dHypotnuese", VisionAprilTag.hypotenuseDistanceXandZ(limelightName));
        Logger.recordOutput("lla.detect", LimelightHelpers.getTY(limelightName));
        //1, 1, Rotation2d.fromDegrees(60)
        Logger.recordOutput("lla.target", VisionAprilTag.transformTargetLocation(new Pose2d(),limelightName).toString());
        Logger.recordOutput("lla.targetX", VisionAprilTag.transformTargetLocation(new Pose2d(), limelightName).getX());
        Logger.recordOutput("lla.targetY", VisionAprilTag.transformTargetLocation(new Pose2d(), limelightName).getY());
        Logger.recordOutput("lla.targetDegrees", VisionAprilTag.transformTargetLocation(new Pose2d(), limelightName).getRotation().getDegrees());
        Logger.recordOutput("lla.targetRadians", VisionAprilTag.transformTargetLocation(new Pose2d(), limelightName).getRotation().getRadians());
        Logger.recordOutput("lla.targetRotation", VisionAprilTag.transformTargetLocation(new Pose2d(), limelightName).getRotation().getRotations());
        Logger.recordOutput("lla.getOriginDistance", VisionAprilTag.transformTargetLocation(new Pose2d(),limelightName).getTranslation().getNorm());
        /* 
        String llname = LimelightConstants.llObjectDetectionName; 
        Vision.visionTargetLocation = Vision.transformTargetLocation(robotContainer.swerveSubsystem.getPose(), llname); 
        HashSet<Integer> tagsFound = new HashSet<>();
        for (int i = 0; i < LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.llLocalizationName).tagCount; i++) {
            tagsFound.add(LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.llLocalizationName).rawFiducials[i].id);
        }
        Vision.tagIds.removeIf(tagId -> !tagsFound.contains(tagId));
        tagsFound.forEach(tagId -> {
            if(!Vision.tagIds.contains(tagId)){
                Vision.tagIds.add(tagId);
            }
        });
        */
        //Logger.recordOutput(llname + ".pipelineIndex", LimelightHelpers.getCurrentPipelineIndex(llname));
        //Logger.recordOutput(llname + ".pipelineName", Vision.getPipelineName(llname));
        //Logger.recordOutput(llname + ".objectDetected", LimelightHelpers.getTV(llname));

        //VisionGamePiece.stabilize(limelightName);
    }

    public String getLimelightName(){
        return limelightName;
    }


    //This method takes in blue alliance april tags and checks if it is the coral station
    public boolean isBlueAllianceCoralStation(int id) {
        return id == 12 || id == 13;
    }

    // This method takes in blue alliance april tags and checks if it is the processor
    public boolean isBlueAllianceProcessor(int id) {
        return id == 4;
    }

    //This method takes in blue alliance april tags and checks if it is the reef
    public boolean isBlueAllianceReef(int id) {
        return id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22;
    }

    // This method takes in blue alliance april tags and checks if it is the barge
    public boolean isBlueAllianceBarge(int id) {
        return id == 4 || id == 14;

    }

    // this method takes in a parameter of the april tag and checks if it is at the coral station
    public boolean isRedAllianceCoralStation(int id) {
        return id == 1 || id == 2;

    }

    // this method takes in a parameter of the april tag and checks if it is at the processor
    public boolean isRedAllianceProcessor(int id) {
        return id == 3;

    }

    // this method takes in a parameter of the april tag and checks if it is at the reef
    public boolean isRedAllianceReef(int id) {
        return id == 6 || id == 7 || id == 8 || id == 9 || id == 10 || id == 11;

    }

    // this method takes in a parameter of the april tag and checks if it is at the barge
    public boolean isRedAllianceBarge(int id) {
        return id == 5;

    }

    // adjusts the robot position based on april tag position and a preset offset
    public Pose2d transformPosition(Pose2d aprilTagPos, double offsetDistance) {

        double givenX = aprilTagPos.getX();
        double givenY = aprilTagPos.getY();
        double givenRot = aprilTagPos.getRotation().getRadians();

        double adjustedX = givenX + offsetDistance * Math.cos(givenRot);
        double adjustedY = givenY + offsetDistance * Math.sin(givenRot);

        Pose2d endResult = new Pose2d(adjustedX, adjustedY, Rotation2d.fromRadians(givenRot));

        return endResult;

        /* is no math fun?
            Transform2d dist = new Transform2d(
            new Translation2d(-offsetDistance, 0.0), 
            new Rotation2d(0.0)           
            );
            return originalPos.transformBy(dist);
         */

    }

    public Pose2d scootRight(Pose2d adjustedPos, double scootDist) {
        double initialX = adjustedPos.getX();
        double initialY = adjustedPos.getY();
        double initialRot = adjustedPos.getRotation().getRadians();

        double scootedX = initialX + scootDist * Math.sin(initialRot);
        double scootedY = initialY - scootDist * Math.cos(initialRot);

        Pose2d scootingRight = new Pose2d(scootedX, scootedY, Rotation2d.fromRadians(initialRot));

        return scootingRight;

        /* is no math fun?
            Transform2d right = new Transform2d(
                new Translation2d(0.0, -scootDist), 
                new Rotation2d(0.0)
            );
            return originalPos.transformBy(right);
         */
    }

    public Pose2d scootLeft(Pose2d adjustedPos, double scootDist) {
        double initialX = adjustedPos.getX();
        double initialY = adjustedPos.getY();
        double initialRot = adjustedPos.getRotation().getRadians();

        double scootedX = initialX - scootDist * Math.sin(initialRot);
        double scootedY = initialY + scootDist * Math.cos(initialRot);

        Pose2d scootingRight = new Pose2d(scootedX, scootedY, Rotation2d.fromRadians(initialRot));

        return scootingRight;

        /* is no math fun?
            Transform2d left = new Transform2d(
                new Translation2d(0.0, scootDist), 
                new Rotation2d(0.0)
            );
            return originalPos.transformBy(left);
         */
    }


    /**
     * 
     * @param targetLocation The location we are trying to go to (our convention)
     * @return The boolean is the April Tag ID for that location (field convention and respective alliance) is currently detected
     */
    public boolean isDetectingTargetID(String targetLocation){
        LimelightResults llr = VisionAprilTag.isValid(limelightName);
        if (llr != null && llr.targets_Fiducials != null){
            return llr.targets_Fiducials[0].fiducialID == getAprilTagId(targetLocation);
        }
        return false;
    }

    public int getAprilTagId(String location){
        var alliance = DriverStation.getAlliance();
        int targetId = -1;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetId = Constants.AprilTags.redAllianceIds.get(location);
        } else {
            targetId = Constants.AprilTags.blueAllianceIds.get(location);
        }
        return targetId; 
    }
}