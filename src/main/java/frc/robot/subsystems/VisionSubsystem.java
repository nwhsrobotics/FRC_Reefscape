// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagOffsets;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TagOffset;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    /**
     * Creates a new Vision.
     */
    private final String limelightName;

    public VisionSubsystem(String limelightName) {
        this.limelightName = limelightName;
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run\
        // Logger.recordOutput(limelightName + ".straightLineDist", VisionGamePiece.straightLineZDistance(limelightName));
        // Logger.recordOutput(limelightName + ".verticalDist", VisionGamePiece.verticalYOffsetDistance(limelightName));
        // Logger.recordOutput(limelightName + ".horizontalDist", VisionGamePiece.horizontalOffestXDistance(limelightName));
        // Logger.recordOutput(limelightName + ".3dHypotnuese", VisionGamePiece.full3DDistance(limelightName));
        // Logger.recordOutput(limelightName + ".2dHypotnuese", VisionGamePiece.hypotenuseDistanceXandZ(limelightName));
        // Logger.recordOutput(limelightName + ".rot", VisionGamePiece.hypotenuseDistanceXandZ(limelightName));
        // Logger.recordOutput(limelightName + ".detect", LimelightHelpers.getTY(limelightName));
        // Pose2d targetPose = VisionGamePiece.transformTargetLocation(new Pose2d(), limelightName);
        // Logger.recordOutput(limelightName + ".target", targetPose.toString());
        // Logger.recordOutput(limelightName + ".targetX", targetPose.getX());
        // Logger.recordOutput(limelightName + ".targetY", targetPose.getY());
        // Logger.recordOutput(limelightName + ".targetDegrees", targetPose.getRotation().getDegrees());
        // Logger.recordOutput(limelightName + ".targetRadians", targetPose.getRotation().getRadians());
        // Logger.recordOutput(limelightName + ".targetRotation", targetPose.getRotation().getRotations());
        // Logger.recordOutput(limelightName + ".getOriginDistance", targetPose.getTranslation().getNorm());

        Logger.recordOutput(limelightName + ".aprilTag.straightLineDist", VisionAprilTag.straightLineZAprilTag(limelightName));
        Logger.recordOutput(limelightName + ".aprilTag.verticalDist", VisionAprilTag.verticalYOffsetDistance(limelightName));
        Logger.recordOutput(limelightName + ".aprilTag.horizontalDist", VisionAprilTag.horizontalOffsetXAprilTag(limelightName));
        // Logger.recordOutput(limelightName + ".aprilTag.3dHypotnuese", VisionAprilTag.hypotenuseDistanceXandZ(limelightName));
        // Logger.recordOutput(limelightName + ".aprilTag.detect", LimelightHelpers.getTY(limelightName));
        // Pose2d aprilTag = VisionAprilTag.transformTargetLocation(new Pose2d(), limelightName);
        // Logger.recordOutput(limelightName + ".aprilTag.target", aprilTag.toString());
        // Logger.recordOutput(limelightName + ".aprilTag.targetX", aprilTag.getX());
        // Logger.recordOutput(limelightName + ".aprilTag.targetY", aprilTag.getY());
        // Logger.recordOutput(limelightName + ".aprilTag.targetDegrees", aprilTag.getRotation().getDegrees());
        // Logger.recordOutput(limelightName + ".aprilTag.targetRadians", aprilTag.getRotation().getRadians());
        // Logger.recordOutput(limelightName + ".aprilTag.targetRotation", aprilTag.getRotation().getRotations());
        // Logger.recordOutput(limelightName + ".aprilTag.getOriginDistance", aprilTag.getTranslation().getNorm());

        // Logger.recordOutput(limelightName + ".tx", LimelightHelpers.getTX(limelightName));
        // Logger.recordOutput(limelightName + ".ty", LimelightHelpers.getTY(limelightName));
        // Logger.recordOutput(limelightName + ".ta", LimelightHelpers.getTA(limelightName));

        Logger.recordOutput("Crosshair", "----------------------------------X-----------------------------");
        for (int i = 0; i < AprilTags.aprilTags.size(); i++) {
            int tagID = i + 1;
            Pose2d org = AprilTags.aprilTags.get(i);
            TagOffset offsets = AprilTagOffsets.getOffset(tagID);

            Pose2d left = transformPosition(scootRight(org, offsets.right), offsets.back);
            Pose2d right = transformPosition(scootLeft(org, offsets.left), offsets.back);

            Logger.recordOutput(
                    "Tag:" + tagID,
                    String.format(
                            " Original( x=%.3f, y=%.3f, rot=%.1f° ) | Left( x=%.3f, y=%.3f, rot=%.1f° ) | Right( x=%.3f, y=%.3f, rot=%.1f° )",
                            org.getX(), org.getY(), org.getRotation().getDegrees(),
                            left.getX(), left.getY(), left.getRotation().getDegrees(),
                            right.getX(), right.getY(), right.getRotation().getDegrees()
                    )
            );
        }
        
        
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
        //VisionAprilTag.isValid(limelightName);

        //Logger.recordOutput("detectingValid", isDetectingTargetID("[1A]"));
    }

    public String getLimelightName() {
        return limelightName;
    }


    //This method takes in blue alliance april tags and checks if it is the coral station
    public boolean isBlueAllianceCoralStation(int id) {
        return id == 12 || id == 13;
    }

    // This method takes in blue alliance april tags and checks if it is the processor
    public boolean isBlueAllianceProcessor(int id) {
        return id == 3;
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
        return id == 16;

    }

    // this method takes in a parameter of the april tag and checks if it is at the reef
    public boolean isRedAllianceReef(int id) {
        return id == 6 || id == 7 || id == 8 || id == 9 || id == 10 || id == 11;

    }

    // this method takes in a parameter of the april tag and checks if it is at the barge
    public boolean isRedAllianceBarge(int id) {
        return id == 5 || id == 15;

    }

    // adjusts the robot position based on april tag position and a preset offset
    public Pose2d transformPosition(Pose2d aprilTagPos, double offsetDistance) {

        // double givenX = aprilTagPos.getX();
        // double givenY = aprilTagPos.getY();
        // double givenRot = aprilTagPos.getRotation().getRadians();

        // double adjustedX = givenX + offsetDistance * Math.cos(givenRot);
        // double adjustedY = givenY + offsetDistance * Math.sin(givenRot);

        // Pose2d endResult = new Pose2d(adjustedX, adjustedY, Rotation2d.fromRadians(givenRot));

        // return endResult;

        // is no math fun?
        Transform2d dist = new Transform2d(
                new Translation2d(-offsetDistance, 0.0),
                new Rotation2d(0.0)
        );
        return aprilTagPos.transformBy(dist);

    }

    /**
     * NOTE: This method scoots left relative to the driver's camera POV
     * NOT relative to april tag
     *
     * @param adjustedPos The position to scoot left
     * @param scootDist   How much left should it be scooted by?
     * @return Lef positon of adjustedPos relative to driver's camera POV
     */
    public Pose2d scootRight(Pose2d adjustedPos, double scootDist) {
        // double initialX = adjustedPos.getX();
        // double initialY = adjustedPos.getY();
        // double initialRot = adjustedPos.getRotation().getRadians();

        // double scootedX = initialX + scootDist * Math.sin(initialRot);
        // double scootedY = initialY - scootDist * Math.cos(initialRot);

        // Pose2d scootingRight = new Pose2d(scootedX, scootedY, Rotation2d.fromRadians(initialRot));

        // return scootingRight;

        // is no math fun?
        Transform2d right = new Transform2d(
                new Translation2d(0.0, -scootDist),
                new Rotation2d(0.0)
        );
        return adjustedPos.transformBy(right);

    }


    /**
     * NOTE: This method scoots right relative to the driver's camera POV
     * NOT relative to april tag
     *
     * @param adjustedPos The position to scoot right
     * @param scootDist   How much right should it be scooted by?
     * @return Right positon of adjustedPos relative to driver's camera POV
     */
    public Pose2d scootLeft(Pose2d adjustedPos, double scootDist) {
        // double initialX = adjustedPos.getX();
        // double initialY = adjustedPos.getY();
        // double initialRot = adjustedPos.getRotation().getRadians();

        // double scootedX = initialX - scootDist * Math.sin(initialRot);
        // double scootedY = initialY + scootDist * Math.cos(initialRot);

        // Pose2d scootingRight = new Pose2d(scootedX, scootedY, Rotation2d.fromRadians(initialRot));

        // return scootingRight;

        // is no math fun?
        Transform2d left = new Transform2d(
                new Translation2d(0.0, scootDist),
                new Rotation2d(0.0)
        );
        return adjustedPos.transformBy(left);
    }


    /**
     * @param targetLocation The location we are trying to go to (our convention)
     * @return The boolean is the April Tag ID for that location (field convention and respective alliance) is currently detected
     */
    public boolean isDetectingTargetID(String targetLocation) {
        LimelightResults llr = VisionAprilTag.isValid(limelightName);
        if (llr != null && llr.targets_Fiducials != null) {
            // distance limiter and adaptive delay hold
            //  && llr.targets_Fiducials[0].getTargetPose_RobotSpace2D().getTranslation().getNorm() < 2
            return llr.targets_Fiducials[0].fiducialID == getAprilTagId(targetLocation)
                    && llr.targets_Fiducials[0].getRobotPose_TargetSpace2D().getTranslation().getNorm() < 2;
        }
        return false;
    }

    public int getAprilTagId(String location) {
        var alliance = DriverStation.getAlliance();
        int targetId = -1;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            targetId = Constants.AprilTags.redAllianceIds.getOrDefault(location, -1);
        } else {
            targetId = Constants.AprilTags.blueAllianceIds.getOrDefault(location, -1);
        }
        return targetId;
    }


    /**
     * Closest left reef from position
     * <p>
     */
    public Pose2d leftReef(Pose2d swervePos) {
        LimelightResults llf = VisionAprilTag.isValid(LimelightConstants.llFront);
        Pose2d finalPose = swervePos;
        int aprilTagId = -1;
        if (llf != null) {
            aprilTagId = (int) llf.targets_Fiducials[0].fiducialID;
            finalPose = getAprilTagPos(aprilTagId);
        } else {
            finalPose = getNearestReef(swervePos);
        }
        TagOffset offset = AprilTagOffsets.getOffset(aprilTagId);
        finalPose = scootLeft(finalPose, offset.left);
        finalPose = transformPosition(finalPose, offset.back);
        return finalPose;
    }


    public Pose2d getAprilTagPos(int id) {
        return AprilTags.aprilTags.get(id - 1);
    }


    /**
     * Closet right reef from position
     */
    public Pose2d rightReef(Pose2d swervePos) {
        LimelightResults llf = VisionAprilTag.isValid(LimelightConstants.llFront);
        Pose2d finalPose = swervePos;
        int aprilTagId = -1;
        if (llf != null) {
            aprilTagId = (int) llf.targets_Fiducials[0].fiducialID;
            finalPose = getAprilTagPos(aprilTagId);
        } else {
            finalPose = getNearestReef(swervePos);
        }
        TagOffset offset = AprilTagOffsets.getOffset(aprilTagId);
        finalPose = scootRight(finalPose, offset.right);
        finalPose = transformPosition(finalPose, offset.back);
        return finalPose;
    }


    public String getCurrentAlignment(Pose2d swervePose) {
        int aprilTag = getCurrentDetectedAprilTag(swervePose);
        Pose2d aprilTagPose = getAprilTagPos(aprilTag);
        Pose2d relativePose = swervePose.relativeTo(aprilTagPose);
        if (relativePose.getTranslation().getY() < 0){
            return "RIGHT";
        }
        return "LEFT";
    }
    

    public double getOffsetY(Pose2d swervePose) {
        int aprilTag = getCurrentDetectedAprilTag(swervePose);
        Pose2d aprilTagPose = getAprilTagPos(aprilTag);
        Pose2d relativePose = swervePose.relativeTo(aprilTagPose);
        return Math.abs(relativePose.getTranslation().getY());
    }

    public double getOffsetX(Pose2d swervePose) {
        int aprilTag = getCurrentDetectedAprilTag(swervePose);
        Pose2d aprilTagPose = getAprilTagPos(aprilTag);
        Pose2d relativePose = swervePose.relativeTo(aprilTagPose);
        return Math.abs(relativePose.getTranslation().getX());
    }

    public void correctTheOffset(double offsetDifY, double offsetDifX, boolean isRight, int id){
        LimelightResults llf = VisionAprilTag.isValid(LimelightConstants.llFront);
        if (llf != null) {
            int aprilTag = getCurrentDetectedAprilTag(getAprilTagPos(id));
            TagOffset offset = AprilTagOffsets.getOffset(aprilTag);
            //tune the tolerance here
            if (isRight){
                if (Math.abs(offsetDifY) > 0.025){
                    offset.right -= offsetDifY;
                }
            } else {
                if (Math.abs(offsetDifY) > 0.025){
                    offset.left -= offsetDifY;
                }
            }
            if (Math.abs(offsetDifX) > 0.05){
                offset.back -= offsetDifX;
            }
        }
    }
    

    public Pose2d getNearestReef(Pose2d swervePos) {
        Pose2d closest = swervePos;
        double dist = Integer.MAX_VALUE;
        for (int i = 0; i < AprilTags.aprilTags.size(); i++) {
            double targetDist = swervePos.getTranslation().getDistance(AprilTags.aprilTags.get(i).getTranslation());
            if (targetDist < dist) {
                if (isBlueAllianceReef(i + 1) || isRedAllianceReef(i + 1)) {
                    closest = AprilTags.aprilTags.get(i);
                    dist = targetDist;
                }
            }
        }
        return closest;


        // return swervePos.nearest(AprilTags.aprilTags);
    }
    

    public int getNearestAprilTag(Pose2d swervePose){
        Pose2d nearest = swervePose.nearest(AprilTags.aprilTags);
        int id = AprilTags.aprilTags.indexOf(nearest)+1;
        return id;
    }

    public Rotation2d getFakeAngle(Pose2d swervePos) {
        LimelightResults llf = VisionAprilTag.isValid(limelightName);
        if (llf != null) {
            int aprilTagId = (int) llf.targets_Fiducials[0].fiducialID;
            return getAprilTagPos(aprilTagId).getRotation();
        }
        return getNearestReef(swervePos).getRotation();
    }

    
    public int getCurrentDetectedAprilTag(Pose2d swervePos) {
        LimelightResults llf = VisionAprilTag.isValid(limelightName);
        if (llf != null) {
            int aprilTagId = (int) llf.targets_Fiducials[0].fiducialID;
            return aprilTagId;
        }
        return getNearestAprilTag(swervePos);
    }

    public static double getStraightLineZDistance(){
        LimelightResults llf = VisionAprilTag.isValid(LimelightConstants.llFront);
        double llToFrontOfRobot = 0.5;
        int aprilTagId = -1;
        if (llf != null) {
            aprilTagId = (int) llf.targets_Fiducials[0].fiducialID;
        } else {
            Pose2d nearest = SwerveSubsystem.currentPose.nearest(AprilTags.aprilTags);
            aprilTagId = AprilTags.aprilTags.indexOf(nearest)+1;
        }
        double distance = Math.abs(SwerveSubsystem.currentPose.relativeTo(AprilTags.aprilTags.get(aprilTagId - 1)).getX())
        - AprilTagOffsets.getOffset(aprilTagId).relative - llToFrontOfRobot;

        return distance;
    }
}