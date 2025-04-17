// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.LimelightHelpers.LimelightResults;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

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
        Logger.recordOutput(limelightName + ".aprilTag.straightLineDist", VisionAprilTag.straightLineZAprilTag(limelightName));
        Logger.recordOutput(limelightName + ".aprilTag.horizontalDist", VisionAprilTag.horizontalOffsetXAprilTag(limelightName));

    }

    public String getLimelightName() {
        return limelightName;
    }


    public boolean isBlueAllianceCoralStation(int id) {
        return id == 12 || id == 13;
    }

    public boolean isBlueAllianceProcessor(int id) {
        return id == 3;
    }

    public boolean isBlueAllianceReef(int id) {
        return id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22;
    }

    public boolean isBlueAllianceBarge(int id) {
        return id == 4 || id == 14;

    }

    public boolean isRedAllianceCoralStation(int id) {
        return id == 1 || id == 2;

    }

    public boolean isRedAllianceProcessor(int id) {
        return id == 16;

    }

    public boolean isRedAllianceReef(int id) {
        return id == 6 || id == 7 || id == 8 || id == 9 || id == 10 || id == 11;

    }

    public boolean isRedAllianceBarge(int id) {
        return id == 5 || id == 15;
    }


    public Pose2d transformPosition(Pose2d aprilTagPos, double offsetDistance) {
        Transform2d dist = new Transform2d(
                new Translation2d(-offsetDistance, 0.0),
                new Rotation2d(180.0)
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
        Transform2d left = new Transform2d(
                new Translation2d(0.0, scootDist),
                new Rotation2d(0.0)
        );
        return adjustedPos.transformBy(left);
    }


    /**
     * Closest left reef from position
     * <p>
     */
    public Pose2d leftReef(Pose2d swervePos) {
        Pose2d finalPose = getNearestReef(swervePos);
        finalPose = scootLeft(finalPose, 0.2051);
        finalPose = transformPosition(finalPose, 0.5445);
        return finalPose;
    }


    public Pose2d getAprilTagPos(int id) {
        return AprilTags.aprilTags.get(id - 1);
    }


    /**
     * Closet right reef from position
     */
    public Pose2d rightReef(Pose2d swervePos) {
        Pose2d finalPose = getNearestReef(swervePos);
        finalPose = scootRight(finalPose, 0.1251);
        finalPose = transformPosition(finalPose, 0.5445);
        return finalPose;
    }


    public String getCurrentAlignment(Pose2d swervePose) {
        int aprilTag = getCurrentDetectedAprilTag(swervePose);
        Pose2d aprilTagPose = getAprilTagPos(aprilTag);
        Pose2d relativePose = swervePose.relativeTo(aprilTagPose);
        if (relativePose.getTranslation().getY() < 0) {
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
    }


    public int getNearestAprilTag(Pose2d swervePose) {
        Pose2d nearest = swervePose.nearest(AprilTags.aprilTags);
        int id = AprilTags.aprilTags.indexOf(nearest) + 1;
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

    public static double getStraightLineZDistance() {
        LimelightResults llf = VisionAprilTag.isValid(LimelightConstants.llFront);
        double llToFrontOfRobot = 0.46;
        int aprilTagId = -1;
        if (llf != null) {
            aprilTagId = (int) llf.targets_Fiducials[0].fiducialID;
        } else {
            List<Pose2d> reefTags = IntStream.range(0, AprilTags.aprilTags.size())
                    .filter(i -> {
                        int id = i + 1;
                        return id == 6 || id == 7 || id == 8 || id == 9 || id == 10 || id == 11 ||
                                id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22;
                    })
                    .mapToObj(i -> AprilTags.aprilTags.get(i))
                    .collect(Collectors.toList());

            Pose2d nearest = SwerveSubsystem.currentPose.nearest(reefTags);
            aprilTagId = AprilTags.aprilTags.indexOf(nearest) + 1;

            // Pose2d nearest = SwerveSubsystem.currentPose.nearest(AprilTags.aprilTags);
            // aprilTagId = AprilTags.aprilTags.indexOf(nearest) + 1;
        }
        // double distance = 0;
        // Pose2d relativePose = SwerveSubsystem.currentPose.relativeTo(AprilTags.aprilTags.get(aprilTagId-1));
        // if (relativePose.getTranslation().getY() < 0){
        //     distance = Math.abs(SwerveSubsystem.currentPose.relativeTo(AprilTags.aprilTags.get(aprilTagId - 1)).getX())
        //     - AprilTagOffsets.getOffset(aprilTagId).relativeRight - llToFrontOfRobot;
        // } else {
        //     distance = Math.abs(SwerveSubsystem.currentPose.relativeTo(AprilTags.aprilTags.get(aprilTagId - 1)).getX())
        //     - AprilTagOffsets.getOffset(aprilTagId).relativeLeft - llToFrontOfRobot;
        // }

        double distance = Math.abs(SwerveSubsystem.currentPose.relativeTo(AprilTags.aprilTags.get(aprilTagId - 1)).getX()) - llToFrontOfRobot;
        return distance;
    }

}