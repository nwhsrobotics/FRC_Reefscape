// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AprilTags;
import frc.robot.Constants.Positions;

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
        for (int i = 0; i < AprilTags.aprilTags.size(); i++) {
            Pose2d org = AprilTags.aprilTags.get(i);

            Pose2d left = transformPosition(scootRight(org, 0.1251), 0.5445);
            Pose2d right = transformPosition(scootLeft(org, 0.2051), 0.5445);

            Positions.allAutoPositions.add(left);
            Positions.allAutoPositions.add(right);
        }
    }


    @Override
    public void periodic() {
        System.out.println(AprilTags.aprilTags);
        
    }

    public String getLimelightName() {
        return limelightName;
    }


    public boolean isBlueAllianceReef(int id) {
        return id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22;
    }


    public boolean isRedAllianceReef(int id) {
        return id == 6 || id == 7 || id == 8 || id == 9 || id == 10 || id == 11;

    }


    public Pose2d transformPosition(Pose2d aprilTagPos, double offsetDistance) {
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

}