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

import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import com.navsight.VisionGP;



public class VisionSubsystem extends SubsystemBase {
    /**
     * Creates a new Vision.
     */
    private final String limelightName;

    private static final Set<Integer> BLUE_REEFS = Set.of(17, 18, 19, 20, 21, 22);
    private static final Set<Integer> RED_REEFS  = Set.of(6,  7,  8,  9,  10, 11);
    private static final Set<Integer> ALL_REEFS  =
        Stream.concat(BLUE_REEFS.stream(), RED_REEFS.stream())
              .collect(Collectors.toUnmodifiableSet());

    public VisionSubsystem(String limelightName) {
        this.limelightName = limelightName;
        for (int i = 0; i < AprilTags.aprilTags.size(); i++) {
            Pose2d org = AprilTags.aprilTags.get(i);

            Pose2d left = scootBack(scootRight(org, 0.1251), 0.5445);
            Pose2d right = scootBack(scootLeft(org, 0.2051), 0.5445);

            Positions.allAutoPositions.add(left);
            Positions.allAutoPositions.add(right);
        }
        
    }


    @Override
    public void periodic() {
    }


    public Pose2d scootBack(Pose2d aprilTagPos, double offsetDistance) {
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
        finalPose = scootBack(finalPose, 0.5445);
        return finalPose;
    }


    /**
     * Closet right reef from position
     */
    public Pose2d rightReef(Pose2d swervePos) {
        Pose2d finalPose = getNearestReef(swervePos);
        finalPose = scootRight(finalPose, 0.1251);
        finalPose = scootBack(finalPose, 0.5445);
        return finalPose;
    }

    public Pose2d getNearestReef(Pose2d swervePos) {
        List<Pose2d> tags = AprilTags.aprilTags;
        return IntStream.range(0, tags.size())
                .filter(i -> ALL_REEFS.contains(i + 1))
                .mapToObj(tags::get)
                .min(Comparator.comparingDouble(
                    tag -> tag.getTranslation().getDistance(swervePos.getTranslation())
                ))
                .orElse(swervePos);
    }

}