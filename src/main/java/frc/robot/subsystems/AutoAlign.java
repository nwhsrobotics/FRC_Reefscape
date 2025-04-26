package frc.robot.subsystems;

import com.navsight.*;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.SwerveDriveAdapter;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.*;
import java.util.stream.*;
import frc.robot.Constants.SwerveDriveAdapter;

public final class AutoAlign {
    private static boolean ready=false;
    private static final Set<Integer> BLUE=Set.of(17,18,19,20,21,22),
                                      RED =Set.of(6,7,8,9,10,11),
                                      ALL =Stream.concat(BLUE.stream(),RED.stream()).collect(Collectors.toUnmodifiableSet());

    public static void init(SwerveSubsystem swerve){
        if(ready)return;

        NavSight.init(new SwerveDriveAdapter(swerve),
                      new AutoConfig(Constants.AutoConstants.pathFollowerConfig,
                                     Constants.AutoConstants.kPathfindingConstraints));

        NavSight.registerCamera(Constants.LimelightConstants.llFront,
                new VisionCamera.LimelightConstants(
                        VisionGP.LimelightConstants.mountHeightForwards,
                        VisionGP.LimelightConstants.mountAngleForwards,
                        VisionGP.LimelightConstants.horizontalOffsetForwards,
                        VisionGP.LimelightConstants.targetHeightForwards));

        NavSight.registerCamera(Constants.LimelightConstants.llBack,
                new VisionCamera.LimelightConstants(
                        VisionGP.LimelightConstants.mountHeightBackwards,
                        VisionGP.LimelightConstants.mountAngleBackwards,
                        VisionGP.LimelightConstants.horizontalOffsetBackwards,
                        VisionGP.LimelightConstants.targetHeightBackwards));

        buildReefWaypoints(); ready=true;
    }

    public static Pose2d leftReef(Pose2d robot){ return buildOffset(robot,true ); }
    public static Pose2d rightReef(Pose2d robot){ return buildOffset(robot,false); }

    private static Pose2d nearestTag(Pose2d robot){
        var tags=Constants.AprilTags.aprilTags;
        return IntStream.range(0,tags.size())
                .filter(i->ALL.contains(i+1))
                .mapToObj(tags::get)
                .min(Comparator.comparingDouble(
                     t->t.getTranslation().getDistance(robot.getTranslation())))
                .orElse(robot);
    }
    private static Pose2d buildOffset(Pose2d robot,boolean left){
        Pose2d base=nearestTag(robot);
        Pose2d shifted=left?PoseTransforms.scootLeft(base,0.2051):
                            PoseTransforms.scootRight(base,0.1251);
        return PoseTransforms.scootBack(shifted,0.5445);
    }
    private static void buildReefWaypoints(){
        for(Pose2d tag:Constants.AprilTags.aprilTags){
            Constants.Positions.allAutoPositions.add(
                    PoseTransforms.scootBack(PoseTransforms.scootRight(tag,0.1251),0.5445));
            Constants.Positions.allAutoPositions.add(
                    PoseTransforms.scootBack(PoseTransforms.scootLeft (tag,0.2051),0.5445));}
    }
}
