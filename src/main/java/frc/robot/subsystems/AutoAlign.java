package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import frc.robot.Constants;
import frc.robot.commands.PosePIDCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.*;
import java.util.stream.*;

public class AutoAlign {
    private static final Set<Integer> BLUE = Set.of(17,18,19,20,21,22),
                                      RED = Set.of(6,7,8,9,10,11),
                                      ALL = Stream.concat(BLUE.stream(), RED.stream()).collect(Collectors.toUnmodifiableSet());
    private final SwerveSubsystem swerve;
    private Command navCmd;

    public AutoAlign(SwerveSubsystem swerve){
        this.swerve = swerve;
    }

    public Pose2d leftReef(Pose2d robot){ 
        return buildOffset(robot,true);
    }

    public Pose2d rightReef(Pose2d robot){ 
        return buildOffset(robot,false);
    }

    private Pose2d nearestTag(Pose2d robot){
        var tags = Constants.AprilTags.aprilTags;
        return IntStream.range(0,tags.size())
                .filter(i->ALL.contains(i+1))
                .mapToObj(tags::get)
                .min(Comparator.comparingDouble(
                     t->t.getTranslation().getDistance(robot.getTranslation())))
                .orElse(robot);
    }
    private Pose2d buildOffset(Pose2d robot, boolean left){
        Pose2d base = nearestTag(robot);
        Pose2d shifted = left ? scootLeft(base,0.2051): scootRight(base,0.1251);
        return scootBack(shifted,0.5445);
    }

    /**
     * Translate the pose backwards (negative X from the driver's perspective).
     *
     * @param pose    original pose
     * @param metres  distance to move backwards in metres
     * @return translated pose
     */
    public static Pose2d scootBack(Pose2d pose, double metres) {
        return pose.transformBy(new Transform2d(
                new Translation2d(-metres, 0), new Rotation2d()));
    }

    /**
     * Immediately navigate the robot to the given pose during teleop.
     *
     * @param destination target pose to drive to
     */
    public void navigateTo(Pose2d destination) {
        if (!RobotState.isTeleop())
            return;
        if (navCmd != null && navCmd.isScheduled())
            navCmd.cancel();

        navCmd = pathOnTheFly(destination);
        navCmd.addRequirements((Subsystem) swerve);
        navCmd.schedule();
    }

    public void disable(){
        if (navCmd != null && navCmd.isScheduled())
            navCmd.cancel();
    }

    /**
     * Translate the pose to the driver's right (negative Y).
     *
     * @param pose    original pose
     * @param metres  distance to move right in metres
     * @return translated pose
     */
    public static Pose2d scootRight(Pose2d pose, double metres) {
        return pose.transformBy(new Transform2d(
                new Translation2d(0, -metres), new Rotation2d()));
    }

    /**
     * Translate the pose to the driver's left (positive Y).
     *
     * @param pose    original pose
     * @param metres  distance to move left in metres
     * @return translated pose
     */
    public static Pose2d scootLeft(Pose2d pose, double metres) {
        return pose.transformBy(new Transform2d(
                new Translation2d(0, metres), new Rotation2d()));
    }

        /**
     * Create a PathPlanner command to drive to the target pose.
     */
    private Command pathOnTheFly(Pose2d target) {
        var cfg = Constants.AutoConstants.kPathfindingConstraints;
        List<Waypoint> wps = PathPlannerPath.waypointsFromPoses(new Pose2d(swerve.getPose().getTranslation(), heading(swerve.getSpeeds(), target)), target);

        PathPlannerPath path = new PathPlannerPath(
                wps, cfg,
                new IdealStartingState(velocityMag(swerve.getSpeeds()), 
                swerve.getPose().getRotation()),
                new GoalEndState(0.0, target.getRotation()));
        path.preventFlipping = true;

        return AutoBuilder.followPath(path)
                .andThen(PosePIDCommand.create(swerve, target, Seconds.of(1)));
    }

    /** Determine the robot heading to use when starting a new path. */
    private Rotation2d heading(ChassisSpeeds cs, Pose2d tgt) {
        if (velocityMag(cs).in(MetersPerSecond) < 0.25) {
            Translation2d diff = tgt.minus(swerve.getPose()).getTranslation();
            return diff.getNorm() < 0.01 ? tgt.getRotation() : diff.getAngle();
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    /** @return magnitude of the chassis translational velocity */
    private LinearVelocity velocityMag(ChassisSpeeds cs) {
        return MetersPerSecond.of(
                new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }
}
