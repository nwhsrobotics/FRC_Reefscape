package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.autos.PosePIDCommand;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import org.littletonrobotics.junction.Logger;

/**
 * Helper class for autonavigation.
 * <p>
 * Works in conjunction with the swerve subsystem.
 */
public class AutoNavigation {
    private Command navigationCommand;
    private boolean enabled = false;
    private final SwerveSubsystem swerve;

    public AutoNavigation(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    /**
     * Enable autonavigation.
     * <p>
     * If the robot is not operating under tele-op, calls to this method will be ignored.
     */
    public void enable() {
        if (!RobotState.isTeleop()) {
            return;
        }

        enabled = true;

        Logger.recordOutput("autonavigator.enabled", enabled);
    }

    /**
     * Disable autonavigation.
     * <p>
     * The current autonavigation destination will be cancelled.
     */
    public void disable() {
        pauseNavigation();
        navigationCommand = null;
        enabled = false;

        Logger.recordOutput("autonavigator.enabled", enabled);
    }

    /**
     * Returns whether autonavigation is enabled.
     *
     * @return - boolean representing whether autonavigation is enabled.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Toggle autonavigation.
     * <p>
     * Calls ".enable()" if currently disabled, and calls ".disable()" if currently enabled.
     */
    public void toggle() {
        if (enabled) {
            disable();
        } else {
            enable();
        }
    }

    /**
     * Stop current autonavigation, but do not clear current navigation command.
     * <p>
     * This is useful for accomodating manual override and et cetera.
     * Run ".resumeNavigation()" to continue autonavigation.
     * <p>
     * It is safe to execute this command if no autonavigation commmand is currently scheduled.
     */
    public void pauseNavigation() {
        if (navigationCommand == null) {
            return;
        }

        navigationCommand.cancel();
    }

    /**
     * Start current autonavigation, but do not clear current navigation command.
     * <p>
     * This is useful for accomodating manual override and et cetera.
     * Run ".pauseNavigation()" to stop autonavigation.
     * <p>
     * It is safe to execute this command if no autonavigation commmand is currently scheduled,
     * or if the autonavigation command is already scheduled.
     */
    public void resumeNavigation() {
        if (navigationCommand == null || navigationCommand.isScheduled()) {
            return;
        }
        navigationCommand.addRequirements(swerve);
        navigationCommand.schedule();
    }

    /**
     * Navigate to position.
     * <p>
     * This will initialize a new navigation command.
     * If an existing navigation command is scheduled, that command will be cancelled,
     * before being overwritten.
     * <p>
     * If autonavigation is disabled, calls to this method will be ignored.
     * Returning command, was void before
     *
     * @param destination - position to navigate to.
     */
    public void navigateTo(Pose2d destination) {
        // if (!enabled || !RobotState.isTeleop()) {
        //     //return new InstantCommand();
        //     return;
        // }
        if (!RobotState.isTeleop()){
            return;
        }
        enable();

        if (navigationCommand != null && navigationCommand.isScheduled()) {
            navigationCommand.cancel();
            // navigationCommand.end(true);
            // navigationCommand = null;
        }

        //navigationCommand = .pathfindToPosition(destination);
        //Path on fly better?
        navigationCommand = pathOnTheFlyToPosition(destination);
        navigationCommand.addRequirements(swerve);
        navigationCommand.schedule();
        Logger.recordOutput("autonavigator.destination", destination);
        //return navigationCommand;
        
    }
    
    /**
     * Finds a path and follows it based on the specified path name.
     * Loads the path from a file, sets constraints, and uses AutoBuilder to create a pathfinding command.
     *
     * @param pathName The name of the path file to load and follow.
     */
    public Command pathFindThenFollowPath(String pathName) {
        Command pathfindingCommand;
        // Load the path we want to pathfind to and follow
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);

            // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
            PathConstraints constraints = new PathConstraints(
                    DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8.0, AutoConstants.kMaxAccelerationMetersPerSecondSquared / 8.0,
                    AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared / 2.0);

            // What pathfinding does is pathfind to the start of a path and then continue along that path.
            // If you don't want to continue along the path, you can make it pathfind to a specific location.
            //Either use this (
            pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                    path,
                    constraints
            );
            //maybe no need to schedule it or addrequirements
            //pathfindingCommand.addRequirements(this);
            //pathfindingCommand.schedule();
            //return pathfindingCommand;
            Command e = AutoBuilder.followPath(path);
            //e.schedule();
            // ) or just this
            return e;
        } catch (Exception io) {
            io.printStackTrace();
        }
        return new InstantCommand();
    }

    /**
     * Run pathfinding to given position.
     *
     * @param position - position to pathfind to.
     * @return - scheduled pathfinding command.
     */
    public Command pathfindToPosition(Pose2d position) {
        //Maybe use on the fly path? Less overhead
        Command command = AutoBuilder.pathfindToPose(
                        position,
                        AutoConstants.kPathfindingConstraints,
                        0.0 // Goal end velocity in meters/sec
                ).andThen(PosePIDCommand.create(swerve, position, Seconds.of(1/3)));
                // .alongWith(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceBlink(LimelightConstants.llFront)))
                // .andThen(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOn(LimelightConstants.llFront)));
        //.andThen(pathOnTheFlyToPosition(position));

        return command;
    }


    public Command pathOnTheFlyToPosition(Pose2d targetPose) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(swerve.getPose().getTranslation(), getPathVelocityHeading(swerve.getSpeeds(), targetPose)),
                targetPose
        );

        //getPathVelocityHeading(getSpeeds(), targetPose)
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                AutoConstants.kPathfindingConstraints,
                new IdealStartingState(getVelocityMagnitude(swerve.getSpeeds()), swerve.getPose().getRotation()),
                new GoalEndState(
                        0.0,
                        targetPose.getRotation()
                )
        );

        path.preventFlipping = true;

        //TODO: Reduce PosePIDCommand time to 0.2s
        Command followPathCommand = AutoBuilder.followPath(path)
                .andThen(PosePIDCommand.create(swerve, targetPose, Seconds.of(1/3)));
                // .alongWith(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceBlink(LimelightConstants.llFront)))
                // .andThen(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOn(LimelightConstants.llFront)));

        return followPathCommand;
    }

    public Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.minus(swerve.getPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    public LinearVelocity getVelocityMagnitude(ChassisSpeeds cs) {
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }


}
