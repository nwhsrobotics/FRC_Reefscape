package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;

/**
 * Command that drives the robot to a target pose using a
 * {@link PPHolonomicDriveController}. The command finishes once the robot is
 * aligned with the pose and moving slowly.
 */
public class PosePIDCommand extends Command {

    private final SwerveSubsystem swerve;
    private final Pose2d targetPose;
    private final PPHolonomicDriveController driveController;

    private static final Rotation2d ROT_TOL = Rotation2d.fromDegrees(2.0);
    private static final Distance POS_TOL = Centimeter.of(1.0);
    private static final LinearVelocity SPD_TOL = InchesPerSecond.of(1);
    private static final Time DEBOUNCE = Seconds.of(0.05);

    private final Trigger completionTrigger;
    private final Trigger debouncedCompletion;

    /**
     * Construct a command to drive to the given pose.
     *
     * @param swerve     drive subsystem
     * @param targetPose pose to reach
     */
    public PosePIDCommand(SwerveSubsystem swerve, Pose2d targetPose) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        this.driveController = AutoConstants.pathFollowerConfig;

        completionTrigger = new Trigger(() -> {
            Pose2d cur = swerve.getPose();
            Pose2d diff = cur.relativeTo(targetPose);
            boolean rotAligned = MathUtil.isNear(
                    0.0, diff.getRotation().getRotations(),
                    ROT_TOL.getRotations(), 0.0, 1.0);
            boolean posAligned = diff.getTranslation().getNorm() < POS_TOL.in(Meters);
            boolean slow = swerve.getSpeeds().omegaRadiansPerSecond < SPD_TOL.in(MetersPerSecond);
            return rotAligned && posAligned && slow;
        });
        debouncedCompletion = completionTrigger.debounce(DEBOUNCE.in(Seconds));
        addRequirements(swerve);
    }

    /**
     * Convenience factory that returns a timed command which stops the drive at
     * the end.
     */
    public static Command create(SwerveSubsystem swerve, Pose2d tgt, Time timeout) {
        return new PosePIDCommand(swerve, tgt)
                .withTimeout(timeout)
                .andThen(() -> swerve.stopModules());
    }

    /**
     * Nothing to initialise.
     */
    @Override
    public void initialize() {
    }

    /**
     * Drives toward the target each loop.
     */
    @Override
    public void execute() {
        PathPlannerTrajectoryState state = new PathPlannerTrajectoryState();
        state.pose = targetPose;
        swerve.driveRobotRelative(
                driveController.calculateRobotRelativeSpeeds(swerve.getPose(), state));
    }

    /**
     * Nothing to clean up.
     */
    @Override
    public void end(boolean interrupted) {
    }

    /**
     * @return true once the robot is aligned with the target pose
     */
    @Override
    public boolean isFinished() {
        return debouncedCompletion.getAsBoolean();
    }
}
