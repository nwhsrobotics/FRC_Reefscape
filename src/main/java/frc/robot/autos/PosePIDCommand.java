package frc.robot.autos;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;

public class PosePIDCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Pose2d targetPose;
    private final PPHolonomicDriveController driveController;

    private static final Rotation2d ROTATION_TOLERANCE = Rotation2d.fromDegrees(2.0);
    private static final Distance POSITION_TOLERANCE = Centimeter.of(2.0);
    private static final LinearVelocity SPEED_TOLERANCE = InchesPerSecond.of(1);
    private static final Time TRIGGER_DEBOUNCE_DELAY = Seconds.of(0.05);

    private final Trigger completionTrigger;
    private final Trigger debouncedCompletionTrigger;

    public PosePIDCommand(SwerveSubsystem swerveSubsystem, Pose2d targetPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetPose = targetPose;
        this.driveController = AutoConstants.pathFollowerConfig;
        addRequirements(swerveSubsystem);
        completionTrigger = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                Pose2d currentPose = swerveSubsystem.getPose();
                Pose2d poseDifference = currentPose.relativeTo(targetPose);
                boolean isRotationAligned = MathUtil.isNear(
                        0.0,
                        poseDifference.getRotation().getRotations(),
                        ROTATION_TOLERANCE.getRotations(),
                        0.0,
                        1.0
                );
                boolean isPositionAligned = poseDifference.getTranslation().getNorm() < POSITION_TOLERANCE.in(Meters);
                boolean isSpeedLow = swerveSubsystem.getVelocityMagnitude(swerveSubsystem.getSpeeds()).baseUnitMagnitude() < SPEED_TOLERANCE.in(MetersPerSecond);
                return isRotationAligned && isPositionAligned && isSpeedLow;
            }
        });
        debouncedCompletionTrigger = completionTrigger.debounce(TRIGGER_DEBOUNCE_DELAY.in(Seconds));
    }

    public static Command create(SwerveSubsystem swerveSubsystem, Pose2d targetPose, Time timeout) {
        return new PosePIDCommand(swerveSubsystem, targetPose)
                .withTimeout(timeout)
                .andThen(() -> swerveSubsystem.stopModules());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState trajectoryState = new PathPlannerTrajectoryState();
        trajectoryState.pose = targetPose;
        swerveSubsystem.driveRobotRelative(
                driveController.calculateRobotRelativeSpeeds(swerveSubsystem.getPose(), trajectoryState)
        );
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return debouncedCompletionTrigger.getAsBoolean();
    }
}
