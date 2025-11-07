package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;
    private boolean fieldRelative;
    private final PPHolonomicDriveController driveController;

    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, XboxController xbox) {
        this.xbox = xbox;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        this.driveController = AutoConstants.pathFollowerConfig;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (xbox.getLeftBumperButton()) {
            PathPlannerTrajectoryState trajectoryState = new PathPlannerTrajectoryState();
            trajectoryState.pose = swerveSubsystem.getPose().nearest(Constants.AprilTags.aprilTags);
            swerveSubsystem.driveRobotRelative(driveController.calculateRobotRelativeSpeeds(swerveSubsystem.getPose(), trajectoryState));
        } else if (xbox.getRightBumperButton()) {
            PathPlannerTrajectoryState trajectoryState = new PathPlannerTrajectoryState();
            trajectoryState.pose = swerveSubsystem.getPose().nearest(Constants.AprilTags.aprilTags);
            swerveSubsystem.driveRobotRelative(driveController.calculateRobotRelativeSpeeds(swerveSubsystem.getPose(), trajectoryState));
        } else if (xbox.getLeftTriggerAxis() > 0.1) {  //if reef (target) relative mode pressed
            swerveSubsystem.targetRelativeDrive(Positions.REEF_CENTERS, xbox.getLeftY(), xbox.getLeftX());
            // for example if driving relative to april tag do
            // swerveSubsystem.targetRelativeDrive(Constants.AprilTags.aprilTags, xbox.getLeftY(), xbox.getLeftX());
            // if for example driving relative to gamepiece do
            // swerveSubsystem.targetRelativeDrive(new ArrayList<>(List.of(VisionGamePiece.getFieldPose())), xbox.getLeftY(), xbox.getLeftX());
        } else if (!(xbox.getRightTriggerAxis() > 0.1)) {  //if trigger(booster) not pressed
            fieldRelative = true;
            swerveSubsystem.drive(
<<<<<<< Updated upstream
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftY(), 2), xbox.getLeftY())) * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftX(), 2), xbox.getLeftX())) * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband((xbox.getRightX()) * 0.3, OIConstants.kDriveDeadband),
=======
<<<<<<< HEAD
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftY(), 2), xbox.getLeftY())), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftX(), 2), xbox.getLeftX())), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband((xbox.getRightX()) * 0.1, OIConstants.kDriveDeadband),
=======
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftY(), 2), xbox.getLeftY())) * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftX(), 2), xbox.getLeftX())) * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband((xbox.getRightX()) * 0.3, OIConstants.kDriveDeadband),
>>>>>>> 90245c6dadfdc517a561a15ef84624261c81f8de
>>>>>>> Stashed changes
                    swerveSubsystem.isFieldRelative() && fieldRelative, true);
        } else {
            // fast mode (or can be booster too) has no slew rate/rate limit
            // fieldRelative = true;
            // swerveSubsystem.drive(
            //         -MathUtil.applyDeadband(invertIfRed(xbox.getLeftY()), OIConstants.kDriveDeadband),
            //         -MathUtil.applyDeadband(invertIfRed(xbox.getLeftX()), OIConstants.kDriveDeadband),
            //         -MathUtil.applyDeadband((xbox.getRightX() * 0.5), OIConstants.kDriveDeadband),
            //         swerveSubsystem.isFieldRelative() && fieldRelative, false);
        }

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double invertIfRed(double num) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return -num;
            }
        }
        return num;
    }


}