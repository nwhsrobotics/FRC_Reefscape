package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionAprilTag;

public class SwerveJoystickDefaultCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;
    private boolean fieldRelative;


    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, XboxController xbox) {
        this.xbox = xbox;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //assuming 2 limelights
        /*if (xbox.getLeftBumperButton()) {  //for object detection aligning
            //while using Limelight, turn off field-relative driving.
            fieldRelative = false;
            swerveSubsystem.drive(
                    VisionGamePiece.limelight_rangeZ_proportional(LimelightConstants.llObjectDetectionNameForwards),
                    0,
                    VisionGamePiece.limelight_aimX_proportional(LimelightConstants.llObjectDetectionNameForwards),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);

        }*/
        if (xbox.getLeftBumperButton()) {  //for back vision april tag detection aligning
            //while using Limelight, turn off field-relative driving.
            fieldRelative = false;
            swerveSubsystem.drive(
                    -VisionAprilTag.limelight_rangeSpeedZ_aprilTag(LimelightConstants.llBack),
                    -VisionAprilTag.horizontalOffsetSpeedXAprilTag(LimelightConstants.llBack),
                    VisionAprilTag.limelight_aimSpeedX_proportional(LimelightConstants.llBack),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);
        } else if (xbox.getRightBumperButton()) { //for forwards april tag align
            fieldRelative = false;
            swerveSubsystem.drive(
                    VisionAprilTag.limelight_rangeSpeedZ_aprilTag(LimelightConstants.llFront),
                    VisionAprilTag.horizontalOffsetSpeedXAprilTag(LimelightConstants.llFront),
                    VisionAprilTag.limelight_aimSpeedX_proportional(LimelightConstants.llFront),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);
        } else if (!(xbox.getRightTriggerAxis() > 0.1)) {  //if trigger(booster) not pressed
            fieldRelative = true;
            swerveSubsystem.drive(
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftY(), 2), xbox.getLeftY())) * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftX(), 2), xbox.getLeftX())) * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband((xbox.getRightX()), OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative() && fieldRelative, true);
        } else {
            // fast mode (or can be booster too) has no slew rate/rate limit
            fieldRelative = true;
            swerveSubsystem.drive(
                    -MathUtil.applyDeadband(invertIfRed(xbox.getLeftY()), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(xbox.getLeftX()), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband((xbox.getRightX() * 0.5), OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);
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