package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionAprilTag;
import frc.robot.util.SwerveUtils;

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
            // fieldRelative = false;
            // swerveSubsystem.drive(
            //         -VisionAprilTag.limelight_rangeSpeedZ_aprilTag(LimelightConstants.llBack),
            //         -VisionAprilTag.horizontalOffsetSpeedXAprilTag(LimelightConstants.llBack),
            //         VisionAprilTag.limelight_aimSpeedX_proportional(LimelightConstants.llBack),
            //         swerveSubsystem.isFieldRelative() && fieldRelative, false);
        } else if (xbox.getRightBumperButton()) { //for forwards april tag align
            fieldRelative = false;
            // swerveSubsystem.drive(
            //         VisionAprilTag.limelight_rangeSpeedZ_aprilTag(LimelightConstants.llFront),
            //         VisionAprilTag.horizontalOffsetSpeedXAprilTag(LimelightConstants.llFront),
            //         VisionAprilTag.limelight_aimSpeedX_proportional(LimelightConstants.llFront),
            //         swerveSubsystem.isFieldRelative() && fieldRelative, false);
        } else if (!(xbox.getRightTriggerAxis() > 0.1)) {  //if trigger(booster) not pressed
            fieldRelative = true;
            swerveSubsystem.drive(
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftY(), 2), xbox.getLeftY())) * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftX(), 2), xbox.getLeftX())) * 0.5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband((xbox.getRightX()) * 0.25, OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative() && fieldRelative, true);
        } else if ((xbox.getLeftTriggerAxis() > 0.1)) {  //if reef relative mode pressed
            reefRelativeDrive();
            //headingLockRobotRelative();
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


private void reefRelativeDrive() {
    // I would need to explain what this does so I will just write it out here
    Pose2d currentPose = swerveSubsystem.getPose();
    // From the current position it gets the nearest reef (either blue or red alliance)
    Pose2d nearestReef = currentPose.nearest(Positions.REEF_CENTERS);
    double robotX = currentPose.getX();
    double robotY = currentPose.getY();
    double centerX = nearestReef.getX();
    double centerY = nearestReef.getY();
    // What dx and dy represent is the difference/delta between the robot's center x,y coordinate and reef's x,y coordinate
    double dx = centerX - robotX;
    double dy = centerY - robotY;
    // Now if you have the x and the y, remember SOHCAHTOA you would need to do inverse tangent (thats what atan2 method does) to find the theta to center of reef from robot
    // NOTE: atan2 always returns the right coordinate quadrant (-pi, pi) unlike actual inverse tangent (atan method) that is just limited to two quadrants (-pi/2, pi/2)
    double angleToCenter = Math.atan2(dy, dx);
    // Remember with reef relative driving, forward is towards the reef's center and backwards is away
    // Left would be moving clockwise in a circle around reef and right is counterclockwise
    // These get the current controller values (from -1 to 1) to determine % speed of max and direction 
    double forwardVal = xbox.getLeftY(); 
    double sidewaysVal = xbox.getLeftX();  
    // Remember SOHCAHTOA again, we can get the x component with cos and y with sin (these components are for helping go forward or backwards relative to the reef)
    double forwardUnitX = Math.cos(angleToCenter);
    double forwardUnitY = Math.sin(angleToCenter);
    // Remember since we are moving in a circle, you are essentially constant moving tangentially to the angle it current is to the center (circular motion basics, tangential velocity)
    // We calculate the tangential angle by adding 90 degrees (pi/2) to the angle to center
    double sidewaysTangentialAngle = angleToCenter + (Math.PI / 2.0);
    // Again the x, y components using SOHCAHTOA (these componenets are for helping go counterclockwise or clockwise around the reef)
    double sidewaysUnitX = Math.cos(sidewaysTangentialAngle);
    double sidewaysUnitY = Math.sin(sidewaysTangentialAngle);
    // Finally you combine all componenets together to get a reef relative mode
    // Let me explain how this works
    // Forward and sideway values are supplied by controllers and they are the % of max speed to go
    // Since we are doing reef relative, we need to convert it into field x and y axis (0,0 at reef cebter) so you can get the moving forward/backward relative to reef (radialUnit/forwardUnit) and moving in a circle (sidewaysUnit/tangentialUnit) and combine their x and y's to get final x and y units
    // With speed limits from controllers
    double fieldX = forwardVal * forwardUnitX + sidewaysVal * sidewaysUnitX;
    double fieldY = forwardVal * forwardUnitY + sidewaysVal * sidewaysUnitY;
    // The current heading (rotation) of the robot
    double currentHeading = currentPose.getRotation().getRadians();
    // So find the difference between current angle of the robot and the angle it should be facing to the center
    double headingError = SwerveUtils.AngleDifference(angleToCenter, currentHeading);
    // Now based on that difference we can run a simple P (proportional) based control
    // kRot is the P value (we need to fine tune this) to help with rotation
    double kRot = Math.PI / 10.0;
    // Using this, we can essentially run a P based control where you just multiply the error diff by P to get the speed it should be going as a %
    double rotCmd = kRot * headingError;

    swerveSubsystem.drive(fieldX, fieldY, rotCmd, false, true);
}



public void headingLockRobotRelative() {
    Pose2d currentPose = swerveSubsystem.getPose();
    Pose2d nearestReef = currentPose.nearest(Positions.REEF_CENTERS);

    double dx = nearestReef.getX() - currentPose.getX();
    double dy = nearestReef.getY() - currentPose.getY();
    double angleToCenter = Math.atan2(dy, dx);

    double forwardVal = xbox.getLeftY();
    double strafeVal = xbox.getLeftX();

    double currentHeading = currentPose.getRotation().getRadians();
    double headingError = SwerveUtils.AngleDifference(angleToCenter, currentHeading);
    double kRot = Math.PI / 10.0;
    double rotCmd = kRot * headingError;

    swerveSubsystem.drive(strafeVal, forwardVal, rotCmd, false, true);
}


}