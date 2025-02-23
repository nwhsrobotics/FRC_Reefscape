package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionAprilTag;
import frc.robot.subsystems.VisionGamePiece;
import frc.robot.util.Buttons;

public class SwerveJoystickDefaultCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController xbox;
    private boolean fieldRelative;



    // constructor that initializes SwerveSubsystem, Joystick and adds SwerveSubsystem as a requirement
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
        /*if (xbox.getLeftBumperButton()) {  //for object detection alligning
            //while using Limelight, turn off field-relative driving.
            fieldRelative = false;
            swerveSubsystem.drive(
                    VisionGamePiece.limelight_rangeZ_proportional(LimelightConstants.llObjectDetectionNameForwards),
                    0,
                    VisionGamePiece.limelight_aimX_proportional(LimelightConstants.llObjectDetectionNameForwards),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);

        }*/
        if (xbox.getLeftBumperButton()) {  //for back vision april tag detection alligning
            //while using Limelight, turn off field-relative driving.
            fieldRelative = false;
            swerveSubsystem.drive(
                VisionAprilTag.limelight_rangeSpeedZ_aprilTag(LimelightConstants.llLocalizationNameBackwards),
                VisionAprilTag.horizontalOffsetSpeedXAprilTag(LimelightConstants.llLocalizationNameBackwards),
                    VisionAprilTag.limelight_aimSpeedX_proportional(LimelightConstants.llLocalizationNameBackwards),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);
        } else if (xbox.getRightBumperButton()) { //for forwards april tag allign
            fieldRelative = false;
            swerveSubsystem.drive(
                VisionAprilTag.limelight_rangeSpeedZ_aprilTag(LimelightConstants.llLocalizationNameForwards),
                VisionAprilTag.horizontalOffsetSpeedXAprilTag(LimelightConstants.llLocalizationNameForwards),
                    VisionAprilTag.limelight_aimSpeedX_proportional(LimelightConstants.llLocalizationNameForwards),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);
        } else if (!(xbox.getRightTriggerAxis() > 0.1)) {  //if booster not pressed
            fieldRelative = true;
            swerveSubsystem.drive(
                    -MathUtil.applyDeadband(xbox.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative() && fieldRelative, true);
            //TODO: invert if red so drivers dont have to abitarly reset gyro and field relative everytime
            /*swerveSubsystem.drive(
                    -MathUtil.applyDeadband(invertIfRed(xbox.getLeftY()), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(xbox.getLeftX()), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(xbox.getRightX()), OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative && fieldRelative, true);*/
        } else {
            fieldRelative = true;
            swerveSubsystem.drive(
                    -MathUtil.applyDeadband(xbox.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);
        /*fancy equation that probably would help us to get rid of speed coefficients (ratelimit probably false)
        swerveSubsystem.drive(
            -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftY(), 2), xbox.getLeftY())), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(invertIfRed(Math.copySign(Math.pow(xbox.getLeftX(), 2), xbox.getLeftX())), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(invertIfRed(xbox.getRightX()), OIConstants.kDriveDeadband),
            true, true);*/
        }

        if (xbox.getPOV() == Buttons.POV_RIGHT){
            VisionAprilTag.offsetRight(Constants.LimelightConstants.llLocalizationNameForwards);
        }
        if (xbox.getPOV() == Buttons.POV_LEFT){
            VisionAprilTag.offsetRight(Constants.LimelightConstants.llLocalizationNameForwards);
        }
        if (xbox.getPOV() == Buttons.POV_UP){
            VisionAprilTag.offsetRight(Constants.LimelightConstants.llLocalizationNameForwards);
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
                return -1;
            }
        }
        return 1;
    }


}