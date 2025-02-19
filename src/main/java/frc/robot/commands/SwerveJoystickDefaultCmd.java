package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionAprilTag;
import frc.robot.subsystems.VisionGamePiece;

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
        if (xbox.getLeftBumperButton()) {  //for object detection alligning
            //while using Limelight, turn off field-relative driving.
            fieldRelative = false;
            swerveSubsystem.drive(
                    VisionGamePiece.limelight_rangeZ_proportional(LimelightConstants.llObjectDetectionNameForwards),
                    0,
                    VisionGamePiece.limelight_aimX_proportional(LimelightConstants.llObjectDetectionNameForwards),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);

        } else if (xbox.getRightBumperButton()) { //for april tag allign
            fieldRelative = false;
            swerveSubsystem.drive(
                VisionAprilTag.limelight_rangeZ_aprilTag(LimelightConstants.llLocalizationNameForwards),
                    VisionAprilTag.horizontalOffsetXAprilTag(LimelightConstants.llLocalizationNameForwards),
                    VisionAprilTag.limelight_aimX_proportional(LimelightConstants.llLocalizationNameForwards),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);

        } else if (!(xbox.getRightTriggerAxis() > 0.1)) {  //if booster not pressed
            fieldRelative = true;
            swerveSubsystem.drive(
                    //TODO: its actually kinda funny we do things manually but dont use MathUtil.clamp, MathUtil.deadband, etc. implement MathUtil. instead of doing that please
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
        }
        //fancy equation probably would us to get rid of speed coefficients
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