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
                    VisionAprilTag.limelight_rangeSpeedZ_proportional(LimelightConstants.llLocalizationNameForwards),
                    0,
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
            swerveSubsystem.drive(
                    -MathUtil.applyDeadband(invertIfRed(xbox.getLeftY()), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(xbox.getLeftX()), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(invertIfRed(xbox.getRightX()), OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative && fieldRelative, true);
        } else {
            fieldRelative = true;
            swerveSubsystem.drive(
                    -MathUtil.applyDeadband(xbox.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband),
                    swerveSubsystem.isFieldRelative() && fieldRelative, false);
        }
        //fancy equation probably would us to get rid of speed coefficients
        swerveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> swerveSubsystem.drive(
                -MathUtil.applyDeadband(Math.copySign(Math.pow(xbox.getRawAxis(1), 3), xbox.getRawAxis(1) + (0.25 * xbox.getRawAxis(1))), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.copySign(Math.pow(xbox.getRawAxis(0), 3), xbox.getRawAxis(0) + (0.25 * xbox.getRawAxis(0))), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(xbox.getRawAxis(4), OIConstants.kDriveDeadband),
                true, true),
                swerveSubsystem));
        if (swerveSubsystem.autonavigator.isEnabled()) {
            if (MathUtil.applyDeadband(xbox.getLeftX(), OIConstants.kDriveDeadband) != 0 || 
                MathUtil.applyDeadband(xbox.getLeftY(), OIConstants.kDriveDeadband) != 0 || 
                MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband) != 0 ||
                MathUtil.applyDeadband(xbox.getRightY(), OIConstants.kDriveDeadband) != 0) {
                swerveSubsystem.autonavigator.pauseNavigation();
            } else {
                swerveSubsystem.autonavigator.resumeNavigation();
            }
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