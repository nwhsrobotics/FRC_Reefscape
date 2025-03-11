package frc.robot.autos;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionNavPID extends Command {
  private final SwerveSubsystem swerve;
  private final VisionSubsystem limelight;
  private final boolean alignLeft;
  
  private final HolonomicDriveController driveController;

  private Pose2d targetPose;

  public VisionNavPID(SwerveSubsystem swerve, VisionSubsystem limelight, boolean alignLeft) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.alignLeft = alignLeft;
    
    PIDController xController = new PIDController(0.1, 0, 0);
    PIDController yController = new PIDController(0.1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(0.1, 0, 0, new Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    driveController = new HolonomicDriveController(xController, yController, thetaController);
    addRequirements(swerve);
  }
  
  @Override
  public void initialize() {
    Pose2d currentPose = swerve.getPose();
    if (alignLeft) {
      targetPose = limelight.leftReef(currentPose);
    } else {
      targetPose = limelight.rightReef(currentPose);
    }
  
  }
  
  @Override
  public void execute() {
      Pose2d currentPose = swerve.getPose();
      ChassisSpeeds speeds = driveController.calculate(currentPose, targetPose, 1, targetPose.getRotation());
      swerve.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true, false);
  }
  
  
  @Override
  public boolean isFinished() {
    Pose2d currentPose = swerve.getPose();
    double translationError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double rotationError = currentPose.getRotation().minus(targetPose.getRotation()).getRadians();
    return (translationError < 0.1) &&(Math.abs(rotationError) < 0.1);
  }
  
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }
  

}
