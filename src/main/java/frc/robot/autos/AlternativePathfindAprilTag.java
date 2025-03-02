// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionAprilTag;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlternativePathfindAprilTag extends Command {
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final int targetAprilTagId;
  private Command pathFind;



  // A-> Right, B-> Left, S-> Center (always end with center)
  /** Creates a new AlternativePathfindAprilTag. */
  public AlternativePathfindAprilTag(int targetAprilTagId, SwerveSubsystem swerve, VisionSubsystem vision, String targetLocation) {
    this.swerve=swerve;
    this.vision=vision;
    addRequirements(swerve);
    this.targetAprilTagId=targetAprilTagId;
    if(targetLocation.indexOf("B") != -1){
      VisionAprilTag.offsetLeft(vision.getLimelightName());
    }
    if(targetLocation.indexOf("A") != -1){
      VisionAprilTag.offsetRight(vision.getLimelightName());

    }


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean fieldRelative=false;
    if(Math.abs(VisionAprilTag.horizontalOffsetXAprilTag(vision.getLimelightName()))>= (Math.PI/20)){
    
    swerve.drive(
                VisionAprilTag.limelight_rangeSpeedZ_aprilTag(vision.getLimelightName()),
                VisionAprilTag.horizontalOffsetSpeedXAprilTag(vision.getLimelightName()),
                VisionAprilTag.limelight_aimSpeedX_proportional(vision.getLimelightName()),
                swerve.isFieldRelative() && fieldRelative, false);
    }
    else{
      swerve.drive(
        VisionAprilTag.limelight_rangeSpeedZ_aprilTag(vision.getLimelightName()),
        VisionAprilTag.horizontalOffsetSpeedXAprilTag(vision.getLimelightName()),
        VisionAprilTag.limelight_aimSpeedX_proportional(vision.getLimelightName()),
        swerve.isFieldRelative() && fieldRelative, false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  { // right now the magic numbers are 0.158m and 0.168m (this is still big gap, try to get them as low as 0.04m and 0.08m or less)
    if (Math.abs(VisionAprilTag.horizontalOffsetXAprilTag(vision.getLimelightName())) <= (Math.PI/20) && 
       Math.abs(VisionAprilTag.straightLineZAprilTag(vision.getLimelightName())) <= ((4*(Math.PI))/75))
  {
    return true;
  }
    else{
      return false;
    }
  }
}
