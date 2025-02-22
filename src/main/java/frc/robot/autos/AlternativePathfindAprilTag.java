// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionAprilTag;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlternativePathfindAprilTag extends Command {
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final int targetAprilTagId;
  private Command pathFind;



  /** Creates a new AlternativePathfindAprilTag. */
  public AlternativePathfindAprilTag(int targetAprilTagId, SwerveSubsystem swerve, VisionSubsystem vision, String targetLocation) {
    this.swerve=swerve;
    this.vision=vision;
    this.targetAprilTagId=targetAprilTagId;

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
    if(Math.abs(VisionAprilTag.horizontalOffsetXAprilTag("llf")) >= (Math.PI/20)){
    
    swerve.drive(
                0*VisionAprilTag.limelight_rangeZ_aprilTag(LimelightConstants.llLocalizationNameForwards),
                0.278*VisionAprilTag.horizontalOffsetXAprilTag(LimelightConstants.llLocalizationNameForwards),
                VisionAprilTag.limelight_aimX_proportional(LimelightConstants.llLocalizationNameForwards),
                swerve.isFieldRelative() && fieldRelative, false);
    }
    else{
      swerve.drive(
        0.314*VisionAprilTag.limelight_rangeZ_aprilTag(LimelightConstants.llLocalizationNameForwards),
        0*VisionAprilTag.horizontalOffsetXAprilTag(LimelightConstants.llLocalizationNameForwards),
        0*VisionAprilTag.limelight_aimX_proportional(LimelightConstants.llLocalizationNameForwards),
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
  { 
    if (Math.abs(VisionAprilTag.horizontalOffsetXAprilTag("llf")) <= (Math.PI/20) && Math.abs(VisionAprilTag.limelight_rangeZ_aprilTag("llf")) <= (4*(Math.PI)/75))
  {
    return true;
  }
    else{
      return false;
    }
  }
}
