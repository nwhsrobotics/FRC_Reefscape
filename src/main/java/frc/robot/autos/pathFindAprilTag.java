// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTags;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionAprilTag;
import frc.robot.subsystems.VisionSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class pathFindAprilTag extends Command {
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final int targetAprilTagId;
  private Command pathFind;

  /** Creates a new pathFindAprilTag. */
  public pathFindAprilTag(int targetAprilTagId, SwerveSubsystem swerve, VisionSubsystem vision, String targetLocation) {
    Pose2d object = new Pose2d();
    //object = AprilTags.aprilTags.get(targetAprilTagId-1);
    object = VisionAprilTag.transformTargetLocation(swerve.getPose(), "limelight");
    this.swerve = swerve;
    this.targetAprilTagId = targetAprilTagId;
    this.vision = vision;
    addRequirements(swerve);
    object = vision.transformPosition(object, 0.5);
    if(targetLocation.indexOf("B") != -1){
      Pose2d temp = object;
      object = vision.scootLeft(temp, 0.5);
    }
    if(targetLocation.indexOf("A") != -1){
      Pose2d temp = object;
      object = vision.scootRight(temp, 0.5);
    }
    pathFind = swerve.pathfindToPosition(object);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathFind.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
