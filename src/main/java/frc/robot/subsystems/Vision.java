// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    Logger.recordOutput("ll.straightLineDist", VisionGamePiece.hypotenuseDistanceXandZ("limelight"));
    Logger.recordOutput("ll.verticalDist", VisionGamePiece.straightLineZDistance("limelight"));
    Logger.recordOutput("ll.horizontalDist", VisionGamePiece.horizontalOffestXDistance("limelight"));
    Logger.recordOutput("ll.3dHypotnuese", VisionGamePiece.full3DDistance("limelight"));
    Logger.recordOutput("ll.detect", LimelightHelpers.getTY(""));
    //1, 1, Rotation2d.fromDegrees(60)
    Logger.recordOutput("ll.target", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").toString());
    Logger.recordOutput("ll.targetX", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getX());
    Logger.recordOutput("ll.targetY", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getY());
    Logger.recordOutput("ll.targetDegrees", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getRotation().getDegrees());
    Logger.recordOutput("ll.targetRadians", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getRotation().getRadians());
    Logger.recordOutput("ll.targetRotation", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getRotation().getRotations());
    Logger.recordOutput("ll.getOriginDistance", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getTranslation().getNorm());

            /* 
        String llname = LimelightConstants.llObjectDetectionName; 
        Vision.visionTargetLocation = Vision.transformTargetLocation(robotContainer.swerveSubsystem.getPose(), llname); 
        HashSet<Integer> tagsFound = new HashSet<>();
        for (int i = 0; i < LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.llLocalizationName).tagCount; i++) {
            tagsFound.add(LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.llLocalizationName).rawFiducials[i].id);
        }
        Vision.tagIds.removeIf(tagId -> !tagsFound.contains(tagId));
        tagsFound.forEach(tagId -> {
            if(!Vision.tagIds.contains(tagId)){
                Vision.tagIds.add(tagId);
            }
        });
        //Logger.recordOutput(llname + ".pipelineIndex", LimelightHelpers.getCurrentPipelineIndex(llname));
        //Logger.recordOutput(llname + ".pipelineName", Vision.getPipelineName(llname));
        //Logger.recordOutput(llname + ".objectDetected", LimelightHelpers.getTV(llname));
        */
  }
}
