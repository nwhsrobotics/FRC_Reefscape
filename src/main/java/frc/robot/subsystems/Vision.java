// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    Logger.recordOutput("ll.straightLineDist", VisionGamePiece.straightLineZDistance("limelight"));
    Logger.recordOutput("ll.verticalDist", VisionGamePiece.verticalYOffsetDistance("limelight"));
    Logger.recordOutput("ll.horizontalDist", VisionGamePiece.horizontalOffestXDistance("limelight"));
    Logger.recordOutput("ll.3dHypotnuese", VisionGamePiece.full3DDistance("limelight"));
    Logger.recordOutput("ll.2dHypotnuese", VisionGamePiece.hypotenuseDistanceXandZ("limelight"));
    Logger.recordOutput("ll.detect", LimelightHelpers.getTY(""));
    //1, 1, Rotation2d.fromDegrees(60)
    Logger.recordOutput("ll.target", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").toString());
    Logger.recordOutput("ll.targetX", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getX());
    Logger.recordOutput("ll.targetY", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getY());
    Logger.recordOutput("ll.targetDegrees", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getRotation().getDegrees());
    Logger.recordOutput("ll.targetRadians", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getRotation().getRadians());
    Logger.recordOutput("ll.targetRotation", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getRotation().getRotations());
    Logger.recordOutput("ll.getOriginDistance", VisionGamePiece.transformTargetLocation(new Pose2d(), "limelight").getTranslation().getNorm());



    Logger.recordOutput("lla.straightLineDist", VisionAprilTag.straightLineZDistance("limelight"));
    Logger.recordOutput("lla.verticalDist", VisionAprilTag.verticalYOffsetDistance("limelight"));
    Logger.recordOutput("lla.horizontalDist", VisionAprilTag.horizontalOffsetXDistance("limelight"));
    Logger.recordOutput("lla.3dHypotnuese", VisionAprilTag.hypotenuseDistanceXandZ("limelight"));
    Logger.recordOutput("lla.detect", LimelightHelpers.getTY(""));
    //1, 1, Rotation2d.fromDegrees(60)
    Logger.recordOutput("lla.target", VisionAprilTag.transformTargetLocation(new Pose2d(), "limelight").toString());
    Logger.recordOutput("lla.targetX", VisionAprilTag.transformTargetLocation(new Pose2d(), "limelight").getX());
    Logger.recordOutput("lla.targetY", VisionAprilTag.transformTargetLocation(new Pose2d(), "limelight").getY());
    Logger.recordOutput("lla.targetDegrees", VisionAprilTag.transformTargetLocation(new Pose2d(), "limelight").getRotation().getDegrees());
    Logger.recordOutput("lla.targetRadians", VisionAprilTag.transformTargetLocation(new Pose2d(), "limelight").getRotation().getRadians());
    Logger.recordOutput("lla.targetRotation", VisionAprilTag.transformTargetLocation(new Pose2d(), "limelight").getRotation().getRotations());
    Logger.recordOutput("lla.getOriginDistance", VisionAprilTag.transformTargetLocation(new Pose2d(), "limelight").getTranslation().getNorm());
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
    
      //This method takes in blue alliance april tags and checks if it is the coral station
      public boolean isBlueAllianceCoralStation(int id) {
        if (id == 12 || id == 13)  {
            return true;
        }
        return false;
      }
      // This method takes in blue alliance april tags and checks if it is the processor
        public boolean isBlueAllianceProcessor(int id) {
            if (id == 4) {
                return true;
            }
            return false;
        }

        //This method takes in blue alliance april tags and checks if it is the reef
        public boolean isBlueAllianceReef(int id) {
            if (id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22) {
                return true;
            }
            return false;
        }
        // This method takes in blue alliance april tags and checks if it is the barge
        public boolean isBlueAllianceBarge(int id) {
            if (id == 4 || id == 14) {
                return true;
            }
            return false;
        
        }
         


// this method takes in a parameter of the april tag and checks if it is at the coral station

public boolean isRedAllianceCoralStation(int id){
    if(id == 1 || id == 2){
        return true;
    }
    return false;

}

// this method takes in a parameter of the april tag and checks if it is at the processor

public boolean isRedAllianceProcessor(int id){
    if(id == 3){
        return true;
    }
    return false;

}

// this method takes in a parameter of the april tag and checks if it is at the reef

public boolean isRedAllianceReef(int id){
    if(id == 6 || id == 7 || id == 8 || id == 9 || id == 10 || id == 11){
        return true;
    }
    return false;

}

// this method takes in a parameter of the april tag and checks if it is at the barge

public boolean isRedAllianceBarge(int id){
    if(id == 5){
        return true;
    }
    return false;

}

// adjusts the robot position based on april tag position and a preset offset
public Pose2d transformPosition (Pose2d originalPos,double offsetDistance){
    
    double givenX = originalPos.getX();
    double givenY= originalPos.getY();
    double givenRot = originalPos.getRotation().getRadians();
    
    Pose2d transformedPose = new Pose2d(new Translation2d(givenX, givenY),
        originalPos.getRotation().plus(originalPos.getRotation()));
    
    double adjustedX = givenX * offsetDistance*Math.cos(givenRot);
    double adjustedY = givenY * offsetDistance*Math.sin(givenRot);

    Pose2d endResult = new Pose2d(adjustedX,adjustedY,Rotation2d.fromRadians(givenRot));

    return endResult;
    
}
}