package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.Elastic;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.LimelightHelpers.LimelightResults;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class VisionAprilTag {

    public static Pose2d visionTargetLocation = new Pose2d();
    public static double tagDist;
    public static ArrayList<Integer> tagIds = new ArrayList<>();

    private static final long DETECTION_HOLD = 100; 
    private static long timeDetected = 0;

    private static LimelightResults stableResults = null;

    /*
     * fiducial.getRobotPoseTargetSpace(); // Robot pose relative it the AprilTag Coordinate System (Most Useful) where April tag is 0, 0
        fiducial.getCameraPoseTargetSpace(); // Camera pose relative to the AprilTag (useful) (never use camera tbh)
        fiducial.getRobotPoseFieldSpace(); // Robot pose in the field coordinate system based on this tag alone (useful) (we already use this to correct swerve odometry) 0,0 at field corner
        fiducial.getTargetPoseCameraSpace(); // AprilTag pose in the camera's coordinate system (not very useful) (never use camera tbh)
        fiducial.getTargetPoseRobotSpace(); // AprilTag pose in the robot's coordinate system (not very useful) (this is like VisionGamePiece.java but for AprilTags)
     */
    /*
     *         if (LimelightHelpers.getTV(limelightName)) {
            // Use Z-axis distance to the target
            LimelightResults ll = LimelightHelpers.getLatestResults(limelightName);
            ll.valid
            Elastic.Notification notification = new Elastic.Notification(NotificationLevel.INFO, "Detected Notification", "Found April Tag");
            Commands.waitUntil(() -> ll.targets_Fiducials.length > 0)
            .andThen(new InstantCommand(() -> Elastic.sendNotification(notification)));
     */


    /**
     * Distance times Magic number = Speed.
     * <p>
     * Magic number is the fine tuned speed limiter value
     * <p>
     * Imagine if distance is 8 meters, if we don't multiply by 
     * magic number (speed limiter) it will output 8 meters/second!
     * <p>
     * If distance is 8, and magic number is 0.25,
     * the output speed value will be a reasonable 2 meters/second
     * @return The speed
     */
    public static double limelight_aimSpeedX_proportional(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            double kP = 0.035;
            double horizontalError = LimelightHelpers.getTX(limelightName);
            return -kP * horizontalError;
        }
        return 0.0;
    }
    
    /**
     * Distance times Magic number = Speed.
     * <p>
     * Magic number is the fine tuned speed limiter value
     * <p>
     * Imagine if distance is 8 meters, if we don't multiply by 
     * magic number (speed limiter) it will output 8 meters/second!
     * <p>
     * If distance is 8, and magic number is 0.25,
     * the output speed value will be reasonable 2 meters/second
     * @return The speed
     */
    public static double limelight_aimSpeedX_aprilTag(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            double kP = 0.035;
            double horizontalError = llr.targets_Fiducials[0].getRobotPose_TargetSpace2D().getRotation().getDegrees();
            return -kP * horizontalError;
        }
        return 0.0;
    }
    
    /**
     * Distance times Magic number = Speed.
     * <p>
     * Magic number is the fine tuned speed limiter value
     * <p>
     * Imagine if distance is 8 meters, if we don't multiply by 
     * magic number (speed limiter) it will output 8 meters/second!
     * <p>
     * If distance is 8, and magic number is 0.25,
     * the output speed value will be reasonable 2 meters/second
     * @return The speed
     */
    public static double limelight_rangeSpeedZ_proportional(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return straightLineZDistance(limelightName) * 0.278;
        }
        return 0.0;
    }

    /**
     * Distance times Magic number = Speed.
     * <p>
     * Magic number is the fine tuned speed limiter value
     * <p>
     * Imagine if distance is 8 meters, if we don't multiply by 
     * magic number (speed limiter) it will output 8 meters/second!
     * <p>
     * If distance is 8, and magic number is 0.25,
     * the output speed value will be reasonable 2 meters/second
     * @return The speed
     */
    public static double limelight_rangeSpeedZ_aprilTag(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return -straightLineZAprilTag(limelightName) * 0.278;
        }
        return 0.0;
    }

    /**
     * Distance times Magic number = Speed.
     * <p>
     * Magic number is the fine tuned speed limiter value
     * <p>
     * Imagine if distance is 8 meters, if we don't multiply by 
     * magic number (speed limiter) it will output 8 meters/second!
     * <p>
     * If distance is 8, and magic number is 0.25,
     * the output speed value will be reasonable 2 meters/second
     * @return The speed
     */
    public static double horizontalOffsetSpeedX(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return horizontalOffsetXDistance(limelightName) * 0.278;
        }
        return 0.0;
    }
    
    /**
     * Distance times Magic number = Speed.
     * <p>
     * Magic number is the fine tuned speed limiter value
     * <p>
     * Imagine if distance is 8 meters, if we don't multiply by 
     * magic number (speed limiter) it will output 8 meters/second!
     * <p>
     * If distance is 8, and magic number is 0.25,
     * the output speed value will be reasonable 2 meters/second
     * @return The speed
     */
    public static double horizontalOffsetSpeedXAprilTag(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return horizontalOffsetXAprilTag(limelightName) * 0.278;
        }
        return 0.0;
    }
    
    public static double straightLineZDistance(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            /*
            var fiducials = llr.getRawFiducials();
            if (fiducials.length > 0) {
                double distToRobot = fiducials[0].distToRobot;
            }
            */
            // Use Z-axis distance to the target
            return llr.targets_Fiducials[0].getTargetPose_CameraSpace().getTranslation().getZ();
        }
        return 0.0;
    }
    
    public static double straightLineZAprilTag(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            // Use Z-axis distance to the target
            return llr.targets_Fiducials[0].getRobotPose_TargetSpace().getTranslation().getZ();
        }
        return 0.0;
    }
    
    public static double hypotenuseDistanceXandZ(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return llr.targets_Fiducials[0].getTargetPose_RobotSpace2D().getTranslation().getNorm();
        }
        return 0.0;
    }
    
    public static double hypotenuseDistanceXandZAprilTag(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return llr.targets_Fiducials[0].getRobotPose_TargetSpace2D().getTranslation().getNorm();
        }
        return 0.0;
    }
    
    public static double full3DDistance(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return llr.targets_Fiducials[0].getTargetPose_RobotSpace().getTranslation().getNorm();
        }
        return 0.0;
    }
    
    public static double full3DDistanceAprilTag(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return llr.targets_Fiducials[0].getRobotPose_TargetSpace().getTranslation().getNorm();
        }
        return 0.0;
    }
    
    public static double horizontalOffsetXDistance(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return llr.targets_Fiducials[0].getTargetPose_RobotSpace().getTranslation().getX();
        }
        return 0.0;
    }
    
    public static double horizontalOffsetXAprilTag(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return llr.targets_Fiducials[0].getRobotPose_TargetSpace().getTranslation().getX();
        }
        return 0.0;
    }
    
    public static double verticalYOffsetDistance(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return llr.targets_Fiducials[0].getTargetPose_RobotSpace().getTranslation().getY();
        }
        return 0.0;
    }
    
    public static double verticalYOffsetAprilTag(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            return llr.targets_Fiducials[0].getRobotPose_TargetSpace().getTranslation().getY();
        }
        return 0.0;
    }
    
    /**
     * Checks if the latest Limelight results are valid. 
     * Returns the results if valid, or null otherwise.
     */
    public static LimelightResults isValid(String limelightName) {
        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

        long now = System.currentTimeMillis();

        if (results != null && results.valid && LimelightHelpers.getTV(limelightName) && results.targets_Fiducials.length > 0) {
            timeDetected = now;
            stableResults = results;
        } else {
            long time = now - timeDetected;
            if (time < DETECTION_HOLD) {
            } else {
                stableResults = null;
            }
        }
        return stableResults;
    }
    

    /**
     * Transforms the target's location based on Limelight's data.
     *
     * @param pos           The current position of the target.
     * @param limelightName Name of the Limelight.
     * @return The transformed position of the target.
     */
    public static Pose2d transformTargetLocation(Pose2d pos, String limelightName) {
        // we could just use transform, but come on thats not fun like the normal math it does all the trig under the hood for us :(
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {

            Pose2d cameraPoseOnRobot = new Pose2d(
                new Translation2d(
                    LimelightConstants.distanceFromCenterForwards,  
                    LimelightConstants.horizontalOffsetForwards      
                ),
                new Rotation2d(0) 
            );

            Pose2d targetInCameraCoords = llr.targets_Fiducials[0].getTargetPose_RobotSpace2D();

            Pose2d targetInRobotCoords = cameraPoseOnRobot.transformBy(
                new Transform2d(
                    targetInCameraCoords.getTranslation(),
                    targetInCameraCoords.getRotation()
                )
            );

            Pose2d targetOnField = pos.transformBy(
                new Transform2d(
                    targetInRobotCoords.getTranslation(),
                    targetInRobotCoords.getRotation()
                )
            );

            Logger.recordOutput("limelight.objectPos", targetOnField);
            return targetOnField;
        }

        return pos;
    }


    /**
     * Switches to the next pipeline.
     */
    public static void nextPipeline(String limelightName) {
        int currentIndex = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        int newIndex = (currentIndex + 1) % 2; // Assuming two pipelines (AprilTag and retro-reflective)
        LimelightHelpers.setPipelineIndex(limelightName, newIndex);
    }

    /**
     * Gets the name of the current pipeline.
     *
     * @return Name of the active pipeline.
     */
    public static String getPipelineName(String limelightName) {
        int currentIndex = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        return currentIndex == 0 ? "AprilTag" : "RetroReflective";
    }

    /**
     * Checks if the AprilTag pipeline is being used.
     *
     * @return True if the AprilTag pipeline is active.
     */
    private static boolean isAprilTagPipeline(String limelightName) {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName) == 0;
    }

    //TODO: Tune this, how far left or right from april tag and distance limiter for isDetecting
    public static void offsetRight(String limelightName){
        LimelightHelpers.setFiducial3DOffset(limelightName, 0, 0.3, 0);
    }

    public static void offsetLeft(String limelightName){
        LimelightHelpers.setFiducial3DOffset(limelightName, 0, -0.3, 0);
    }

    public static void offsetCenter(String limelightName){
        LimelightHelpers.setFiducial3DOffset(limelightName, 0, 0, 0);
    }
}
