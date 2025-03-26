package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.AprilTagOffsets;
import frc.robot.Constants.TagOffset;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class VisionAprilTag {

    public static Pose2d visionTargetLocation = new Pose2d();
    public static double tagDist;
    public static ArrayList<Integer> tagIds = new ArrayList<>();

    // maybe 100?
    private static double holdDuration = 200;
    private static long timeDetected = 0;

    private static LimelightResults stableResults = null;

    private static double scootOffsetX = 0.0;


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
     *
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
     *
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
     *
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
     *
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
     *
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
     *
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
            double llToFrontOfRobot = 0.5;
            /*
            var fiducials = llr.getRawFiducials();
            if (fiducials.length > 0) {
                double distToRobot = fiducials[0].distToRobot;
            }
            */
            // Use Z-axis distance to the target
            return llr.targets_Fiducials[0].getTargetPose_CameraSpace().getTranslation().getZ() - AprilTagOffsets.getOffset((int)llr.targets_Fiducials[0].fiducialID).relative - llToFrontOfRobot;
        }
        return 0.0;
    }

    public static double straightLineZAprilTag(String limelightName) {
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {
            double llToFrontOfRobot = 0.5;
            // Use Z-axis distance to the target
            return llr.targets_Fiducials[0].getRobotPose_TargetSpace().getTranslation().getZ() - AprilTagOffsets.getOffset((int)llr.targets_Fiducials[0].fiducialID).relative - llToFrontOfRobot;
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
            return llr.targets_Fiducials[0].getRobotPose_TargetSpace().getTranslation().getX() + scootOffsetX;
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
            // experimental distance adaptive stabilizer (make it greater than linear if needed) 
            int idealDist = 2;
            double distance = results.targets_Fiducials[0].getRobotPose_TargetSpace2D().getTranslation().getNorm();
            holdDuration = Math.max((distance / idealDist) * 100, 50);
        } else {
            long time = now - timeDetected;
            if (time < holdDuration) {
            } else {
                stableResults = null;
            }
        }
        return stableResults;
    }


    /**
     * Transforms the target's location based on Limelight's data.
     *
     * @param pos           The current position of the swerve.
     * @param limelightName Name of the Limelight.
     * @return The transformed position of the target.
     */
    public static Pose2d transformTargetLocation(Pose2d pos, String limelightName) {
        // we could just use transform, but come on that's not fun like the normal math it does all the trig under the hood for us :(
        LimelightResults llr = isValid(limelightName);
        if (llr != null) {

            Pose2d targetInRobotCoords = llr.targets_Fiducials[0].getTargetPose_RobotSpace2D();
            double robotFrontDist = 0.457;

            Translation2d adjustedTranslation = new Translation2d(targetInRobotCoords.getX() - robotFrontDist, -(targetInRobotCoords.getY() + scootOffsetX));


            Pose2d targetOnField = pos.transformBy(
                    new Transform2d(
                            //targetInRobotCoords.getTranslation(),
                            adjustedTranslation,
                            //get tx instead?
                            //LimelightHelpers.getTX(limelightName)
                            //targetInRobotCoords.getRotation()
                            Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName))
                    )
            );

            Logger.recordOutput("limelight.objectPos", targetOnField);
            return targetOnField;
        }

        return pos;
    }

    // public static Pose2d transformTargetLocation(Pose2d pos, String limelightName) {
    //     LimelightResults llr = isValid(limelightName);
    //     if (llr != null) {
    //         Pose2d rawTarget = llr.targets_Fiducials[0].getTargetPose_RobotSpace2D();

    //         Translation2d rawTranslation = rawTarget.getTranslation();
    //         Translation2d adjustedTranslation = new Translation2d(rawTranslation.getX(), -rawTranslation.getY());

    //         Rotation2d rawRotation = rawTarget.getRotation();
    //         Rotation2d adjustedRotation = rawRotation.unaryMinus(); 

    //         Pose2d adjustedTarget = new Pose2d(adjustedTranslation, adjustedRotation);

    //         Pose2d targetOnField = pos.transformBy(
    //             new Transform2d(
    //                 adjustedTarget.getTranslation(),
    //                 adjustedTarget.getRotation()
    //             )
    //         );

    //         Logger.recordOutput("limelight.objectPos", targetOnField);
    //         return targetOnField;
    //     }
    //     return pos;
    // }


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
    static boolean isAprilTagPipeline(String limelightName) {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName) == 0;
    }

    public static void offsetRight(String limelightName) {
        scootOffsetX = 0.2051;
        LimelightHelpers.setFiducial3DOffset(limelightName, 0, scootOffsetX, 0);
        //LimelightHelpers.setFiducial3DOffset(limelightName, scootOffsetX, 0, 0);
    }

    public static void offsetLeft(String limelightName) {
        scootOffsetX = -0.1251;
        LimelightHelpers.setFiducial3DOffset(limelightName, 0, scootOffsetX, 0);
        //LimelightHelpers.setFiducial3DOffset(limelightName, scootOffsetX, 0, 0);
    }

    public static void offsetCenter(String limelightName) {
        scootOffsetX = 0.0;
        //LimelightHelpers.setFiducial3DOffset(limelightName, 0, 0.0, 0);
        LimelightHelpers.setFiducial3DOffset(limelightName, scootOffsetX, 0, 0);
    }
}
