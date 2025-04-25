package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.LimelightHelpers;


public class VisionGamePiece {
    
    private static Pose2d targetPose = new Pose2d();
    private static Pose2d robotPose = new Pose2d();
    private static double lastSeen = Timer.getFPGATimestamp();
    public static boolean isTargetDetected;
    private static final double TIMEOUT = 0.2;
    private static final double DT = 0.02;
    private static final double TC = 0.1;
    private static final double ALPHA = DT / (DT + TC);
    private static final LinearFilter xFilter = LinearFilter.singlePoleIIR(ALPHA, DT);
    private static final LinearFilter yFilter = LinearFilter.singlePoleIIR(ALPHA, DT);
    private static final LinearFilter yawFilter = LinearFilter.singlePoleIIR(ALPHA, DT);

    public static void updateRobotPose(Pose2d currentPose) {
        robotPose = currentPose;
    }

    public static void update(String limelightName) {
        double now = Timer.getFPGATimestamp();

        if (LimelightHelpers.getTV(limelightName)) {
            lastSeen = now;
            Pose2d raw = computeRawPose(robotPose, limelightName);
            double sx = xFilter.calculate(raw.getX());
            double sy = yFilter.calculate(raw.getY());
            double syaw = yawFilter.calculate(raw.getRotation().getRadians());
            targetPose = new Pose2d(new Translation2d(sx, sy), new Rotation2d(syaw));
            isTargetDetected = true;
        } else if (now - lastSeen > TIMEOUT) {
            xFilter.reset();
            yFilter.reset();
            yawFilter.reset();
            xFilter.calculate(robotPose.getX());
            yFilter.calculate(robotPose.getY());
            yawFilter.calculate(robotPose.getRotation().getRadians());
            targetPose = robotPose;
            isTargetDetected = false;
        }
    }

    public static double getDistance() {
        return robotPose
                .getTranslation()
                .getDistance(targetPose.getTranslation());
    }

    public static double getHorizontalDistance() {
        Translation2d delta = targetPose.getTranslation()
                .minus(robotPose.getTranslation());
        return delta.getX();
    }

    public static double getLateralOffset() {
        Translation2d delta = targetPose.getTranslation()
                .minus(robotPose.getTranslation());
        return delta.getY();
    }

    public static double getStraightLineDistance() {
        Translation2d delta = targetPose.getTranslation()
                .minus(robotPose.getTranslation());
        return delta.getDistance(new Translation2d());
    }

    public static double get3dDistance() {
        double z = (LimelightConstants.targetHeightForwards
                - LimelightConstants.mountHeightForwards);
        return Math.sqrt(getStraightLineDistance() * getStraightLineDistance() + z * z);
    }

    public static double getAngleError() {
        return targetPose
                .relativeTo(robotPose)
                .getRotation()
                .getDegrees();
    }

    public static Pose2d getRelativePose() {
        return targetPose.relativeTo(robotPose);
    }

    public static Pose2d getFieldPose() {
        return targetPose;
    }

    //TODO: targetHeightForwards (meaning gamepiece has to be on a static height)
    private static Pose2d computeRawPose(Pose2d robot, String name) {
        double ty = LimelightHelpers.getTY(name);
        double tx = LimelightHelpers.getTX(name);
        double heightDiff = LimelightConstants.targetHeightForwards
                - LimelightConstants.mountHeightForwards;
        double angle = LimelightConstants.mountAngleForwards + ty;
        double z = heightDiff / Math.tan(Math.toRadians(angle));
        double x = z * Math.tan(Math.toRadians(tx));
        double hyp = Math.hypot(z, x);

        Transform2d cameraOffset = new Transform2d(
                new Translation2d(LimelightConstants.horizontalOffsetForwards, 0),
                new Rotation2d()
        );
        Transform2d visionOffset = new Transform2d(
                new Translation2d(hyp, 0),
                Rotation2d.fromDegrees(tx)
        );
        return robot.transformBy(cameraOffset).transformBy(visionOffset);
    }
}
