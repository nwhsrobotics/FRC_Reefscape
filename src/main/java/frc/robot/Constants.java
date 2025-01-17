package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public final class Constants {
    public static final class CANAssignments {
        //TODO: This year, lets organise CAN IDs, as a resault, we def need to re flash motor controllors 
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 6;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 10;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 3;

        public static final int FRONT_LEFT_STEER_MOTOR_ID = 8;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 11;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 4;

        public static final int FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID = 22;
        public static final int BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID = 20;
        public static final int FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 21;
        public static final int BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 23;


        public static final int PDU_ID = 24;

        /**
         * Check for duplicate CAN assignments,
         * declared under the class this method is defined in.
         * <p>
         * If an assignment cannot be loaded,
         * or a duplicate assignment is found,
         * a message will be printed in the console.
         *
         * @return - true if duplicate assignment is found, otherwise false.
         */
        public static boolean checkAssignments() {
            Field[] constants = CANAssignments.class.getFields();
            HashMap<Integer, String> tracker = new HashMap<>();
            boolean dupeFound = false;

            for (Field field : constants) {
                field.setAccessible(true);

                int workingId;
                try {
                    workingId = field.getInt(CANAssignments.class);
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    System.out.println("Achtung! Checking CAN assignment for " + field.getName() + " failed!");
                    continue;
                }

                if (tracker.put(workingId, field.getName()) != null) {  // this also adds the field to the tracker.
                    System.out.println("Fehler! Duplicate CAN assignment on " +
                            workingId +
                            " for " +
                            field.getName() +
                            " already used by " +
                            tracker.get(workingId) +
                            "!");

                    dupeFound = true;
                }
            }

            Logger.recordOutput("canassignmentsok", !dupeFound);

            return dupeFound;
        }
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.10033; // set up for MK4(i)
        public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // (set up for MK4(i) L2)
        public static final double kTurningMotorGearRatio = (15.0 / 32.0) * (10.0 / 60.0); // (set up for MK4 L2)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = .5; // P constant for turning
        //public static final double kPTolerance = 2.5 * (Math.PI/180);
        public static final double kITurning = 0.;
    }

    public static final class DriveConstants {
        // left-to-right distance between the drivetrain wheels, should be measured from center to center AND IN METERS
        public static final double kTrackWidth = 0.597;
        // front-back distance between drivetrain wheels, should be measured from center to center AND IN METERS 
        public static final double kWheelBase = 0.546;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //back right

        public static final double kDirectionSlewRate = 0.9; // radians per second
        public static final double kMagnitudeSlewRate = 1.35; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 1.5; // percent per second (1 = 100%)

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //FOR ALL OFFSETS: turn wheels until they become straight, replace with the value of encoders
        //THE BLACK GEAR SHOULD BE ON THE OUTSIDE FOR ALL WHEELS, regardless of side
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -1.064582666792635;//2.66 + Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.202796411403781;//5.24 - Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.064738140494073;//0.61 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -2.066272121281959;//5.20 - Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI; // set up for NEOs to drive
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0); //adapted from SDS
    }



    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
        public static final double kPXController = 5;
        public static final double kPThetaController = 5;

        public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
                new com.pathplanner.lib.config.PIDConstants(AutoConstants.kPXController, 0, 0), // Translation constants
                new com.pathplanner.lib.config.PIDConstants(AutoConstants.kPThetaController, 0, 0), // Rotation constants
                //DriveConstants.kPhysicalMaxSpeedMetersPerSecond// Drive base radius (distance from center to furthest module)
                0.02
        );

        public static final PathConstraints kPathfindingConstraints = new PathConstraints(
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.8, AutoConstants.kMaxAccelerationMetersPerSecondSquared * 0.75,
                AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final double scaleFactor = 0.6;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * scaleFactor;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond * scaleFactor;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class Positions {

        public static final Pose2d LEFT_GROUND_START_PIECES = new Pose2d(1.219, 5.855, Rotation2d.fromDegrees(0)); //TODO: fix rotation 
        public static final Pose2d MID_GROUND_START_PIECES = new Pose2d(1.219, 4.022, Rotation2d.fromDegrees(0)); //TODO: fix rotation 
        public static final Pose2d RIGHT_GROUND_START_PIECES = new Pose2d(1.219, 2.189, Rotation2d.fromDegrees(0)); //TODO: fix rotation 

        public static final Pose2d RIGHT_CAGE = new Pose2d(8.777, 7.267, Rotation2d.fromDegrees(0)); //TODO: fix rotation 
        public static final Pose2d RIGHT_MID = new Pose2d(8.777, 6.173, Rotation2d.fromDegrees(0)); //TODO: fix rotation 
        public static final Pose2d RIGHT_LEFT = new Pose2d(8.777, 5.075, Rotation2d.fromDegrees(0)); //TODO: fix rotation 

        public static final Pose2d FRONT_LEFT_REEF = new Pose2d(3.645, 5.454, Rotation2d.fromDegrees(-60)); 
        public static final Pose2d FRONT_MID_REEF = new Pose2d(2.837, 4.029, Rotation2d.fromDegrees(0)); 
        public static final Pose2d FRONT_RIGHT_REEF = new Pose2d(3.614, 2.613, Rotation2d.fromDegrees(60)); 
        public static final Pose2d BACK_LEFT_REEF = new Pose2d(5.352, 5.444, Rotation2d.fromDegrees(-120)); 
        public static final Pose2d BACK_MID_REEF = new Pose2d(6.152, 4.026, Rotation2d.fromDegrees(180)); 
        public static final Pose2d BACK_RIGHT_REEF = new Pose2d(5.310, 2.582, Rotation2d.fromDegrees(120)); 

        public static final Pose2d SOURCE_LEFT = new Pose2d(1.201, 7.018, Rotation2d.fromDegrees(125)); //this is for the center of the source 
        public static final Pose2d SOURCE_RIGHT = new Pose2d(1.149, 1.043, Rotation2d.fromDegrees(-125)); //this is for the center of the source 
        
        public static final List<Pose2d> allNotes = new ArrayList<Pose2d>(); //temp fix 
    }

    public static final class LimelightConstants {
        public static final double mountHeight = 1.32; //in meters
        public static final double mountAngle = 1.5; //in degrees with straight being 0 up being 90 and down being -90
        public static final double horizontalOffset = -0.18; //in meters, this offset is how far left or right LL3 is mounted from center (negative is left, positive right)
        //this might not be needed but doesn't hurt us
        public static final double distanceFromCenter = 0.3; //in meters, straight distance to the camera from middle
        public static final double hypotenuseDistance = Math.hypot(horizontalOffset, distanceFromCenter); // actual distance in 2d from middle
        public static final double thethaFromCenter = -32; //this might be needed for angle offset
        public static String llObjectDetectionName = "limelight";
        public static String llLocalizationName = "limelightLoc";
        public static double targetHeight;
    }

    public enum RuntimeEnvironment {
        /**
         * Running on physical robot.
         */
        REAL,
        /**
         * Running on simulated robot.
         */
        SIMULATION,
        /**
         * Replaying robot from log file.
         */
        REPLAY
    }

    public static final ModuleType PDU_TYPE = ModuleType.kRev;

    public static final class LoggerConstants {
        public static final RuntimeEnvironment MODE = RuntimeEnvironment.REAL;
        public static final String RUNNING_UNDER = "2025.q1";

        // SET TO FALSE IF WE'RE RUNNING OUT OF BANDWIDTH.
        public static final boolean SILENT_NT4 = false;
    }
}
