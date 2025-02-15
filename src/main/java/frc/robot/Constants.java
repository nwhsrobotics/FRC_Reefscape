package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
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
import java.util.Map;

public final class Constants {
    public static final class CANAssignments {
        // 2024 robot
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

        // 2025 robot
        /*
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 7;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 5;

        public static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 8;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 6;

        public static final int FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID = 9;
        public static final int BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID = 12;
        public static final int FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 10;
        public static final int BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 11;*/

        //ele can IDs
        public static final int LEFT_ELEVATOR_MOTOR_ID = 13;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 14;
        public static final int CLIMB_ABSOLUTE_ENCODER_LEFT_ID = 15;
        public static final int CLIMB_ABSOLUTE_ENCODER_RIGHT_ID = 16;


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
        public static final double kTrackWidth = 0.5715;
        // front-back distance between drivetrain wheels, should be measured from center to center AND IN METERS 
        public static final double kWheelBase = 0.5715;
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
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //FOR ALL OFFSETS: turn wheels until they become straight, replace with the value of encoders
        //THE BLACK GEAR SHOULD BE ON THE OUTSIDE FOR ALL WHEELS, regardless of side

        

        //THESE ONES ARE FOR THE 2025 ROBOT 
        //public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.6212622190936846;//2.66 + Math.PI;
        //public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.679553489033339;//5.24 - Math.PI;
        //public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.103087660191214;//0.61 + Math.PI;
        //public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -2.2534177774040067;//5.20 - Math.PI;


        //2024 ROBOT
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -1.0998642249140047;//2.66 + Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.1475731030398975;//5.24 - Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.0829904362472627;//0.61 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.0737865515199487;//5.20 - Math.PI;
        

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
                new PIDConstants(AutoConstants.kPXController, 0, 0), // Translation constants
                new PIDConstants(AutoConstants.kPThetaController, 0, 0), // Rotation constants
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
        public static final double kDriveDeadband = 0.10;
    }

    public static final class Positions {
        //from driver perspective
        
        public static final Pose2d LEFT_GROUND_START_PIECES = new Pose2d(1.219, 5.855, Rotation2d.fromDegrees(0)); //TODO: fix rotation 
        public static final Pose2d MID_GROUND_START_PIECES = new Pose2d(1.219, 4.022, Rotation2d.fromDegrees(0)); //TODO: fix rotation 
        public static final Pose2d RIGHT_GROUND_START_PIECES = new Pose2d(1.219, 2.189, Rotation2d.fromDegrees(0)); //TODO: fix rotation 

        public static final Pose2d CAGE_A = new Pose2d(8.777, 7.267, Rotation2d.fromDegrees(0)); //TODO: fix rotation 
        public static final Pose2d CAGE_B = new Pose2d(8.777, 6.173, Rotation2d.fromDegrees(0)); //TODO: fix rotation 
        public static final Pose2d CAGE_C = new Pose2d(8.777, 5.075, Rotation2d.fromDegrees(0)); //TODO: fix rotation 

        public static final Pose2d BACK_LEFT_REEF = new Pose2d(3.645, 5.454, Rotation2d.fromDegrees(-60)); // POV_LEFT - Side 6
        public static final Pose2d BACK_MID_REEF = new Pose2d(2.837, 4.029, Rotation2d.fromDegrees(0)); // A button - Side 5
        public static final Pose2d BACK_RIGHT_REEF = new Pose2d(3.614, 2.613, Rotation2d.fromDegrees(60)); // POV_RIGHT - Side 4
        public static final Pose2d FRONT_LEFT_REEF = new Pose2d(5.352, 5.444, Rotation2d.fromDegrees(-120)); // X button - Side 1
        public static final Pose2d FRONT_MID_REEF = new Pose2d(6.152, 4.026, Rotation2d.fromDegrees(180)); // Y button - Side 2
        public static final Pose2d FRONT_RIGHT_REEF = new Pose2d(5.310, 2.582, Rotation2d.fromDegrees(120)); // B button - Side 3

        public static final Pose2d STATION_LEFT = new Pose2d(1.201, 7.018, Rotation2d.fromDegrees(126)); //this is for the center of the source 
        public static final Pose2d STATION_RIGHT = new Pose2d(1.149, 1.043, Rotation2d.fromDegrees(-126)); //this is for the center of the source 
        
        public static final Pose2d PROCESSOR = new Pose2d(1.201, 7.018, Rotation2d.fromDegrees(-90));

        public static final List<Pose2d> allNotes = new ArrayList<Pose2d>(); 
        // get rid of temp fix
    }

    public static final class SysIdConstants {
        //Place holder ratio
        public static final double SYSIDMOTORGEARRATIO = 99;
        public static final double SYSIDSUBSYSTEMDIAMETER = 100;
        //equation needs to be reviewed
        
        public static final double SYSIDENCOCERROT2METER = SYSIDMOTORGEARRATIO * Math.PI * SYSIDSUBSYSTEMDIAMETER;
        public static final double SYSIDENCODERMETERPERSECONDS = SYSIDENCOCERROT2METER/60;

    }
    public static final class ElevatorConstants{
        // Elevator Limits
        //units are most likely in roatations
        private static final double MAX_Elevator_HEIGHT = 100.0; // Adjust as needed
        private static final double MIN_Elevator_HEIGHT = 0.0;
    }

    public static final class AprilTags{
        //degress are facing inwards towards the tags
        //https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
        //https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
        public static final Pose2d ID_1  = new Pose2d(16.697, 0.655,  Rotation2d.fromDegrees(126));
        public static final Pose2d ID_2  = new Pose2d(16.697, 7.396,  Rotation2d.fromDegrees(234));
        public static final Pose2d ID_3  = new Pose2d(11.561, 8.056,  Rotation2d.fromDegrees(270));
        public static final Pose2d ID_4  = new Pose2d(9.276,  6.137,  Rotation2d.fromDegrees(0));
        public static final Pose2d ID_5  = new Pose2d(9.276,  1.915,  Rotation2d.fromDegrees(0));
        public static final Pose2d ID_6  = new Pose2d(13.474, 3.306,  Rotation2d.fromDegrees(300));
        public static final Pose2d ID_7  = new Pose2d(13.890, 4.026,  Rotation2d.fromDegrees(0));
        public static final Pose2d ID_8  = new Pose2d(13.474, 4.745,  Rotation2d.fromDegrees(60));
        public static final Pose2d ID_9  = new Pose2d(12.643, 4.745,  Rotation2d.fromDegrees(120));
        public static final Pose2d ID_10 = new Pose2d(12.227, 4.026,  Rotation2d.fromDegrees(180));
        public static final Pose2d ID_11 = new Pose2d(12.643, 3.306,  Rotation2d.fromDegrees(240));
        public static final Pose2d ID_12 = new Pose2d(0.851,  0.655,  Rotation2d.fromDegrees(54));
        public static final Pose2d ID_13 = new Pose2d(0.851,  7.396,  Rotation2d.fromDegrees(306));
        public static final Pose2d ID_14 = new Pose2d(8.272,  6.137,  Rotation2d.fromDegrees(180));
        public static final Pose2d ID_15 = new Pose2d(8.272,  1.915,  Rotation2d.fromDegrees(180));
        public static final Pose2d ID_16 = new Pose2d(5.987,  -0.004,  Rotation2d.fromDegrees(90));
        public static final Pose2d ID_17 = new Pose2d(4.073,  3.306,  Rotation2d.fromDegrees(240));
        public static final Pose2d ID_18 = new Pose2d(3.657,  4.026,  Rotation2d.fromDegrees(180));
        public static final Pose2d ID_19 = new Pose2d(4.074,  4.745,  Rotation2d.fromDegrees(120));
        public static final Pose2d ID_20 = new Pose2d(4.905,  4.745,  Rotation2d.fromDegrees(60));
        public static final Pose2d ID_21 = new Pose2d(5.321,  4.026,  Rotation2d.fromDegrees(0));
        public static final Pose2d ID_22 = new Pose2d(4.905,  3.306,  Rotation2d.fromDegrees(300));
        public static final List<Pose2d> aprilTags = new ArrayList<Pose2d>(List.of(ID_1,ID_2,ID_3,ID_4,ID_5,ID_6,ID_7,ID_8,ID_9,ID_10,ID_11,ID_12,ID_13,ID_14,ID_15,ID_16,ID_17,ID_18,ID_19,ID_20,ID_21,ID_22));
        public static final Map<String, Integer> blueAllianceIds = new HashMap<>() {{
            put("[1A]", 20);
            put("[1B]", 20);
            put("[2A]", 21);
            put("[2B]", 21);
            put("[3A]", 22);
            put("[3B]", 22);
            put("[4A]", 17);
            put("[4B]", 17);
            put("[5A]", 18);
            put("[5B]", 18);
            put("[6A]", 19);
            put("[6B]", 19);
        }};

        public static final Map<String, Integer> redAllianceIds = new HashMap<>() {{
            put("[1A]", 11);
            put("[1B]", 11);
            put("[2A]", 10);
            put("[2B]", 10);
            put("[3A]",  9);
            put("[3B]",  9);
            put("[4A]",  8);
            put("[4B]",  8);
            put("[5A]",  7);
            put("[5B]",  7);
            put("[6A]",  6);
            put("[6B]",  6);
        }};
    }

    public static final class LimelightConstants {
        public static final double mountHeightForwards = 1.32; //in meters
        public static final double mountAngleForwards = 0.0; //in degrees with straight being 0 up being 90 and down being -90
        public static final double horizontalOffsetForwards = -0.18; //in meters, this offset is how far left or right LL3 is mounted from center (negative is left, positive right)
        //this might not be needed but doesn't hurt us
        public static final double distanceFromCenterForwards = 0.3; //in meters, straight distance to the camera from middle
        public static final double hypotenuseDistanceForwards = Math.hypot(horizontalOffsetForwards, distanceFromCenterForwards); // actual distance in 2d from middle
        public static final double thethaFromCenterForwards = -32; //this might be needed for angle offset
        public static String llObjectDetectionNameForwards = "limelight";
        public static String llLocalizationNameForwards = "limelight";
        public static double targetHeightForwards = 0;
        
        public static final double mountHeightBackwards = 1.32; //in meters
        public static final double mountAngleBackwards = 0.0; //in degrees with straight being 0 up being 90 and down being -90
        public static final double horizontalOffsetBackwards = -0.18; //in meters, this offset is how far left or right LL3 is mounted from center (negative is left, positive right)
        //this might not be needed but doesn't hurt us
        public static final double distanceFromCenterBackwards = 0.3; //in meters, straight distance to the camera from middle
        public static final double hypotenuseDistanceBackwards = Math.hypot(horizontalOffsetForwards, distanceFromCenterForwards); // actual distance in 2d from middle
        public static final double thethaFromCenterBackwards = -32; //this might be needed for angle offset
        public static String llObjectDetectionNameBackwards = "limelight1";
        public static String llLocalizationNameBackwards = "limelight1Loc";
        public static double targetHeightBackwards = 0;
        
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
