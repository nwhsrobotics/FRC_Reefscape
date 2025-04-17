package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import swervelib.math.Matter;

import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public final class Constants {

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(14.5);


    public static final class CANAssignments {
        public static final int ALGAE_MOTOR_ID = 19;

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
        public static final int BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 11;

        public static final int LEFT_ELEVATOR_MOTOR_ID = 13;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 14;


        public static final int PDU_ID = 24;
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

    public final class Buttons {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int VIEW = 7;
        public static final int MENU = 8;
        public static final int LEFT_STICK_BUTTON = 9;
        public static final int RIGHT_STICK_BUTTON = 10;
        public static final int POV_UP = 0;
        public static final int POV_DOWN = 180;
        public static final int POV_RIGHT = 90;
        public static final int POV_LEFT = 270;
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

        //0.9, 1.35, 1.5
        public static final double kDirectionSlewRate = 0.95; // radians per second
        public static final double kMagnitudeSlewRate = 1.425; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 1.3; // percent per second (1 = 100%)

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
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.607456392002714;//2.66 + Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.6902913545485385;//5.24 - Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.06934008285773;//0.61 + Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -2.2779614700101773;//5.20 - Math.PI;


        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI; // set up for NEOs to drive
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0); //adapted from SDS
    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kPXController = 5;
        public static final double kPThetaController = 5;

        public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
                new PIDConstants(AutoConstants.kPXController, 0, 0), // Translation constants
                new PIDConstants(AutoConstants.kPThetaController, 0, 0)// Rotation constants
        );

        public static final PathConstraints kPathfindingConstraints = new PathConstraints(
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared * 0.3,
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

        public static final Pose2d LEFT_GROUND_START_PIECES = new Pose2d(1.219, 5.855, Rotation2d.fromDegrees(180));
        public static final Pose2d MID_GROUND_START_PIECES = new Pose2d(1.219, 4.022, Rotation2d.fromDegrees(180));
        public static final Pose2d RIGHT_GROUND_START_PIECES = new Pose2d(1.219, 2.189, Rotation2d.fromDegrees(180));

        //These are the starting positions near the respective cages, they are on starting line, not in the cage itself
        public static final Pose2d START_A = new Pose2d(7.584, 7.283, Rotation2d.fromDegrees(180));
        public static final Pose2d START_B = new Pose2d(7.584, 6.185, Rotation2d.fromDegrees(180));
        public static final Pose2d START_C = new Pose2d(7.584, 5.056, Rotation2d.fromDegrees(180));

        public static final Pose2d BACK_LEFT_REEF = new Pose2d(3.645, 5.454, Rotation2d.fromDegrees(-60)); // POV_LEFT - Side 6
        public static final Pose2d BACK_MID_REEF = new Pose2d(2.837, 4.029, Rotation2d.fromDegrees(0.01)); // A button - Side 5
        public static final Pose2d BACK_RIGHT_REEF = new Pose2d(3.614, 2.613, Rotation2d.fromDegrees(60)); // POV_RIGHT - Side 4
        public static final Pose2d FRONT_LEFT_REEF = new Pose2d(5.352, 5.444, Rotation2d.fromDegrees(-120)); // X button - Side 1
        public static final Pose2d FRONT_MID_REEF = new Pose2d(6.152, 4.026, Rotation2d.fromDegrees(180)); // Y button - Side 2
        public static final Pose2d FRONT_RIGHT_REEF = new Pose2d(5.310, 2.582, Rotation2d.fromDegrees(120)); // B button - Side 3

        public static final Pose2d FRONT_REEF = new Pose2d(7, 4, Rotation2d.fromDegrees(180));
        public static final Pose2d BACK_REEF = new Pose2d(2.5, 4, Rotation2d.fromDegrees(0.01));

        public static final Pose2d STATION_LEFT = new Pose2d(1.201, 7.018, Rotation2d.fromDegrees(126)); //this is for the center of the source 
        public static final Pose2d STATION_RIGHT = new Pose2d(1.149, 1.043, Rotation2d.fromDegrees(-126)); //this is for the center of the source 

        public static final Pose2d PROCESSOR = new Pose2d(1.201, 7.018, Rotation2d.fromDegrees(-90));


        public static final Pose2d BLUE_REEF_CENTER = new Pose2d(4.5, 4.05, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_REEF_CENTER = new Pose2d(13, 4.05, Rotation2d.fromDegrees(0));

        public static final List<Pose2d> REEF_CENTERS = new ArrayList<>(List.of(BLUE_REEF_CENTER, RED_REEF_CENTER));


        public static final List<Pose2d> allNotes = new ArrayList<Pose2d>();
        // get rid of temp fix
    }

    public static final class ElevatorConstants {
        // Elevator Limits
        //units are most likely in roatations
        public static final double MAX_Elevator_HEIGHT = 100.0; // Adjust as needed
        public static final double MIN_Elevator_HEIGHT = 0.0;
        public static final double ELEVATOR_GEAR_RATIO = 1.0 / 12.0; //3:1, change later to 12:1

        public static final double SPROCKET_TEETH = 22.0;
        public static final double CHAIN_PITCH_INCH = 0.25;

        public static final double METERS_PER_INCH = 0.0254;

        public static final double ELEVATOR_STAGES = 3.0;


        public static final double ELEVATOR_MOTOR_ENCODER_ROT2METER = ELEVATOR_GEAR_RATIO * SPROCKET_TEETH * CHAIN_PITCH_INCH * METERS_PER_INCH * ELEVATOR_STAGES;
        public static final double ELEVATOR_ENCODER_METER_PER_SECONDS = ELEVATOR_MOTOR_ENCODER_ROT2METER / 60;

        public static final double MAX_VELOCITY_M_S = 4.0;

        public static final double MAX_ACCEL_M_S2 = 2.5;

        public static final double MAX_VELOCITY_RPM = ((MAX_VELOCITY_M_S) / ELEVATOR_MOTOR_ENCODER_ROT2METER) * 60.0;

        public static final double MAX_ACCEL_RPM_S = ((MAX_ACCEL_M_S2) / ELEVATOR_MOTOR_ENCODER_ROT2METER) * 60.0;
    }

    public static final class AprilTags {
        //degrees are facing inwards towards the tags (NOT THE ACTUAL TAG DEGREE)
        //it is tagDegree + 180
        //https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
        //https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
        //NO 0 DEGREE rotation, pathpathplanner bugs, do 1 degree instead   
    
        public static final List<Pose2d> aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTags().stream().map(tag -> tag.pose.toPose2d()).collect(Collectors.toCollection(ArrayList::new));
    }

    public static final class LimelightConstants {
        // 3.8cm or 0.038m base height of robot (add to all heights)
        //Forward Limelight
        public static final double mountHeightForwards = 0.625; //in meters
        public static final double mountAngleForwards = 17.5; //in degrees with straight being 0 up being 90 and down being -90
        public static final double horizontalOffsetForwards = 0.0; //in meters, this offset is how far left or right LL3 is mounted from center (negative is left, positive right)
        //this might not be needed but doesn't hurt us
        public static final double distanceFromCenterForwards = 0.0387; //in meters, straight distance to the camera from middle
        public static final double hypotenuseDistanceForwards = Math.hypot(horizontalOffsetForwards, distanceFromCenterForwards); // actual distance in 2d from middle
        public static final double thethaFromCenterForwards = -32; //this might be needed for angle offset
        public static String llObjectDetectionNameForwards = "limelight-llf";
        public static String llFront = "limelight-llf";
        public static double targetHeightForwards = 0;

        //Rear Limelight
        public static final double mountHeightBackwards = 0.8814; //in meters
        public static final double mountAngleBackwards = 0.0; //in degrees with straight being 0 up being 90 and down being -90
        public static final double horizontalOffsetBackwards = 0; //in meters, this offset is how far left or right LL3 is mounted from center (negative is left, positive right)
        //this might not be needed but doesn't hurt us
        public static final double distanceFromCenterBackwards = 0.0317; //in meters, straight distance to the camera from middle
        public static final double hypotenuseDistanceBackwards = Math.hypot(horizontalOffsetBackwards, distanceFromCenterBackwards); // actual distance in 2d from middle
        public static final double thethaFromCenterBackwards = -32; //this might be needed for angle offset
        public static String llObjectDetectionNameBackwards = "limelight-llb";
        public static String llBack = "limelight-llb";
        public static double targetHeightBackwards = 0;
        //Fix ll name

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
        public static final String RUNNING_UNDER = "2025.q2";

        // SET TO FALSE IF WE'RE RUNNING OUT OF BANDWIDTH.
        public static final boolean SILENT_NT4 = false;
    }
}
