package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANAssignments;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.SwerveUtils;
import org.littletonrobotics.junction.Logger;

/**
 * Represents the swerve drive subsystem, managing four swerve modules and handling overall robot control.
 */
public class SwerveSubsystem extends SubsystemBase {
    // when the robot is set to "field relative,"
    // linear movement will be relative to the field.
    //
    // for example, even if the robot is oriented towards the driver station,
    // holding forwards will move the robot away from the driver station,
    // because that is the forwards direction relative to the field.
    private boolean isFieldRelative = true;

    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public AutoNavigation autonavigator;

    // 4 instances of SwerveModule to represent each wheel module with the constants
    private final SwerveModule frontLeft = new SwerveModule(
            CANAssignments.FRONT_LEFT_DRIVE_MOTOR_ID,
            CANAssignments.FRONT_LEFT_STEER_MOTOR_ID,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            CANAssignments.FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            CANAssignments.FRONT_RIGHT_DRIVE_MOTOR_ID,
            CANAssignments.FRONT_RIGHT_STEER_MOTOR_ID,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            CANAssignments.FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            CANAssignments.BACK_LEFT_DRIVE_MOTOR_ID,
            CANAssignments.BACK_LEFT_STEER_MOTOR_ID,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            CANAssignments.BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            CANAssignments.BACK_RIGHT_DRIVE_MOTOR_ID,
            CANAssignments.BACK_RIGHT_STEER_MOTOR_ID,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            CANAssignments.BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    // array of SwerveModules for convenience in accessing all modules
    private final SwerveModule[] swerveMods = {frontLeft, frontRight, backLeft, backRight};

    // create an AHRS object for gyro
    public final AHRS gyro = new AHRS(NavXComType.kUSB1);

    //odometry is a system to keep track of robots current position and rotation on the fields based on the coordinate system
    private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(getHeading()),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5)));
    //The default standard deviations of the module states are 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading. 
    //The default standard deviations of the vision measurements are 0.9 meters for x, 0.9 meters for y, and 0.9 radians for heading.
    //Decrease standard deviations to trust the data more (right now the vision is mostly insignificant compared to module state)

    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;

    private final SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double prevTime = WPIUtilJNI.now() * 1e-6;

    public static Pose2d currentPose = new Pose2d();

    /**
     * Constructor for the SwerveSubsystem class.
     * Configures the AutoBuilder for holonomic/swerve path planning and initializes the gyro.
     */
    public SwerveSubsystem() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder for holonomic/swerve path planning & paths
            AutoBuilder.configure(
                    this::getPose,               // Supplier for getting the robot's pose
                    this::resetOdometry,         // Runnable for resetting odometry
                    this::getSpeeds,             // Supplier for getting the robot's speeds
                    this::driveRobotRelative,    // Consumer for driving the robot relative to its orientation
                    AutoConstants.pathFollowerConfig, // Path follower configuration
                    config,                     // Robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this

            );

            this.autonavigator = new AutoNavigation(this);

            // Pause for 500 milliseconds to allow the gyro to stabilize.
            // Set the yaw of the gyro to 0 afterwards (hardware offset).
            // Calculate sysid MOI for swerve/pathplanner constants, swerve setpoint generator, and elastic notifications
            //gyro.reset();
            Commands.waitUntil(() -> !gyro.isCalibrating()).andThen(new InstantCommand(() -> gyro.zeroYaw()));
            setpointGenerator = new SwerveSetpointGenerator(
                    config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
                    Units.rotationsToRadians(10.0) // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
            );
            // Initialize the previous setpoint to the robot's current speeds & module states
            ChassisSpeeds currentSpeeds = getSpeeds(); // Method to get current robot-relative chassis speeds
            SwerveModuleState[] currentStates = getModuleStates(); // Method to get the current swerve module states
            previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
            /*Commands.waitSeconds(0.5)
                    .andThen(new RunCommand(() -> gyro.zeroYaw()));*/
        } catch (Exception e) {
            e.printStackTrace();
            Logger.recordOutput("errors.autobuilder", "initializing: " + e.toString());
            setpointGenerator = null;
        }


    }

    /**
     * Set the speed of the robot in the x direction.
     *
     * @param xSpeed The desired speed in the x direction.
     */
    public void setSpeed(double xSpeed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0.0, 0.0);
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Get the heading (yaw) of the robot.
     *
     * @return The heading of the robot in degrees.
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * -1;
    }

    /**
     * Get the pitch of the robot in degrees.
     *
     * @return The pitch of the robot in degrees.
     */
    public double getPitchDeg() {
        return -gyro.getPitch();
    }

    public boolean isFieldRelative() {
        return isFieldRelative;
    }

    /**
     * Drive the robot relative to its current orientation.
     *
     * @param robotRelativeSpeeds The desired robot-relative speeds.
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        // swerve setpoint generator (uncomment this)
        // previousSetpoint = setpointGenerator.generateSetpoint(
        //     previousSetpoint, // The previous setpoint
        //     robotRelativeSpeeds, // The desired target speeds
        //     0.02 // The loop time of the robot code, in seconds
        // );
        // setModuleStates(previousSetpoint.moduleStates()); // Method that will drive the robot given target module states

        // earlier code
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }


    /**
     * Reset the heading (yaw) and the odometry pose of the robot.
     */
    public void resetHeading() {
        gyro.zeroYaw(); // Reset the yaw angle
    }

    /**
     * Switch between field-relative and robot-relative driving.
     */
    public void switchFR() {
        isFieldRelative = !isFieldRelative; // switch between field-relative and robot-relative driving
    }

    /**
     * Get the speeds of the robot.
     *
     * @return The speeds of the robot.
     */
    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Get the current pose of the robot in meters.
     *
     * @return The current pose of the robot.
     */
    public Pose2d getPose() {
        // return odometer.getPoseMeters();
        return odometer.getEstimatedPosition(); // get the robot's current pose from the odometry system
    }

    /**
     * MAKE SURE TO FLIP POSTION IF NEEDED GeometryUtil.flipFieldPose
     * <p>
     * Reset the odometry with the specified pose.
     *
     * @param pose The desired pose for resetting odometry.
     */
    public void resetOdometry(Pose2d pose) {
        // Reset the odometry system using the current heading, module positions, and specified pose
        odometer.resetPosition(Rotation2d.fromDegrees(getHeading()), getModulePositions(), pose);
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    /**
     * Straighten the orientation of each swerve module.
     */
    public void straighten() {
        // Turn each swerve module to point straight ahead
        for (SwerveModule s_mod : swerveMods) {
            s_mod.straighten();
        }
    }

    /**
     * Get the states of all swerve modules.
     *
     * @return The states of all swerve modules.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < swerveMods.length; i++) {
            states[i] = swerveMods[i].getState();
        }
        return states;
    }

    /**
     * Get the positions of all swerve modules.
     *
     * @return The positions of all swerve modules.
     */
    public SwerveModulePosition[] getModulePositions() {
        // Get the current position of each swerve module
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < swerveMods.length; i++) {
            positions[i] = swerveMods[i].getPosition();
        }
        return positions;
    }


    // This method is called periodically to update the robot's state and log data
    @Override
    public void periodic() {
        updateOdometry();
        currentPose = getPose();

        // Log position of robot.
        Logger.recordOutput("swerve.pose", getPose());
        Logger.recordOutput("swerve.odometer", odometer.getEstimatedPosition());
        Logger.recordOutput("swerve.odometer.xCoordinate", odometer.getEstimatedPosition().getX());
        Logger.recordOutput("swerve.odometer.yCoordinate", odometer.getEstimatedPosition().getY());
        Logger.recordOutput("swerve.odometer.rotation", odometer.getEstimatedPosition().getRotation().getDegrees());

        // Log whether robot is driving in field relative mode.
        Logger.recordOutput("swerve.isfieldrelative", isFieldRelative);

        // Log pitch and heading of IMU.
        Logger.recordOutput("swerve.pitch", getPitchDeg());
        Logger.recordOutput("swerve.heading", getHeading());


        // Log steering direction.
        Logger.recordOutput("swerve.steer.front.left.abs", frontLeft.getAbsoluteEncoderRad());
        Logger.recordOutput("swerve.steer.front.right.abs", frontRight.getAbsoluteEncoderRad());
        Logger.recordOutput("swerve.steer.back.left.abs", backLeft.getAbsoluteEncoderRad());
        Logger.recordOutput("swerve.steer.back.right.abs", backRight.getAbsoluteEncoderRad());

        Logger.recordOutput("swerve.steer.front.left.absraw", frontLeft.getAbsoluteEncoderRadRaw());
        Logger.recordOutput("swerve.steer.front.right.absraw", frontRight.getAbsoluteEncoderRadRaw());
        Logger.recordOutput("swerve.steer.back.left.absraw", backLeft.getAbsoluteEncoderRadRaw());
        Logger.recordOutput("swerve.steer.back.right.absraw", backRight.getAbsoluteEncoderRadRaw());

        // Log travel velocity.
        Logger.recordOutput("swerve.drive.front.left.velocity", frontLeft.getDriveVelocity());
        Logger.recordOutput("swerve.drive.front.right.velocity", frontRight.getDriveVelocity());
        Logger.recordOutput("swerve.drive.back.left.velocity", backLeft.getDriveVelocity());
        Logger.recordOutput("swerve.drive.back.right.velocity", backRight.getDriveVelocity());

        // Below code is just to test elastic dashboard custom widget
        //    SmartDashboard.putData("Swerve Drive", new Sendable() {
        //         @Override
        //         public void initSendable(SendableBuilder builder) {
        //             builder.setSmartDashboardType("SwerveDrive");

        //             builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getTurningPosition(), null);
        //             builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity(), null);

        //             builder.addDoubleProperty("Front Right Angle", () -> frontRight.getTurningPosition(), null);
        //             builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity(), null);

        //             builder.addDoubleProperty("Back Left Angle", () -> backLeft.getTurningPosition(), null);
        //             builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity(), null);

        //             builder.addDoubleProperty("Back Right Angle", () -> backRight.getTurningPosition(), null);
        //             builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity(), null);

        //             builder.addDoubleProperty("Robot Angle", () -> gyro.getAngle(), null);
        //         }
        //         });

    }

    /**
     * Stops all of the robot's swerve modules.
     * Iterates through each swerve module and stops its motion.
     */
    public void stopModules() {
        for (SwerveModule sMod : swerveMods) {
            sMod.stop();
        }
    }

    /**
     * Sets the states of the robot's swerve modules based on the desired states.
     * Scales all speeds down instead of truncating them if they exceed the max speed.
     *
     * @param desiredStates An array of SwerveModuleState representing the desired states for each swerve module.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Scales all speeds down instead of truncating them if they exceed the max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Orient wheels into a X (clover) position to prevent movement
     */
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Update odometry periodically with vision and internal measurements
     */
    public void updateOdometry() {
        odometer.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

        addVisionMeasurement(LimelightConstants.llFront, 0.00, 0.00, 9999999);
        //dont need the back one most likely
        //addVisionMeasurement(LimelightConstants.llBack, 0.2, 0.2, 9999999);
    }


    /**
     * Adds a vision measurement from the specified limelight.
     *
     * @param limelightName The name of the limelight (LimelightConstants.llFront).
     * @param stdX          The standard deviation for the x position.
     * @param stdY          The standard deviation for the y position.
     * @param stdTheta      The standard deviation for the rotation (radians, but usually 9999999 since vision rotation is very inaccurate).
     */
    private void addVisionMeasurement(String limelightName, double stdX, double stdY, double stdTheta) {
        try {
            if (!VisionAprilTag.isAprilTagPipeline(limelightName)) {
                return;
            }
            boolean useMegaTag2 = true; // Set to false to use the MegaTag1 branch if desired.
            // we might use megatag1 when disabled to auto orient and megatag2 when enable
            // here: https://www.chiefdelphi.com/t/introducing-megatag2-by-limelight-vision/461243/78
            if (RobotState.isDisabled()){
                // If doing this, untick mark reset odometry in pathplanner
                useMegaTag2 = false;
            }
            boolean doRejectUpdate = false;

            if (!useMegaTag2) {
                // LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                // if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                //     if (mt1.rawFiducials[0].ambiguity > 0.7) {
                //         doRejectUpdate = true;
                //     }
                //     if (mt1.rawFiducials[0].distToCamera > 3) {
                //         doRejectUpdate = true;
                //     }
                // }
                // if (mt1.tagCount == 0) {
                //     doRejectUpdate = true;
                // }
                // if (!doRejectUpdate) {
                //     odometer.setVisionMeasurementStdDevs(VecBuilder.fill(stdX, stdY, stdTheta));
                //     odometer.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                Pose2d currentPose = getPose();
                if (mt1.pose.getRotation().getDegrees() == 0.0){
                    return;
                }
                Logger.recordOutput("mt1.rotation", mt1.pose.getRotation());
                Pose2d finalPoseRotated = new Pose2d(currentPose.getX(), currentPose.getY(), mt1.pose.getRotation());
                resetOdometry(finalPoseRotated);
                Logger.recordOutput("mt1.pose", finalPoseRotated);
                Logger.recordOutput("mt1.poseReset", getPose());
                LimelightHelpers.SetRobotOrientation(limelightName, odometer.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                Logger.recordOutput("mt2.pose", mt2.pose);
                Logger.recordOutput("mt2.posex", mt2.pose.getX());
                Logger.recordOutput("mt2.posey", mt2.pose.getY());
                odometer.setVisionMeasurementStdDevs(VecBuilder.fill(stdX, stdY, stdTheta));
                if (mt2.pose.getX() == 0.0 || mt2.pose.getY() == 0.0 || mt2.pose.getRotation().getDegrees() == 0.0){
                    return;
                }
                resetOdometry(mt2.pose);
                Logger.recordOutput("mt2.final", getPose());
                Logger.recordOutput("mt2.finalx", getPose().getX());
                Logger.recordOutput("mt2.finaly", getPose().getY());
            } else {
                // Always set robot orientation before getting MegaTag2 measurement
                LimelightHelpers.SetRobotOrientation(limelightName, odometer.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                if (Math.abs(gyro.getRate()) > 720) { // reject if the robot is spinning too fast
                    doRejectUpdate = true;
                }
                if (mt2.tagCount == 0) {
                    doRejectUpdate = true;
                }
                if (!doRejectUpdate) {
                    odometer.setVisionMeasurementStdDevs(VecBuilder.fill(stdX, stdY, stdTheta));
                    odometer.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
                }
            }
        } catch (Exception error) {
            Logger.recordOutput("errors.vision", "Limelight disconnected: " + error);
            //error.printStackTrace();
        }
    }


    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    currentTranslationMag = magLimiter.calculate(0.0);
                } else {
                    currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
                    currentTranslationMag = magLimiter.calculate(inputTranslationMag);
                }
            } else {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(0.0);
            }
            prevTime = currentTime;

            xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
            ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
            currentRotation = rotLimiter.calculate(rot);
        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double rotDelivered = currentRotation * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, odometer.getEstimatedPosition().getRotation())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Update odometry using precise vision measurements with high accuracy.
     * <p>
     * It is RECOMMENDED to stand still and be close to the April tag when resetting this way as it solely relies on vision
     * Use Megatag 2 if gryo rotation is accurate, otherwise use megatag 1 to also fix gyro rotation.
     */
    public void resetOdometryWithVision(boolean useMegaTag2) {
        String name = LimelightConstants.llFront;
        int pipeline = (int) LimelightHelpers.getCurrentPipelineIndex(name);
        //set the pipeline index to the high resolution april tag (less fps but high accuracy)
        LimelightHelpers.setPipelineIndex(name, 0);
        LimelightHelpers.SetRobotOrientation(LimelightConstants.llFront, odometer.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.llFront);
        LimelightHelpers.PoseEstimate llmtg1Measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.llFront);
        odometer.setVisionMeasurementStdDevs(VecBuilder.fill(0, 0, Units.degreesToRadians(0)));

        //megatag 1, will also fix gyro rotation
        if (!useMegaTag2) {
            odometer.addVisionMeasurement(llmtg1Measurement.pose, llmtg1Measurement.timestampSeconds);
        } else {
            //megatag 2 (assume already accurate gyro rotation)
            odometer.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
        }
        //set back to normal april tag pipeline
        LimelightHelpers.setPipelineIndex(name, pipeline);
    }
}