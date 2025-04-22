// Copyright (c) Yo mum and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANAssignments;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.RobotCANUtils.CANSparkMaxController;
import frc.robot.util.RobotCANUtils.MotorKind;


public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMaxConfig configEl = new SparkMaxConfig();

    // Create ele(vator) motors
    private final CANSparkMaxController leftElevatorMotor = new CANSparkMaxController(CANAssignments.LEFT_ELEVATOR_MOTOR_ID, MotorKind.NEO30AMP, configEl, IdleMode.kBrake, 0.8, 0.0, 0.0, ElevatorConstants.MAX_VELOCITY_RPM, ElevatorConstants.MAX_ACCEL_RPM_S, 0.02 / ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER, 11.0);
    private final CANSparkMaxController rightElevatorMotor = new CANSparkMaxController(CANAssignments.RIGHT_ELEVATOR_MOTOR_ID, MotorKind.NEO30AMP, configEl, IdleMode.kBrake, 0.8, 0.0, 0.0, ElevatorConstants.MAX_VELOCITY_RPM, ElevatorConstants.MAX_ACCEL_RPM_S, 0.02 / ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER, 11.0);


    SparkClosedLoopController leftElevatorController = leftElevatorMotor.getClosedLoopController();
    SparkClosedLoopController rightElevatorController = rightElevatorMotor.getClosedLoopController();

    // set up relative encoders for elevator
    public RelativeEncoder relativeEncoderLeft = leftElevatorMotor.getEncoder();
    public RelativeEncoder relativeEncoderRight = rightElevatorMotor.getEncoder();

    public double currentPositionLeft = 0.0;
    public double currentPositionRight = 0.0;
    public static double currentHeight = 0.0;

    // setpoint
    public double setPointRotations = 0.0;

    //Current Elevator Level
    int currentElevatorLevel = 0;

    //Evevator presets (prob should go to constants)
    double loadStationPreset = 0.0;
    double L1Preset = 0.224;
    double L2Preset = 0.7973;
    double L3Preset = 1.1973;
    double L4Preset = 1.8173;

    public double[] elevatorHeights = new double[]{
            0.0, //loadSttion
            //0.574, // L1
            0.4674,
            //0.7473, //L2
            0.8547,
            //1.1373,//L3
            1.2147,
            //1.7723,  //L4
            1.8293,
            1.8223 //L5
    };


    public double metersToRotations(double setpointNumMeters) {
        return setpointNumMeters / ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER;

    }

    public double rotationsToMeters(double setpointNumRotations) {
        return setpointNumRotations * ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER;
    }

    // create L(#) preset methods

    //Drive bace in 3.8cm off the ground
    public void loadStation_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[0]);

    }

    public void L1_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[1]); // 46cm >> 42.2cm >> 0.224m
    }

    public void L2_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[2]);   // 81cm >> 77.2cm  >> 0.772m
    }

    public void L3_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[3]);  //121cm  >> 117.2cm  >>  1.172m
    }

    public void L4_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[4]);  //183cm >>  179.3cm  >>   1.793m
    }

    public void elevator_zero() {
        relativeEncoderLeft.setPosition(0.0);
        relativeEncoderRight.setPosition(0.0);
    }

    public void elevator_top() {
        relativeEncoderLeft.setPosition(-metersToRotations(elevatorHeights[4]));
        relativeEncoderRight.setPosition(metersToRotations(elevatorHeights[4]));
    }

    public ElevatorSubsystem() {
    }

    @Override
    public void periodic() {
        leftElevatorController.setReference(-setPointRotations, ControlType.kMAXMotionPositionControl);
        rightElevatorController.setReference(setPointRotations, ControlType.kMAXMotionPositionControl);
        currentPositionLeft = relativeEncoderLeft.getPosition();
        currentPositionRight = relativeEncoderRight.getPosition();
        // Logger.recordOutput("elevator_height", rotationsToMeters(setPointRotations));
        currentHeight = rotationsToMeters(currentPositionLeft);
    }


    public boolean isNearTargetPosition() {
        double metersError = 0.02;
        double targetMetersHeight = rotationsToMeters(setPointRotations);
        double currentMetersHeight = rotationsToMeters(relativeEncoderRight.getPosition());

        return Math.abs(targetMetersHeight - currentMetersHeight) <= metersError;
    }

}
