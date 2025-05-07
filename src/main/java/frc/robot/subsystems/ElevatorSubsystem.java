// Copyright (c) Yo mum and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANAssignments;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.ImprovedCanSpark;
import org.littletonrobotics.junction.Logger;


public class ElevatorSubsystem extends SubsystemBase {
    //Config object for changing applying spark max controller configuration for left and right motor controllers (PID Tuning, MAXMotion, Standard configs)
    private final SparkBaseConfig configEl = new SparkMaxConfig();

    //Left and right elevator motors with PID tuning values and MAXMotion values
    private final SparkMax leftElevatorMotor = new ImprovedCanSpark(CANAssignments.LEFT_ELEVATOR_MOTOR_ID, configEl, ImprovedCanSpark.MotorKind.NEO, IdleMode.kBrake, 0.8, 0.0, 0, ElevatorConstants.MAX_VELOCITY_RPM, ElevatorConstants.MAX_ACCEL_RPM_S, 0.02 / ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER);
    private final SparkMax rightElevatorMotor = new ImprovedCanSpark(CANAssignments.RIGHT_ELEVATOR_MOTOR_ID, configEl, ImprovedCanSpark.MotorKind.NEO, IdleMode.kBrake, 0.8, 0.0, 0, ElevatorConstants.MAX_VELOCITY_RPM, ElevatorConstants.MAX_ACCEL_RPM_S, 0.02 / ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER);
    
    //Pid controllers for left and right motors
    SparkClosedLoopController leftElevatorController = leftElevatorMotor.getClosedLoopController();
    SparkClosedLoopController rightElevatorController = rightElevatorMotor.getClosedLoopController();

    // set up relative encoders for elevator
    public RelativeEncoder relativeEncoderLeft = leftElevatorMotor.getEncoder();
    public RelativeEncoder relativeEncoderRight = rightElevatorMotor.getEncoder();

    //Current motor rotations/position and elevators current height for logging
    public double currentPositionLeft = 0.0;
    public double currentPositionRight = 0.0;
    public static double currentHeight = 0.0;

    //Set value of rotations for motor to turn to move the elevator to the desired height
    public double setPointRotations = 0.0;

    //Evevator preset heights for each level
    double loadStationPreset = 0.0;
    double L1Preset = 0.224;
    double L2Preset = 0.7973;
    double L3Preset = 1.1973;
    double L4Preset = 1.8173;

    public double[] elevatorHeights = new double[]{
            0.0, //loadstation
            0.4674, //L1
            0.8547, //L2
            1.2147,  //L3
            1.8293,  //L4
            1.8500  //Boost
    };

    //Conversion for desired vertical height in meters to motor rotations
    public double metersToRotations(double setpointNumMeters) {
        return setpointNumMeters / ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER;
    }
    
    //Conversion for desired motor rotations to desired vertical height in meters
    public double rotationsToMeters(double setpointNumRotations) {
        return setpointNumRotations * ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER;
    }

    //Set the desired motors rotations to the rotations it take to reach the loadstation
    public void loadStation_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[0]);
    }

    //Set the desired motors rotations to the rotations it take to reach the L1
    public void L1_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[1]);
    }

    //Set the desired motors rotations to the rotations it take to reach the L2
    public void L2_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[2]);
    }

    //Set the desired motors rotations to the rotations it take to reach the L3
    public void L3_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[3]);
    }

    //Set the desired motors rotations to the rotations it take to reach the L4
    public void L4_Preset() {
        setPointRotations = metersToRotations(elevatorHeights[4]);
    }

    //Set the current position of elevator encoders to the lowest position (0 meters)
    //Usually used in cases if the auto doesn't go correctly and the elevator gets stuck at the bottom
    //The pid controller will move the elevator up since it accounts for the difference in heights
    public void elevator_zero(){
        relativeEncoderLeft.setPosition(0.0);
        relativeEncoderRight.setPosition(0.0);
    }

    //Set the current position of the elevator encoders to the highest position (L4)
    //Usually used in cases if the auto doesn't go correctly and the elevator gets stuck at the top
    //The pid controller will move the elevator back down since it accounts for the difference in heights
    public void elevator_top(){
        relativeEncoderLeft.setPosition(-metersToRotations(elevatorHeights[4]));
        relativeEncoderRight.setPosition(metersToRotations(elevatorHeights[4]));
    }

    //Set the desired motor rotations to corrent amount of rotations to score at previously pressed preset at any drive base position relative to the base of the reef
    public void dynamic_Height() {
        setPointRotations = Math.min(setPointRotations + metersToRotations((VisionSubsystem.getStraightLineZDistance() * Math.sin(Math.toRadians(35)))), metersToRotations(2.0));
    }

    //Set the desired motor rotations to corrent amount of rotations to score at L4 at any drive base position relative to the base of the reef
    //This method can be implemented for any desired level by changing the elevatorHeights[] index to the desired position
    public void dynamic_L4_Preset() {
        setPointRotations = metersToRotations((elevatorHeights[4] + Math.min(VisionSubsystem.getStraightLineZDistance(), 0.08) * Math.sin(Math.toRadians(35))));
    }

    //Set the desired motor rotations to the rotations it takes to reach 2 inches aboce the height for L4
    //This is for giving a little kick to the coral if the scores a little under the peg of L4
    public void boost() {
        setPointRotations = metersToRotations(elevatorHeights[5]);
    }

    public ElevatorSubsystem() {
    }

    @Override
    public void periodic() {
        //Set the desired rotations periodically for the left motor and the right motor
        //Left motor is inverted with the negative sign
        //Control type is MAXMotion for the motion profile
        leftElevatorController.setReference(-setPointRotations, ControlType.kMAXMotionPositionControl);
        rightElevatorController.setReference(setPointRotations, ControlType.kMAXMotionPositionControl);
        
        //Periodically stores the current encoder positions of the left and right motors
        currentPositionLeft = relativeEncoderLeft.getPosition();
        currentPositionRight = relativeEncoderRight.getPosition();

        //periodically outputting the current elevator height in meters into log files
        Logger.recordOutput("elevator_height", rotationsToMeters(setPointRotations));

        //Periodically stores the current height of the elevator in meters
        currentHeight = rotationsToMeters(currentPositionLeft);
    }


    //Method to make sure the program knows the current elevator height is in the allowed range in meters for auto routine scheduling
    //Mainly used in auto scoring buttons that the gunner controls
    public boolean isNearTargetPosition() {
        double metersError = 0.02;
        double targetMetersHeight = rotationsToMeters(setPointRotations);
        double currentMetersHeight = rotationsToMeters(relativeEncoderRight.getPosition());

        //return the truth of whether the magnitude of the difference in the desired height and the current height is within the allowed error range
        return Math.abs(targetMetersHeight - currentMetersHeight) <= metersError;
    }

}
