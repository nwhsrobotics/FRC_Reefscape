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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANAssignments;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.ImprovedCanSpark;
import org.littletonrobotics.junction.Logger;


public class ElevatorSubsystem extends SubsystemBase {
    // k values
    private final double kS = 0.0;
    private final double kG = 0.0;
    private final double kV = 0.0;
    private final double kA = 0.0;
    // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
    ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    private final SparkBaseConfig configEl = new SparkMaxConfig();

    //Encoder absoluteEncoder = new Encoder(null, null);
    // Create ele(vator) motors

    private final SparkMax leftElevatorMotor = new ImprovedCanSpark(CANAssignments.LEFT_ELEVATOR_MOTOR_ID, configEl, ImprovedCanSpark.MotorKind.NEO, IdleMode.kBrake, 0.8, 0.0, 0, ElevatorConstants.MAX_VELOCITY_RPM, ElevatorConstants.MAX_ACCEL_RPM_S, 0.02 / ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER);
    private final SparkMax rightElevatorMotor = new ImprovedCanSpark(CANAssignments.RIGHT_ELEVATOR_MOTOR_ID, configEl, ImprovedCanSpark.MotorKind.NEO, IdleMode.kBrake, 0.8, 0.0, 0, ElevatorConstants.MAX_VELOCITY_RPM, ElevatorConstants.MAX_ACCEL_RPM_S, 0.02 / ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER);

    SparkClosedLoopController leftElevatorController = leftElevatorMotor.getClosedLoopController();
    SparkClosedLoopController rightElevatorController = rightElevatorMotor.getClosedLoopController();

    //private final SparkMax leftElevatorMotor = new ImprovedCanSpark(CANAssignments.LEFT_ELEVATOR_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO, SparkBaseConfig.IdleMode.kBrake);

    //private final SparkMax rightElevatorMotor = new ImprovedCanSpark(CANAssignments.RIGHT_ELEVATOR_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO, SparkBaseConfig.IdleMode.kBrake);


    // set up absolute encoders for elevator
    //public final CANcoder absoluteEncoderLeft = new CANcoder(CANAssignments.CLIMB_ABSOLUTE_ENCODER_LEFT_ID);
    //public final CANcoder absoluteEncoderRight = new CANcoder(CANAssignments.CLIMB_ABSOLUTE_ENCODER_RIGHT_ID);


    // set up relative encoders for elevator
    public RelativeEncoder relativeEncoderLeft = leftElevatorMotor.getEncoder();
    public RelativeEncoder relativeEncoderRight = rightElevatorMotor.getEncoder();

    public double currentPositionLeft = 0.0;
    public double currentPositionRight = 0.0;
    public static double currentHeight = 0.0;

    //Create limit switches
    //DigitalInput toplimitSwitch = new DigitalInput(0);
    //DigitalInput bottomlimitSwitch = new DigitalInput(0);

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
        // configEl.closedLoop.maxMotion.apply(new MAXMotionConfig().maxAcceleration((2.5/ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER) * 60));
        // leftElevatorMotor.configure(configEl, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // rightElevatorMotor.configure(configEl, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
        // configEl.closedLoop.maxMotion.apply(new MAXMotionConfig().maxAcceleration((4.0/ElevatorConstants.ELEVATOR_MOTOR_ENCODER_ROT2METER) * 60));
        // leftElevatorMotor.configure(configEl, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // rightElevatorMotor.configure(configEl, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        setPointRotations = metersToRotations(elevatorHeights[4]);  //183cm >>  179.3cm  >>   1.793m
    }

    public void elevator_zero(){
        relativeEncoderLeft.setPosition(0.0);
        relativeEncoderRight.setPosition(0.0);
    }

    public void elevator_top(){
        relativeEncoderLeft.setPosition(-metersToRotations(elevatorHeights[4]));
        relativeEncoderRight.setPosition(metersToRotations(elevatorHeights[4]));
    }

    public void dynamic_Height() {
        setPointRotations = Math.min(setPointRotations + metersToRotations((VisionSubsystem.getStraightLineZDistance() * Math.sin(Math.toRadians(35)))), metersToRotations(2.0));
        //you can preset this to a seperate button or make this the new l4 button
        //this method assumes that 0 is the base of the april tag
        //this method will correct any preset based off of its position, so you would press a standard preset and adjust it with this one
    }

    public void dynamic_L4_Preset() {
        setPointRotations = metersToRotations((elevatorHeights[4] + Math.min(VisionSubsystem.getStraightLineZDistance(), 0.08) * Math.sin(Math.toRadians(35))));
        //you can preset this to a seperate button or make this the new l4 button
        //this method assumes that 0 is the base of the april tag
        //I reccomend using the old l4 elevator height from comp for the initial setpoint rotations
        //this preset is just an l4 preset itself, so if you want to go to l4 based on your position, just hit this one
    }

    public void boost() {
        setPointRotations = metersToRotations(elevatorHeights[5]);
    }

    public ElevatorSubsystem() {
    }

    @Override
    public void periodic() {
        leftElevatorController.setReference(-setPointRotations, ControlType.kMAXMotionPositionControl);
        rightElevatorController.setReference(setPointRotations, ControlType.kMAXMotionPositionControl);
        // System.out.println("leftElevatorController:" + relativeEncoderLeft.getPosition());
        // System.out.println("rightElevatorController:" + relativeEncoderRight.getPosition());
        // System.out.println("difference:" + String.valueOf(Math.abs(relativeEncoderRight.getPosition()) - Math.abs(relativeEncoderLeft.getPosition())));

        // leftElevatorController.setReference(setPointRotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedforward.calculate(0.0));
        // rightElevatorController.setReference(setPointRotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedforward.calculate(0.0));
        // if( VisionSubsystem.getStraightLineZDistance() > 0.1){
        //     leftElevatorController.setReference(-setPointRotations, ControlType.kMAXMotionPositionControl);
        //     rightElevatorController.setReference(setPointRotations, ControlType.kMAXMotionPositionControl);
        // }
        // else if(VisionSubsystem.getStraightLineZDistance() <= 0.1 && setPointRotations != 0.0){
        //     leftElevatorController.setReference(-setPointRotations + -metersToRotations(VisionSubsystem.getStraightLineZDistance() * Math.sin(Math.toRadians(35))), ControlType.kMAXMotionPositionControl);
        //     rightElevatorController.setReference(setPointRotations + metersToRotations(VisionSubsystem.getStraightLineZDistance() * Math.sin(Math.toRadians(35))), ControlType.kMAXMotionPositionControl);
        // }
        
        
        //leftElevatorController.setReference(-setPointRotations, ControlType.kMAXMotionPositionControl);
        //rightElevatorController.setReference(setPointRotations, ControlType.kMAXMotionPositionControl);
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
