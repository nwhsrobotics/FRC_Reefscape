// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANAssignments;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.ImprovedCanSpark;




public class ElevatorSubsystem extends SubsystemBase {
  
  // k values
  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
  ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  Encoder absoluteEncoder = new Encoder(null, null);

  // Create ele(vator) motors
  private final SparkMax leftElevatorMotor = new ImprovedCanSpark(CANAssignments.LEFT_ELEVATOR_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, null, SparkBaseConfig.IdleMode.kBrake);
  private final SparkMax rightElevatorMotor = new ImprovedCanSpark(CANAssignments.RIGHT_ELEVATOR_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, null, SparkBaseConfig.IdleMode.kBrake);
  // set up absolute encoders for elevator
  //public final CANcoder absoluteEncoderLeft = new CANcoder(CANAssignments.CLIMB_ABSOLUTE_ENCODER_LEFT_ID);
  //public final CANcoder absoluteEncoderRight = new CANcoder(CANAssignments.CLIMB_ABSOLUTE_ENCODER_RIGHT_ID);

  
  // set up relative encoders for elevator
  public RelativeEncoder relativeEncoderLeft = leftElevatorMotor.getEncoder();
  public RelativeEncoder relativeEncoderRight = rightElevatorMotor.getEncoder();

  

  //PID controllers 
  public PIDController pidControllerLeft = new PIDController(0, 0, 0);
  public PIDController pidControllerRight = new PIDController(0, 0, 0);

  //Create limit switches
  DigitalInput toplimitSwitch = new DigitalInput(0);
  DigitalInput bottomlimitSwitch = new DigitalInput(0);

  // setpoint
  private double setPointRotations = 0.0;

  //Current Elevator Level
  int currentElevatorLevel = 0;

  //Evevator presets (prob should go to constants)
  double loadStationPreset = 0.0;
  double L1Preset = 0.224 ; 
  double L2Preset = 0.7973;
  double L3Preset = 1.1973;
  double L4Preset = 1.8173; 

  double[] elevatorHeights = new double[] {
    0.0, //loadStation 
    0.224, // L1
    0.7973, //L2
    1.1973,  //L3 
    1.8173  //L4 
  };
 
  
  public double metersToRotations(double setpointNumMeters){
    return setpointNumMeters * ElevatorConstants.ELEVATOR_GEAR_RATIO;

  }
  
  // create L(#) preset methods

  //Drive bace in 3.8cm off the ground  
  public void loadStation_Preset(){
    setPointRotations = metersToRotations(loadStationPreset); 
  }
  public void L1_Preset() {
    setPointRotations = metersToRotations(L1Preset); // 46cm >> 42.2cm >> 0.224m
  }
  public void L2_Preset() {
    setPointRotations = metersToRotations(L2Preset);   // 81cm >> 77.2cm  >> 0.772m  
  }
  public void L3_Preset() {
    setPointRotations = metersToRotations(L3Preset);  //121cm  >> 117.2cm  >>  1.172m
  }
  public void L4_Preset() {
    setPointRotations = metersToRotations(L4Preset);  //183cm >>  179.3cm  >>   1.793m 
  }
  

  //Seting elevator heights 
  public void increaseCurrentLevel(){
    if(currentElevatorLevel>=elevatorHeights.length-1){
      System.out.println("LEVEL IS CURRENTLY 5 AND CAN NOT GO UP ANYMORE!");
    }else{
      currentElevatorLevel++; 
      setPointRotations = metersToRotations(elevatorHeights[currentElevatorLevel]);
    }
  }




  public void decreaseCurrentLevel(){
    if(currentElevatorLevel<=0){
      System.out.println("LEVEL IS CURRENTLY 1 AND CAN NOT GO DOWN ANYMORE!");
    }else{
      currentElevatorLevel--; 
      setPointRotations = metersToRotations(elevatorHeights[currentElevatorLevel]); 
    }
  }
  
  
  private void emergencyLimitSwitchLogic(){

    if(toplimitSwitch.get()){
      setPointRotations = elevatorHeights[elevatorHeights.length-1]; 
      System.out.println("[WARNING] Elevator height maxed out (Hint: this should have not happened)!");
    }
    if(!toplimitSwitch.get()){
      setPointRotations = elevatorHeights[0]; 
      System.out.println("[WARNING] Elevator height at 0 (Hint: Stop doing what u doing rn)!");
    }

  } 



  public ElevatorSubsystem() {

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    emergencyLimitSwitchLogic();
    leftElevatorMotor.setVoltage(pidControllerLeft.calculate(absoluteEncoder.getDistance(), setPointRotations) + feedforward.calculate(0.0));
    rightElevatorMotor.setVoltage(pidControllerRight.calculate(absoluteEncoder.getDistance(), setPointRotations) + feedforward.calculate(0.0));
  }
}
