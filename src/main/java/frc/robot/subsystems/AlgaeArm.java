package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANAssignments;
import frc.robot.util.ImprovedCanSpark;

public class AlgaeArm extends SubsystemBase {

    private double knockoutSpeed = .1;

    Encoder algaeEncoder = new Encoder(null, null); 
    private final SparkMax motor = new ImprovedCanSpark(CANAssignments.ALGAE_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, null, SparkBaseConfig.IdleMode.kBrake);

    
    
    public void triggerAlgaeArm(){

        


    }
        





        
    public AlgaeArm(){

    }
    
    public void stop(){

    }

    

}

