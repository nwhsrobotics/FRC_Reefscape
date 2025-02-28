package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
    //public CANcoder rotatingArmCaNcoder  =  new CANcoder(Something);
    private CANcoder rotatingArmCaNcoder;
    private SparkBaseConfig RotatingArmMotorConfig;
    
    private final SparkMax RotatingArmMotor = new SparkMax(29, MotorType.kBrushless);
    public void armMovement() {
        
        rotatingArmCaNcoder = new CANcoder(30);
        RotatingArmMotorConfig.encoder.positionConversionFactor(1); // I don't know what to do with this but its part of an encoder so I added it
        RotatingArmMotorConfig.encoder.velocityConversionFactor(1); // I don't know what to do with this but its part of an encoder so I added it
    }
    
    
       public void MovingAlgaeMotor(double AlgaeSpeed){
        RotatingArmMotor.set(AlgaeSpeed); //Change as needed

       }
        
        
    
    public void stop(){
        RotatingArmMotor.set(0);

    }

    

}

