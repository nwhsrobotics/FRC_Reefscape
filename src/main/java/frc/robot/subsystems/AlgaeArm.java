
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANAssignments;
import frc.robot.util.ImprovedCanSpark;

public class AlgaeArm extends SubsystemBase {

    
    //private final SparkBaseConfig alageConfig = new SparkBaseConfig();

    private final SparkBaseConfig algaeConfig = new SparkMaxConfig();
    

    private final SparkMax motor = new ImprovedCanSpark(CANAssignments.ALGAE_MOTOR_ID, ImprovedCanSpark.MotorKind.NEO550, algaeConfig, SparkBaseConfig.IdleMode.kBrake, 0.1, 0.0, 0.0);
    private final RelativeEncoder  algaeEncoder = motor.getEncoder();
    SparkClosedLoopController AlgaeController = motor.getClosedLoopController();
    private final double Target = 60.0;
    private double Algaerotations = 0.0;

    

    

    
    
    private double degreesToMotorRotation(double degrees) {
        return (degrees / 360.0) * 98.7;
    }
   
   
    @Override
    
    public void periodic(){
    
    AlgaeController.setReference(-Algaerotations, ControlType.kMAXMotionPositionControl);
    
    //System.out.println(motor.getEncoder().getPosition());
    // System.out.println(Algaerotations);
    


}
    public void knockoutAlgae(){
        Algaerotations  = degreesToMotorRotation(Target);
        

    }



    public void Homeposition(){
        Algaerotations = degreesToMotorRotation(10.0);


    }

      
    


}

