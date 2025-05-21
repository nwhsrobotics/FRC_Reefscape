package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANAssignments;
import frc.robot.util.RobotCANUtils.CANSparkMaxController;
import frc.robot.util.RobotCANUtils.MotorKind;

public class AlgaeArm extends SubsystemBase {

    private final SparkMaxConfig algaeConfig = new SparkMaxConfig();
    private final SparkMax motor = new CANSparkMaxController(CANAssignments.ALGAE_MOTOR_ID, MotorKind.NEO550, algaeConfig, IdleMode.kBrake, 0.1, 0.0, 0.0, 11844.0, 11844.0, 0.6);
    SparkClosedLoopController AlgaeController = motor.getClosedLoopController();
    private final double Target = 75.0;
    private double Algaerotations = 0.0;


    private double degreesToMotorRotation(double degrees) {
        return (degrees / 360.0) * 98.7;
    }


    @Override

    public void periodic() {
        AlgaeController.setReference(-Algaerotations, ControlType.kMAXMotionPositionControl);
    }

    public void knockoutAlgae() {
        Algaerotations = degreesToMotorRotation(Target);


    }


    public void Homeposition() {
        Algaerotations = degreesToMotorRotation(10.0);
    }


}

