package frc.robot.util;

import com.revrobotics.sim.SparkFlexExternalEncoderSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;

public class ImprovedCanSparkFlex extends SparkFlex {

    public enum MotorKind {
        NEO550,
        NEO,
        VORTEX
    }

    public ImprovedCanSparkFlex(int id, MotorKind motor, IdleMode mode, double volComp) {
        super(id, MotorType.kBrushless);
        SparkFlex
        restoreFactoryDefaults();
        clearFaults();
        setIdleMode(mode);
        setVoltage(volComp);
        switch (motor) {
            case NEO -> setSmartCurrentLimit(80);
            case NEO550 -> setSmartCurrentLimit(20);
            case VORTEX -> setSmartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            burnFlash();
        }
    }

    public ImprovedCanSparkFlex(int id, frc.robot.util.ImprovedCanSpark.MotorKind neo, IdleMode mode) {
        super(id, MotorType.kBrushless);
        restoreFactoryDefaults();
        clearFaults();
        setIdleMode(mode);
        switch (neo) {
            case NEO -> setSmartCurrentLimit(80);
            case NEO550 -> setSmartCurrentLimit(20);
            case VORTEX -> setSmartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            burnFlash();
        }
    }
}
