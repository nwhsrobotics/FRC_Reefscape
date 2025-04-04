package frc.robot.util;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;

public class ImprovedCanSpark extends SparkMax {

    public enum MotorKind {
        NEO550,
        NEO,
        VORTEX
    }

    public ImprovedCanSpark(int id, MotorKind motor, SparkBaseConfig config, IdleMode mode, double volComp) {
        super(id, MotorType.kBrushless);
        clearFaults();
        config.idleMode(mode);
        setVoltage(volComp);
        //config.voltageCompensation(volComp)
        switch (motor) {
            case NEO -> config.smartCurrentLimit(80);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }


    public ImprovedCanSpark(int id, SparkBaseConfig config, MotorKind motor, IdleMode mode, double P, double I, double D, double maxVel, double maxAccel, double allowedError) {
        super(id, MotorType.kBrushless);
        clearFaults();

        config.idleMode(mode);
        switch (motor) {
            case NEO -> config.smartCurrentLimit(30);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }
        config.closedLoop
                .p(P)
                .i(I)
                .d(D)
                .maxMotion
                .maxVelocity(maxVel)
                .maxAcceleration(maxAccel).
                allowedClosedLoopError(allowedError);
        config.voltageCompensation(11);
        //setVoltage(11);
        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }


    public ImprovedCanSpark(int id, MotorKind motor, SparkBaseConfig config, IdleMode mode) {
        super(id, MotorType.kBrushless);
        clearFaults();
        config.idleMode(mode);
        switch (motor) {
            case NEO -> config.smartCurrentLimit(80);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }
    public ImprovedCanSpark(int id, MotorKind motor, SparkBaseConfig config, IdleMode mode, double p, double i, double d) {
        super(id, MotorType.kBrushless);
        clearFaults();
        config.idleMode(mode);
        switch (motor) {
            case NEO -> config.smartCurrentLimit(80);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }

        config
        .closedLoop
        .p(p)
        .i(i)
        .d(d)
        .maxMotion
        .maxVelocity(11844.0)
        .maxAcceleration(11844.0)
        .allowedClosedLoopError(0.6);

        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public ImprovedCanSpark(int id, MotorKind motor, IdleMode mode, double volComp) {
        super(id, MotorType.kBrushless);
        clearFaults();
        SparkMaxConfig config = new SparkMaxConfig();
        setVoltage(volComp);
        //config.voltageCompensation(volComp)
        switch (motor) {
            case NEO -> config.smartCurrentLimit(80);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public ImprovedCanSpark(int id, MotorKind motor, IdleMode mode) {
        super(id, MotorType.kBrushless);
        clearFaults();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(mode);
        switch (motor) {
            case NEO -> config.smartCurrentLimit(80);
            case NEO550 -> config.smartCurrentLimit(20);
            case VORTEX -> config.smartCurrentLimit(80);
        }
        if (DriverStation.isFMSAttached()) {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        } else {
            configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }
}
