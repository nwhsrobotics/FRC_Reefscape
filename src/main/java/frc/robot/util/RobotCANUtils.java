package frc.robot.util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.CANAssignments;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

public final class RobotCANUtils {
    private RobotCANUtils() {
    }

    public enum MotorKind {
        NEO30AMP(30), NEO80AMP(80), NEO550(20), VORTEX(80);
        final int limit;

        MotorKind(int l) {
            limit = l;
        }
    }

    private static void applySettings(SparkBaseConfig cfg, MotorKind kind, IdleMode mode, double voltage) {
        cfg.idleMode(mode);
        cfg.smartCurrentLimit(kind.limit);
        if (voltage > 0) cfg.voltageCompensation(voltage);
    }

    private static void finishConfigure(SparkMax m, SparkMaxConfig cfg) {
        boolean p = DriverStation.isFMSAttached();
        m.configure(cfg, ResetMode.kResetSafeParameters, p ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    }

    private static void finishConfigure(SparkFlex f, SparkFlexConfig cfg) {
        boolean p = DriverStation.isFMSAttached();
        f.configure(cfg, ResetMode.kResetSafeParameters, p ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    }

    public static class CANSparkMaxController extends SparkMax {
        public CANSparkMaxController(int id, MotorKind kind, SparkMaxConfig cfg, IdleMode mode, double voltage) {
            super(id, MotorType.kBrushless);
            clearFaults();
            applySettings(cfg, kind, mode, voltage);
            finishConfigure(this, cfg);
        }

        public CANSparkMaxController(int id, MotorKind kind, IdleMode mode) {
            this(id, kind, new SparkMaxConfig(), mode, 0);
        }

        public CANSparkMaxController(int id, MotorKind kind, IdleMode mode, double voltage) {
            this(id, kind, new SparkMaxConfig(), mode, voltage);
        }


        public CANSparkMaxController(int id, MotorKind kind, SparkMaxConfig cfg,
                                     IdleMode mode, double p, double i, double d,
                                     double maxVel, double maxAccel, double err) {
            super(id, MotorType.kBrushless);
            clearFaults();
            applySettings(cfg, kind, mode, 0);
            cfg.closedLoop
                    .p(p).i(i).d(d)
                    .maxMotion
                    .maxVelocity(maxVel)
                    .maxAcceleration(maxAccel)
                    .allowedClosedLoopError(err);
            finishConfigure(this, cfg);
        }

        public CANSparkMaxController(int id, MotorKind kind, SparkMaxConfig cfg,
                                     IdleMode mode, double p, double i, double d,
                                     double maxVel, double maxAccel, double err, double voltage) {
            super(id, MotorType.kBrushless);
            clearFaults();
            applySettings(cfg, kind, mode, voltage);
            cfg.closedLoop
                    .p(p).i(i).d(d)
                    .maxMotion
                    .maxVelocity(maxVel)
                    .maxAcceleration(maxAccel)
                    .allowedClosedLoopError(err);
            finishConfigure(this, cfg);
        }
    }

    public static class CANSparkFlexController extends SparkFlex {
        public CANSparkFlexController(int id, MotorKind kind, SparkFlexConfig cfg, IdleMode mode) {
            super(id, MotorType.kBrushless);
            clearFaults();
            applySettings(cfg, kind, mode, 0);
            finishConfigure(this, cfg);
        }
    }

    public static class PowerDistributionManager extends PowerDistribution {
        private static final long COOLDOWN_MS = 5_000;
        private final Map<Integer, Boolean> states = new HashMap<>();
        private long lastLog = 0;

        public PowerDistributionManager(int id, ModuleType type) {
            super(id, type);
            clearStickyFaults();
            PowerDistributionFaults f = getFaults();
            for (int i = 0; i < getNumChannels(); i++) {
                states.put(i, f.getBreakerFault(i));
            }
            checkCANAssignments();
            new RunCommand(this::poll).schedule();
        }

        private void poll() {
            long now = System.currentTimeMillis();
            if (now - lastLog < COOLDOWN_MS) return;
            lastLog = now;
            PowerDistributionFaults f = getFaults();
            for (int i = 0; i < getNumChannels(); i++) {
                boolean prev = states.get(i), curr = f.getBreakerFault(i);
                if (prev != curr) {
                    System.out.println("Channel " + i + " fault " + prev + "→" + curr);
                    states.put(i, curr);
                }
            }
        }

        public static boolean checkCANAssignments() {
            boolean dup = false;
            Map<Integer, String> seen = new HashMap<>();
            for (Field fld : CANAssignments.class.getFields()) {
                try {
                    fld.setAccessible(true);
                    int id = fld.getInt(null);
                    if (seen.put(id, fld.getName()) != null) {
                        System.out.println("Duplicate CAN assignment on " + id +
                                " for " + fld.getName() + " already used by " + seen.get(id) + "!");
                        dup = true;
                    }
                } catch (Exception e) {
                    System.out.println("Checking CAN assignment for " + fld.getName() + " failed!");
                }
            }
            Logger.recordOutput("canassignmentsok", !dup);
            return dup;
        }
    }
}
