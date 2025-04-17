package frc.robot.util;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.CANAssignments;

import java.util.HashMap;
import java.util.Map;
import java.lang.reflect.Field;

import org.littletonrobotics.junction.Logger;

/**
 * Manages the PDP/PDH of the robot, along with providing logging and retrieval methods for distribution data outputs.
 */
public class ImprovedPowerDistribution extends PowerDistribution {
    private final Map<Integer, Boolean> channelTracker = new HashMap<>();
    private static final long COOLDOWN = 5000;
    private long lastTime = 0;

    private final RunCommand watchdogCommand = new RunCommand(() -> {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTime >= COOLDOWN) {
            lastTime = currentTime;
            PowerDistributionFaults activeFaults = getFaults();
            for (int i = 0; i < getNumChannels(); i++) {
                boolean before = channelTracker.getOrDefault(i, false);
                boolean now = activeFaults.getBreakerFault(i);
                if (before != now) {
                    System.out.println("Achtung! Detected new change in PDP/PDH breaker fault for channel " + i + ", whose state went from " + before + " to " + now + ".");
                    channelTracker.put(i, now);
                }
            }
        }
    });

    public ImprovedPowerDistribution(int id, ModuleType type) {
        super(id, type);
        clearStickyFaults();

        PowerDistributionFaults activeFaults = getFaults();
        for (int i = 0; i < getNumChannels(); i++) {
            channelTracker.put(i, activeFaults.getBreakerFault(i));
        }

        checkAssignments();
        watchdogCommand.schedule();
    }

    /**
     * Check for duplicate CAN assignments,
     * declared under the class this method is defined in.
     * <p>
     * If an assignment cannot be loaded,
     * or a duplicate assignment is found,
     * a message will be printed in the console.
     *
     * @return - true if duplicate assignment is found, otherwise false.
     */
    public static boolean checkAssignments() {
        Field[] constants = CANAssignments.class.getFields();
        HashMap<Integer, String> tracker = new HashMap<>();
        boolean dupeFound = false;

        for (Field field : constants) {
            field.setAccessible(true);

            int workingId;
            try {
                workingId = field.getInt(CANAssignments.class);
            } catch (IllegalArgumentException | IllegalAccessException e) {
                System.out.println("Achtung! Checking CAN assignment for " + field.getName() + " failed!");
                continue;
            }

            if (tracker.put(workingId, field.getName()) != null) {  // this also adds the field to the tracker.
                System.out.println("Fehler! Duplicate CAN assignment on " +
                        workingId +
                        " for " +
                        field.getName() +
                        " already used by " +
                        tracker.get(workingId) +
                        "!");

                dupeFound = true;
            }
        }

        Logger.recordOutput("canassignmentsok", !dupeFound);

        return dupeFound;
    }
}
