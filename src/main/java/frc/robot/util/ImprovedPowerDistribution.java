package frc.robot.util;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.HashMap;
import java.util.Map;

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

        watchdogCommand.schedule();
    }
}
