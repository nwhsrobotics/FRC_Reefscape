package frc.robot.legacy;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class RobotErrorHandler {
    private static int errorCount = 0;

    public static void initialize() {
        try {
            Thread.setDefaultUncaughtExceptionHandler((thread, throwable) -> {
                handleException(throwable, "Thread: " + thread.getName());
            });
        } catch (Throwable t) {
            t.printStackTrace();
        }
    }

    private static void handleException(Throwable throwable, String source) {
        try {
            errorCount++;
            String errorMessage = "[ERROR #" + errorCount + "] in " + source + ": " + throwable.getMessage();
            System.err.println(errorMessage);
            throwable.printStackTrace();
            DriverStation.reportError(errorMessage, throwable.getStackTrace());
            try {
                Logger.recordOutput("LastError", errorMessage);
                Logger.recordOutput("ErrorCount", errorCount);
            } catch (Throwable ignored) {
            }
        } catch (Throwable t) {
            System.err.println("ERROR IN ERROR HANDLER: " + t.getMessage());
            t.printStackTrace();
        }
    }
}
