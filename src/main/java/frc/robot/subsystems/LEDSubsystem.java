package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

public class LEDSubsystem extends SubsystemBase {

    private static final int LED_LENGTH = 300;
    private static final AddressableLED strip = new AddressableLED(8);
    private static final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_LENGTH);

    static {
        strip.setLength(buffer.getLength());
        strip.start();
    }

    private static final Color ORANGE = new Color(255, 40, 0);

    private static final LEDPattern ROBOT_NOT_READY = LEDPattern.solid(Color.kRed);
    private static final LEDPattern IDLE_ROUND_RUNNING = LEDPattern.solid(ORANGE);
    private static final LEDPattern AUTO_RUNNING = LEDPattern.solid(Color.kCoral);
    private static final LEDPattern AUTO_ALIGN_RUNNING = LEDPattern.solid(Color.kPurple);
    private static final LEDPattern ELE_DROPPING = LEDPattern.solid(Color.kGreen);
    private static final LEDPattern BROKEN_GRADIENT_BASE = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kBlack, ORANGE, Color.kBlack, Color.kBlack, ORANGE, Color.kBlack, Color.kBlack, ORANGE);
    private static final LEDPattern IDLE = BROKEN_GRADIENT_BASE.scrollAtRelativeSpeed(Percent.per(Second).of(25));
    private static final LEDPattern BASE_PATTERN = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, ORANGE);
    private static final LEDPattern ELE_UP = BASE_PATTERN.mask(LEDPattern.progressMaskLayer(() -> ElevatorSubsystem.currentHeight / 1.9));

    public enum LEDState {
        IDLE,
        ELEDROPPING,
        AUTOALIGNRUNNING,
        AUTORUNNING,
        IDLEROUNDRUNNING,
        ROBOTNOTREADY,
        ELEUP
    }

    private static LEDState state = LEDState.ROBOTNOTREADY;

    public LEDSubsystem() {

    }

    public static void setState(LEDState newState) {
        state = newState;
        switch (state) {
            case IDLE -> setPattern(IDLE);
            case ELEDROPPING -> setPattern(ELE_DROPPING);
            case AUTOALIGNRUNNING -> setPattern(AUTO_ALIGN_RUNNING);
            case AUTORUNNING -> setPattern(AUTO_RUNNING);
            case IDLEROUNDRUNNING -> setPattern(IDLE_ROUND_RUNNING);
            case ROBOTNOTREADY -> setPattern(ROBOT_NOT_READY);
            case ELEUP -> setPattern(ELE_UP);
        }
    }

    public static void setStateUntil(LEDState tempState, BooleanSupplier bool) {
        LEDState previous = state;
        tempCommand(tempState, new WaitUntilCommand(bool), previous).schedule();
    }

    public static void setStateFor(LEDState tempState, double seconds) {
        LEDState previous = state;
        tempCommand(tempState, new WaitCommand(seconds), previous).schedule();
    }

    private static void setPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
        strip.setData(buffer);
    }

    private static Command tempCommand(LEDState enter, Command wait, LEDState endState) {
        return new InstantCommand(() -> setState(enter)).andThen(wait).andThen(new InstantCommand(() -> setState(endState)));
    }

    @Override
    public void periodic() {
    }
}
