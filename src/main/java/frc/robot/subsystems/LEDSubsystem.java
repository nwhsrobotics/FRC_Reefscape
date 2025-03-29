package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class LEDSubsystem extends SubsystemBase {

    //LED strip lengths
    private final int elevatorLEDLength = 80;

    //LED objects
    private final AddressableLED elevatorLEDLeft = new AddressableLED(1);
    private final AddressableLED elevatorLEDRight = new AddressableLED(2);

    //LED buffers
    private final AddressableLEDBuffer elevatorLEDLeft_Buffer = new AddressableLEDBuffer(elevatorLEDLength);
    private final AddressableLEDBuffer elevatorLEDRight_Buffer = new AddressableLEDBuffer(elevatorLEDLength);


    private final Distance ledSpacing = Meters.of(1 / 120.0);

    //LED colors =====================================================
    private final Color orange = new Color(255, 40, 0);


    //LED patterns ===================================================
    private final LEDPattern coral = LEDPattern.solid(Color.kCoral);
    private final LEDPattern canBusError = LEDPattern.solid(Color.kRed);
    private final LEDPattern autoModeRunning = LEDPattern.solid(Color.kGold);
    private final LEDPattern autoAlineRunning = LEDPattern.solid(Color.kPurple);
    private final LEDPattern eleDroping = LEDPattern.solid(Color.kGreen);


    //broken orange gradient
    private final LEDPattern brokenGradientBase = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kBlack, orange, Color.kBlack, Color.kBlack, orange, Color.kBlack, Color.kBlack, orange);
    private final LEDPattern idle = brokenGradientBase.scrollAtRelativeSpeed(Percent.per(Second).of(25));

    //ele going up
  /*static LEDPattern elePattern = LEDPattern.LEDprogressMaskLayer();
  LEDPattern basePattern = LEDPattern.gradient(Color.kRed, Color.kBlue);
  LEDPattern progressPattern = basePattern.mask(progressMaskLayer(() -> elevator.getHeight() / elevator.maxHeight());*/


    //setting LED length, should only be done on startup
    public LEDSubsystem() {
        elevatorLEDLeft.setLength(elevatorLEDLeft_Buffer.getLength());
        elevatorLEDRight.setLength(elevatorLEDRight_Buffer.getLength());

    }


    //Set LEDs to a pattern
    private void setLED_Pattern(LEDPattern pattern) {
        pattern.applyTo(elevatorLEDLeft_Buffer);
        elevatorLEDLeft.setData(elevatorLEDLeft_Buffer);
        elevatorLEDLeft.start();

        pattern.applyTo(elevatorLEDLeft_Buffer);
        elevatorLEDLeft.setData(elevatorLEDLeft_Buffer);
        elevatorLEDLeft.start();
    }


    //hadleing toggling difrent LED patterns

    private final boolean isCANBUS_Error = false;

    public void triggerLED(LEDPattern ledPattern) {
        if (isCANBUS_Error) {
            setLED_Pattern(canBusError);
        }


    }


    @Override
    public void periodic() {

    }
}
