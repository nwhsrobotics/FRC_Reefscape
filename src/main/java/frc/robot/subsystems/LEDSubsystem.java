package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.function.BooleanSupplier;

public class LEDSubsystem extends SubsystemBase {

    //LED strip lengths
    private static final int elevatorLEDLengthLEFT = 20;
    //private final int elevatorLEDLengthRIGHT = 10;

    //LED objects
    private static final AddressableLED elevatorLEDLeft = new AddressableLED(9);
    //private final AddressableLED elevatorLEDRight = new AddressableLED(7); 

    //LED buffers
    private static final AddressableLEDBuffer elevatorLEDLeft_Buffer = new AddressableLEDBuffer(elevatorLEDLengthLEFT);
    //private final AddressableLEDBuffer elevatorLEDRight_Buffer = new AddressableLEDBuffer(elevatorLEDLengthRIGHT);

    static {
        elevatorLEDLeft.setLength(elevatorLEDLeft_Buffer.getLength());
    }


    //LED colors =====================================================
    private static final Color orange = new Color(255, 40, 0);


    //LED patterns ===================================================
    private static final LEDPattern robotNotReady = LEDPattern.solid(Color.kRed);
    private static final LEDPattern idleRoundRunning = LEDPattern.solid(orange);
    private static final LEDPattern autoRunning = LEDPattern.solid(Color.kCoral);
    private static final LEDPattern autoAlineRunning = LEDPattern.solid(Color.kPurple);
    private static final LEDPattern eleDroping = LEDPattern.solid(Color.kGreen);


    //broken orange gradient
    private static final LEDPattern brokenGradientBase = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kBlack, orange, Color.kBlack, Color.kBlack, orange, Color.kBlack, Color.kBlack, orange);
    private static final LEDPattern idle = brokenGradientBase.scrollAtRelativeSpeed(Percent.per(Second).of(25));

    //EleUp 
    private static final LEDPattern bacePattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, orange);
    private static final LEDPattern eleUp = bacePattern.mask(LEDPattern.progressMaskLayer(() -> ElevatorSubsystem.currentHeight / 1.9));


    //setting LED length, should only be done on startup
    public LEDSubsystem() {
        //elevatorLEDRight.setLength(elevatorLEDRight_Buffer.getLength());
        /*﻿﻿﻿﻿﻿﻿﻿﻿ERROR ﻿﻿ 1 ﻿﻿ Unhandled exception: edu.wpi.first.hal.util.UncleanStatusException:  Code: -1028. Data length must be less than or equal to 1. 20 was requested ﻿﻿ frc.robot.subsystems.LEDSubsystem.setLED_Pattern(LEDSubsystem.java:66) ﻿﻿﻿
﻿﻿﻿﻿﻿﻿ Error at frc.robot.subsystems.LEDSubsystem.setLED_Pattern(LEDSubsystem.java:66): Unhandled exception: edu.wpi.first.hal.util.UncleanStatusException:  Code: -1028. Data length must be less than or equal to 1. 20 was requested ﻿
﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.hal.AddressableLEDJNI.setData(Native Method) ﻿
﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.AddressableLED.setData(AddressableLED.java:67) ﻿
﻿﻿﻿﻿﻿﻿ 	at frc.robot.subsystems.LEDSubsystem.setLED_Pattern(LEDSubsystem.java:66) ﻿
﻿﻿﻿﻿﻿﻿ 	at frc.robot.subsystems.LEDSubsystem.setState(LEDSubsystem.java:111) ﻿
﻿﻿﻿﻿﻿﻿ 	at frc.robot.Robot.disabledPeriodic(Robot.java:126) ﻿
﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.IterativeRobotBase.loopFunc(IterativeRobotBase.java:377) ﻿
﻿﻿﻿﻿﻿﻿ 	at org.littletonrobotics.junction.LoggedRobot.startCompetition(LoggedRobot.java:117) ﻿
﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:419) ﻿
﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.RobotBase.startRobot(RobotBase.java:510) ﻿
﻿﻿﻿﻿﻿﻿ 	at frc.robot.Main.main(Main.java:24) ﻿
 */

    }


    //Set LEDs to a pattern
    private static void setLED_Pattern(LEDPattern pattern) {
        pattern.applyTo(elevatorLEDLeft_Buffer);
        elevatorLEDLeft.setData(elevatorLEDLeft_Buffer);
        elevatorLEDLeft.start();

        //pattern.applyTo(elevatorLEDRight_Buffer);
        //elevatorLEDRight.setData(elevatorLEDRight_Buffer);
        //elevatorLEDRight.start();

    }


    //hadleing toggling difrent LED patterns

    public static LEDState state = LEDState.ROBOTNOTREADY;

    public enum LEDState {

        IDLE,

        ELEDROPING,

        AUTOALINERUNNING,

        AUTORUNNING,

        IDLEROUNDRUNNING,

        ROBOTNOTREADY,

        ELEUP,
    }

    public static void setStateWaitUntilBoolean(LEDState ledState, BooleanSupplier bool){
        LEDState temp = state;
        new InstantCommand(() -> setState(ledState)).andThen(new WaitUntilCommand(bool)).andThen(new InstantCommand(() -> setState(temp)));
    }

    public static void setStateWaitUntilTime(LEDState ledState, double seconds){
        LEDState temp = state;
        new InstantCommand(() -> setState(ledState)).andThen(new WaitCommand(seconds)).andThen(new InstantCommand(() -> setState(temp)));
    }

    public static void setState(LEDState ledState){
        state = ledState;
        switch (state) {
            case IDLE:
                setLED_Pattern(idle);
                break;
            case ELEDROPING:
                setLED_Pattern(eleDroping);
                break;
            case AUTOALINERUNNING:
                setLED_Pattern(autoAlineRunning);
                break;
            case AUTORUNNING:
                setLED_Pattern(autoRunning);
                break;
            case IDLEROUNDRUNNING:
                setLED_Pattern(idleRoundRunning);
                break;
            case ROBOTNOTREADY:
                setLED_Pattern(robotNotReady);
                break;
            case ELEUP:
                setLED_Pattern(eleUp);
            default:
                break;
        }
    }


    @Override
    public void periodic() {

    }
}
