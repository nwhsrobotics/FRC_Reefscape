package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class LEDSubsystem extends SubsystemBase {

    //LED strip lengths
    private final int elevatorLEDLengthLEFT = 10;
    private final int elevatorLEDLengthRIGHT = 10;

    //LED objects
    private final AddressableLED elevatorLEDLeft = new AddressableLED(8);
    private final AddressableLED elevatorLEDRight = new AddressableLED(7); 

    //LED buffers
    private final AddressableLEDBuffer elevatorLEDLeft_Buffer = new AddressableLEDBuffer(elevatorLEDLengthLEFT);
    private final AddressableLEDBuffer elevatorLEDRight_Buffer = new AddressableLEDBuffer(elevatorLEDLengthRIGHT);




    //LED colors =====================================================
    private final Color orange = new Color(255, 40, 0);


    //LED patterns ===================================================
    private final LEDPattern robotNotReady = LEDPattern.solid(Color.kRed); 
    private final LEDPattern idleRoundRunning = LEDPattern.solid(orange); 
    private final LEDPattern autoRunning = LEDPattern.solid(Color.kCoral); 
    private final LEDPattern autoAlineRunning = LEDPattern.solid(Color.kPurple);  
    private final LEDPattern eleDroping = LEDPattern.solid(Color.kGreen); 


    //broken orange gradient
    private final LEDPattern brokenGradientBase = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kBlack, orange, Color.kBlack, Color.kBlack, orange, Color.kBlack, Color.kBlack, orange);
    private final LEDPattern idle = brokenGradientBase.scrollAtRelativeSpeed(Percent.per(Second).of(25));    
    
    //EleUp 
    private final LEDPattern bacePattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, orange);
    private final LEDPattern eleUp = bacePattern.mask(LEDPattern.progressMaskLayer(()-> ElevatorSubsystem.currentHeight/1.9));


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

        pattern.applyTo(elevatorLEDRight_Buffer);
        elevatorLEDRight.setData(elevatorLEDRight_Buffer);
        elevatorLEDRight.start();
        
    }


    //hadleing toggling difrent LED patterns

    public static LEDState state = LEDState.ROBOTNOTREADY;       

    public static enum LEDState {
        
        IDLE,
        
        ELEDROPING,
        
        AUTOALINERUNNING, 
        
        AUTORUNNING,    

        IDLEROUNDRUNNING,

        ROBOTNOTREADY,

        ELEUP,
    }


    @Override
    public void periodic() {
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
}
