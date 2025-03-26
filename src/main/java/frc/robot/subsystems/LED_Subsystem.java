package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED_Subsystem extends SubsystemBase {
  
  //LED strip lengths 
  private int elevatorLED1Length = 80;
  //private int elevatorLED2Length = 0;
  //private int baseLEDLength = 0;

  //LED objects 
  private AddressableLED elevatorLED1 = new AddressableLED(1);
  //private AddressableLED elevatorLED2 = new AddressableLED(2);
  //private AddressableLED baseLED = new AddressableLED(3);

  //LED buffers 
  private AddressableLEDBuffer elevatorLED1_Buffer = new AddressableLEDBuffer(elevatorLED1Length);
  //private AddressableLEDBuffer elevatorLED2_Buffer = new AddressableLEDBuffer(elevatorLED2Length);
  //private AddressableLEDBuffer baseLED_Buffer = new AddressableLEDBuffer(baseLEDLength);

  //List of LEDs
  //private AddressableLED[] LED_List = {elevatorLED1, elevatorLED2};


  //LED patterns =====================================================  
  private Color orangeRGB = new Color(255,40,0);  
  private LEDPattern red = LEDPattern.solid(Color.kRed);
  private LEDPattern orange = LEDPattern.solid(Color.kOrange);
  private LEDPattern yellow = LEDPattern.solid(Color.kYellow);
  private LEDPattern green = LEDPattern.solid(Color.kGreen);
  private LEDPattern blue = LEDPattern.solid(Color.kBlue);
  private LEDPattern violet  = LEDPattern.solid(Color.kViolet);
  private LEDPattern white  = LEDPattern.solid(Color.kWhite); 
  private LEDPattern customOrange = LEDPattern.solid(orangeRGB);
    //can set the rainbow to only spacific colors, Needs to have correct LED density 
  //private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
   //private static final Distance kLedSpacing = Meters.of(1 / 80.0);
   //private final LEDPattern m_scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
  Distance ledSpacing = Meters.of(1 / 120.0);
  LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous,Color.kBlack, Color.kBlack, orangeRGB,Color.kBlack, Color.kBlack, orangeRGB,Color.kBlack, Color.kBlack, orangeRGB);
  LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
  LEDPattern absolute = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);


   

  //setting LED length, should only be done on startup 
  public LED_Subsystem() {
 
    elevatorLED1.setLength(elevatorLED1_Buffer.getLength()); 
    //elevatorLED2.setLength(elevatorLED2_Buffer.getLength()); 
    //baseLED.setLength(baseLED_Buffer.getLength()); 

  }



  //LED presets ======================================================

  public void LED_Pattern_CANBUS_Error(){
    pattern.applyTo(elevatorLED1_Buffer);
    elevatorLED1.setData(elevatorLED1_Buffer);
    elevatorLED1.start();

    //red.applyTo(elevatorLED2_Buffer);
    //elevatorLED2.setData(elevatorLED2_Buffer);   
    
    //red.applyTo(baseLED_Buffer);
    //baseLED.setData(baseLED_Buffer); 
  } 

  public void LED_Pattern_Rainbow(){
    orange.applyTo(elevatorLED1_Buffer);
    elevatorLED1.setData(elevatorLED1_Buffer);

    //red.applyTo(elevatorLED2_Buffer);
    //elevatorLED2.setData(elevatorLED2_Buffer);   
    
    //red.applyTo(baseLED_Buffer);
    //baseLED.setData(baseLED_Buffer); 
  } 



  //Things to trigger LEDs ===========================================

  private void runLEDs(){
    /*for(AddressableLED LED: LED_List){
      LED.start();
    }*/
  }

  @Override
  public void periodic() {
    LED_Pattern_CANBUS_Error();
  }
}
