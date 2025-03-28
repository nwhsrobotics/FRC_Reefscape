package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED_Subsystem extends SubsystemBase {
  
  //LED strip lengths 
  private int elevatorLEDLength = 80;

  //LED objects 
  private AddressableLED elevatorLEDLeft = new AddressableLED(1);
  private AddressableLED elevatorLEDRight = new AddressableLED(2);

  //LED buffers 
  private AddressableLEDBuffer elevatorLEDLeft_Buffer = new AddressableLEDBuffer(elevatorLEDLength);
  private AddressableLEDBuffer elevatorLEDRight_Buffer = new AddressableLEDBuffer(elevatorLEDLength);
  


  private Distance ledSpacing = Meters.of(1 / 120.0);                   

  //LED colors =====================================================  
  private Color orange = new Color(255,40,0);  
   


  //solid color LED patterns 
  //coral pattern(if coral is in outtake)
  private LEDPattern coral = LEDPattern.solid(Color.kCoral);
  //
  //broken orange gradient 
  private LEDPattern brokenGradientBase = LEDPattern.gradient(GradientType.kDiscontinuous,Color.kBlack, Color.kBlack, orange,Color.kBlack, Color.kBlack, orange,Color.kBlack, Color.kBlack, orange);
  private LEDPattern idle = brokenGradientBase.scrollAtRelativeSpeed(Percent.per(Second).of(25));
  private LEDPattern absolute = brokenGradientBase.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);
  //breathe pattern
  private LEDPattern base = LEDPattern.discontinuousGradient(Color.orange);
  private LEDPattern pattern = base.breathe(Seconds.of(2));
  //progress mask (elevator pattern)
  LEDPattern pattern = LEDPattern.progressMaskLayer(() -> m_elevator.getHeight() / m_elevator.getMaxHeight());



   

  //setting LED length, should only be done on startup 
  public LED_Subsystem() {
 
    elevatorLEDLeft.setLength(elevatorLEDLeft_Buffer.getLength()); 
    elevatorLEDRight.setLength(elevatorLEDRight_Buffer.getLength()); 
    

  }



  //LED presets ======================================================

  public void LED_Pattern_Idle(){
    idle.applyTo(elevatorLEDLeft_Buffer);
    elevatorLEDLeft.setData(elevatorLEDLeft_Buffer); 
    elevatorLEDLeft.start();

    idle.applyTo(elevatorLEDLeft_Buffer);
    elevatorLEDLeft.setData(elevatorLEDLeft_Buffer); 
    elevatorLEDLeft.start(); 
  } 




  //Things to trigger LEDs ===========================================


  @Override
  public void periodic() {

  }
}
