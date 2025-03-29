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
   


  //LED patterns ===================================================                                           
  private LEDPattern coral = LEDPattern.solid(Color.kCoral);
  private LEDPattern canBusError = LEDPattern.solid(Color.kRed);
  private LEDPattern autoModeRunning = LEDPattern.solid(Color.kGold);
  private LEDPattern autoAlineRunning = LEDPattern.solid(Color.kPurple);
  private LEDPattern eleDroping = LEDPattern.solid(Color.kGreen); 
  

  //broken orange gradient 
  private LEDPattern brokenGradientBase = LEDPattern.gradient(GradientType.kDiscontinuous,Color.kBlack, Color.kBlack, orange,Color.kBlack, Color.kBlack, orange,Color.kBlack, Color.kBlack, orange);
  private LEDPattern idle = brokenGradientBase.scrollAtRelativeSpeed(Percent.per(Second).of(25)); 
  
  //ele going up 
  /*static LEDPattern elePattern = LEDPattern.LEDprogressMaskLayer();
  LEDPattern basePattern = LEDPattern.gradient(Color.kRed, Color.kBlue);
  LEDPattern progressPattern = basePattern.mask(progressMaskLayer(() -> elevator.getHeight() / elevator.maxHeight());*/
   

  //setting LED length, should only be done on startup 
  public LED_Subsystem() {
    elevatorLEDLeft.setLength(elevatorLEDLeft_Buffer.getLength()); 
    elevatorLEDRight.setLength(elevatorLEDRight_Buffer.getLength()); 

  }


  //Set LEDs to a pattern 
  private void setLED_Pattern(LEDPattern pattern){
    pattern.applyTo(elevatorLEDLeft_Buffer);
    elevatorLEDLeft.setData(elevatorLEDLeft_Buffer); 
    elevatorLEDLeft.start();

    pattern.applyTo(elevatorLEDLeft_Buffer);
    elevatorLEDLeft.setData(elevatorLEDLeft_Buffer); 
    elevatorLEDLeft.start(); 
  } 



  //hadleing toggling difrent LED patterns 

  private boolean isCANBUS_Error = false;

  public void triggerLED(String mode){
    if(isCANBUS_Error){
      setLED_Pattern(canBusError);
      return; 
    }
    



  }



  @Override
  public void periodic() {

  }
}
