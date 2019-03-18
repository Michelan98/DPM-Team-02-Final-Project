package ca.mcgill.ecse211.entryPoint;

import java.io.FileNotFoundException;
import java.io.UnsupportedEncodingException;
import ca.mcgill.ecse211.colorClassification.ColorClassification;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Lab5 {

  public static EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final int TR = -1;  //0: red can, 1: green can, 2: yellow can, 3: blue can 
  
  /**
   * the entry point of the whole program. Run this class to start color detection at stationary position
   * @param str
   */
  public static void main(String str[]) {
    final ColorClassification colorClassification = new ColorClassification(sensorMotor, lcd, TR);
    //System.out.println("initialize");
    new Thread() {
      public void run() {
        try {
          colorClassification.colorClassify();
        } catch (InterruptedException e) {
          //e.printStackTrace();
        }
      }
      }.start();
      System.out.println("exit");   
  }
  
}
