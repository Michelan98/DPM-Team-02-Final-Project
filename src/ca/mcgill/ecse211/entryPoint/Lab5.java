package ca.mcgill.ecse211.entryPoint;

import java.io.FileNotFoundException;
import java.io.UnsupportedEncodingException;
import ca.mcgill.ecse211.colorClassification.ColorClassification;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
<<<<<<< HEAD
import ca.mcgill.ecse211.WiFi.*;
=======
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

>>>>>>> 72699868f7b5cc9179bffd7ce9712451dfc26c43

public class Lab5 {

	// declaring all variables for Wifi connection 
	public static int RedTeam, GreenTeam, redCorner, greenCorner, Red_UR_x, Red_LLx, Red_UR_y,
	Red_LL_y, Green_UR_x, Green_LL_x, Green_UR_y, Green_LL_y, Island_UR_x, Island_LL_x, Island_UR_y,
	Island_LL_y, TNR_LL_x, TNR_UR_x, TNR_LL_y, TNR_UR_y, TNG_LL_x, TNG_UR_x, TNG_LL_y, TNG_UR_y, 
	SZR_UR_x, SZR_UR_y, SZG_UR_x, SZG_UR_y, SZR_LL_y, SZR_LL_x, SZG_LL_x, SZG_LL_y; 	
	
	public static EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final int TR = -1;  //0: red can, 1: green can, 2: yellow can, 3: blue can 

	/**
	 * the entry point of the whole program. Run this class to start color detection at stationary position
	 * @param str
	 */
	public static void main(String str[]) {
		final ColorClassification colorClassification = new ColorClassification(sensorMotor, lcd, TR);
		
		WiFi.getData();
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

  public static EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final int TR = -1;  //0: red can, 1: green can, 2: yellow can, 3: blue can 
  
  
  public static EV3MediumRegulatedMotor canGrabbingMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));
  
  public static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  private static Port sensorPort = LocalEV3.get().getPort("S1");
  private static SensorModes us = new EV3UltrasonicSensor(sensorPort);
  private static SampleProvider sampleProvider = us.getMode("Distance");

  //for correction
  private static final Port rightLightPort = LocalEV3.get().getPort("S3");
  private static final SensorModes rightLightMode = new EV3ColorSensor(rightLightPort);
  private static SampleProvider rightLightSampleProvider = rightLightMode.getMode("Red");
  
  private static final Port leftLightPort = LocalEV3.get().getPort("S2");
  private static final SensorModes leftLightMode = new EV3ColorSensor(leftLightPort);
  private static SampleProvider leftLightSampleProvider = leftLightMode.getMode("Red");
  
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
