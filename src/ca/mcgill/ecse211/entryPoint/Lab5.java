package ca.mcgill.ecse211.entryPoint;

import java.io.FileNotFoundException;
import java.io.UnsupportedEncodingException;
import ca.mcgill.ecse211.colorClassification.ColorClassification;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.navigation.NavigationWithObstacle;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.WiFi.*;

/**
 * This class is the entry point of the whole project.
 * 
 * @author Michel Abdel Nour
 * @author Sandra Deng
 *
 */
public class Lab5 {

  public static EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final int TR = -1;  //0: red can, 1: green can, 2: yellow can, 3: blue can 
    
  public static EV3MediumRegulatedMotor canGrabbingMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));

  public static EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  private static Port sensorPort = LocalEV3.get().getPort("S1");
  private static SensorModes us = new EV3UltrasonicSensor(sensorPort);
  private static SampleProvider sampleProvider = us.getMode("Distance");

  //for correction
  private static final Port rightLightPort = LocalEV3.get().getPort("S3");
  private static final EV3ColorSensor rightLightMode = new EV3ColorSensor(rightLightPort);
//  private static SampleProvider rightLightSampleProvider = rightLightMode.getMode("Red");
  
  private static final Port leftLightPort = LocalEV3.get().getPort("S2");
  private static final EV3ColorSensor leftLightMode = new EV3ColorSensor(leftLightPort);
//  private static SampleProvider leftLightSampleProvider = leftLightMode.getMode("Red");
  
//  private static int LL_X = 2;
//  private static int LL_Y = 2;
//  private static int UR_X = 4;
//  private static int UR_Y = 3;
//  private static int corner = 0;
//  private static int SZ_LL_X = 6;
//  private static int SZ_LL_Y = 0;
//  private static int SZ_UR_X = 14;
//  private static int SZ_UR_Y = 5;
  
  public static double TRACK = 11.99;
  public static double WHEEL_RAD = 2.06;
  
  public static int  TURNING_SPEED = 70;
  public static final int  FORWARD_SPEED = 200;
  public static final double TILE_SIZE = 30.48;
  
  /*
   * Serves as Visual aid for now for calling Variable names
   * 
   *redTeam, greenTeam, redCorner, greenCorner, Red_LL_x, Red_LL_y, Red_UR_x, Red_UR_y, 
   *Green_LL_x, Green_LL_y, Green_UR_x, Green_UR_y, Island_LL_x, Island_LL_y, Island_UR_x, Island_UR_y, 
   *TNR_LL_x, TNR_LL_y, TNR_UR_x, TNR_UR_y, TNG_LL_x, TNG_LL_y, TNG_UR_x, TNG_UR_y
   *
   */
 
  
  private static LightSensorController leftLightSensor;
  private static LightSensorController rightLightSensor;
  
  public static int ACCELERATION = 300;

  /**
   * the entry point of the whole program. Run this class to start color detection at stationary position
   * @param str
   */
  public static void main(String str[]) { 
    
    //get data from the wifi class
//    WiFi.getData();
    
    
    
    //initialize and start the odometer
    Odometer odometer =null;
    try {
      odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      Thread odoThread = new Thread(odometer);
      odoThread.start();    
    } catch (OdometerExceptions e1) {
      // TODO Auto-generated catch block
      e1.printStackTrace();
    }

    //initialzise sensor controller
    leftLightSensor = new LightSensorController(leftLightPort);
    rightLightSensor = new LightSensorController(rightLightPort);
    
    //initialize OdometryCorrection
    OdometryCorrection odometryCorrection = new OdometryCorrection(odometer, leftMotor, rightMotor,leftLightSensor, rightLightSensor );
    
    //initialize and start localization
    try {
      UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(sampleProvider);
      usLocalizer.fallingEdge();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftLightSensor, rightLightSensor, odometryCorrection);
    lightLocalizer.startLocalize();
    
    //Testing data
    int LL_X = 1;
    int LL_Y = 1;
    int UR_X = 5;
    int UR_Y = 5;
 
    
    //localization is done, initialize navigation
    try {
      NavigationWithObstacle navigation = new NavigationWithObstacle(leftMotor, rightMotor, TRACK, WHEEL_RAD, LL_X, LL_Y, UR_X, UR_Y, sensorMotor, lcd, TR, sampleProvider, odometryCorrection);
      Thread navigationThread = new Thread(navigation);
      navigationThread.start();
    } catch (OdometerExceptions e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
  }
  
}
