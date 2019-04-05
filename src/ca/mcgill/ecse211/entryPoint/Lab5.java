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
import lejos.hardware.Sound;
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

  public static EV3LargeRegulatedMotor sensorMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final int TR = -1; // 0: red can, 1: green can, 2: yellow can, 3: blue can

  public static EV3MediumRegulatedMotor canGrabbingMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));

  public static EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  private static Port sensorPort = LocalEV3.get().getPort("S4");
  private static SensorModes us = new EV3UltrasonicSensor(sensorPort);
  private static SampleProvider sampleProvider = us.getMode("Distance");

  // for correction
  private static final Port rightLightPort = LocalEV3.get().getPort("S3");
  private static final EV3ColorSensor rightLightMode = new EV3ColorSensor(rightLightPort);
  // private static SampleProvider rightLightSampleProvider = rightLightMode.getMode("Red");

  private static final Port leftLightPort = LocalEV3.get().getPort("S2");
  private static final EV3ColorSensor leftLightMode = new EV3ColorSensor(leftLightPort);
  // private static SampleProvider leftLightSampleProvider = leftLightMode.getMode("Red");


  public static double TRACK = 11.99;
  public static double WHEEL_RAD = 2.06;
  // Time limit for Final demo is 300 seconds (5 min)
  public static final long TIME_LIMIT = 300;
  // time threshold for going back to the starting point
  private static final float END_TIME = 45f;

  public static int TURNING_SPEED = 100;
  public static final int FORWARD_SPEED = 250;
  public static final double TILE_SIZE = 30.48;
  
  public static final int BOARD_X = 15;
  public static final int BOARD_Y = 9;

  // start time for the timer
  private static long startTime;


  //
  /*
   * Serves as Visual aid for now for calling Variable names
   * 
   * redTeam, greenTeam, redCorner, greenCorner, Red_LL_x, Red_LL_y, Red_UR_x, Red_UR_y, Green_LL_x,
   * Green_LL_y, Green_UR_x, Green_UR_y, Island_LL_x, Island_LL_y, Island_UR_x, Island_UR_y,
   * TNR_LL_x, TNR_LL_y, TNR_UR_x, TNR_UR_y, TNG_LL_x, TNG_LL_y, TNG_UR_x, TNG_UR_y
   *
   */


  private static final LightSensorController leftLightSensor =
      new LightSensorController(leftLightMode);
  private static final LightSensorController rightLightSensor =
      new LightSensorController(rightLightMode);

  public static int ACCELERATION = 300;

  /**
   * the entry point of the whole program. Run this class to start color detection at stationary
   * position
   * 
   * @param str
   */
  public static void main(String str[]) {

    Odometer odometer = null;
    NavigationWithObstacle navigation = null;
    try {
      odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      Thread odoThread = new Thread(odometer);
      odoThread.start();
    } catch (OdometerExceptions e1) {
      e1.printStackTrace();
    }
    final OdometryCorrection odometryCorrection =
        new OdometryCorrection(odometer, leftMotor, rightMotor, leftLightSensor, rightLightSensor);
    LightLocalizer lightLocalizer =
        new LightLocalizer(odometer, leftLightSensor, rightLightSensor, odometryCorrection);



    WiFi.getData();

    // get the startTime at the beginning
    startTime = System.currentTimeMillis();

    //ultrasonic localization
     try {
     UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(sampleProvider);
     usLocalizer.fallingEdge();
     } catch (OdometerExceptions e) {
     e.printStackTrace();
     }

     //light localization
    lightLocalizer.startLocalize(WiFi.localizeX*TILE_SIZE, WiFi.localizeY*TILE_SIZE, (int) WiFi.localizeTheta);
    
    //navigation initialization
    try {
      navigation = new NavigationWithObstacle(leftMotor, rightMotor, TRACK, WHEEL_RAD, WiFi.TunLL_x,
          WiFi.TunLL_y, WiFi.TunUR_x, WiFi.TunUR_y, WiFi.SZ_LL_x, WiFi.SZ_LL_y, WiFi.SZ_UR_x,
          WiFi.SZ_UR_y, WiFi.corner, sensorMotor, lcd, 1, sampleProvider, odometryCorrection,
          lightLocalizer);
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    navigation.initializePointsAndAngle();
    navigation.searchPointCalculation();
    navigation.navigateToSearchingArea();
    Thread navigationThread = new Thread(navigation);
    navigationThread.start();
    while(navigationThread.isAlive()) {}
    navigation.leaveSearchingArea();
    
    //reach the starting point
    navigation.turnTo((int) WiFi.localizeTheta);
    lightLocalizer.startLocalize(WiFi.localizeX*TILE_SIZE, WiFi.localizeY*TILE_SIZE, (int) WiFi.localizeTheta);

    //push the can to the starting tile
    navigation.turnTo(((int) WiFi.localizeTheta + 180)%360);
    leftMotor.rotate(720, true);
    rightMotor.rotate(720, false);
    navigation.releaseCan();
    
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();

  }

  private float getTimeRemaining() {
    return TIME_LIMIT - ((System.currentTimeMillis() - startTime) / 1000f);
  }

}
