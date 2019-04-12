package ca.mcgill.ecse211.colorClassification;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

/**
 * This class is responsible for can color classification with a rotating light sensor.
 * 
 * @author Sandra Deng
 *
 */
public class ColorClassification implements TimerListener {

  private static Port colorPort = LocalEV3.get().getPort("S1");
  private static final SensorModes colorSensor = new EV3ColorSensor(colorPort);
  private static final SampleProvider colorSampleProvider = colorSensor.getMode("RGB");
  private float[] sample = new float[colorSampleProvider.sampleSize()];

  private static EV3LargeRegulatedMotor sensorMotor;
  private static TextLCD lcd;

  // 1: blue, 2: green, 3: yellow, 4: red, 5: white
  private double canRGBNormalizedMeans[][] = {{0.298917037, 0.726693533, 0.612517117},
      {0.250897607, 0.943892592, 0.207426042}, {0.833111075, 0.526009994, 0.156434809},
      {0.927464231, 0.317823524, 0.130221225}, {0.711488185, 0.570923876, 0.377780294}};

  // 1: blue can, 2: green can, 3: yellow can, 4: red can, 5:label
  private static int colorCount[] = {0, 0, 0, 0, 0};

  // target color
  private static int TR = -1;

  private int beepingTime = 500;

  private SampleProvider usSensor = null;
  private static boolean noCan = false;


  /**
   * constructor
   * @param sensorMotor
   * @param lcd
   * @param TR
   */
  public ColorClassification(EV3LargeRegulatedMotor sensorMotor, TextLCD lcd, int TR) {
    this.sensorMotor = sensorMotor;
    this.lcd = lcd;
    this.TR = TR; // 1 larger than the index
  }

  /**
   * This method classifies the color of the can. The sensor motor will rotate 110 degrees forward
   * and 110 degrees backward, so around 80 samples will be fetched from the color sensor, and the
   * sensor will go back to its original position. Each sample will be classified as a color, and
   * the number of "hit" of the 4 colors will be counted. The final result is determined by the
   * color with largest number of "hit". The robot will issue a sequence of beeps based on the
   * result.
   * 
   * @throws InterruptedException
   */
  public boolean colorClassify() throws InterruptedException {



    Timer timer = new Timer(30, new ColorClassification(sensorMotor, lcd, TR));


    // rotate 110 degrees to push the can into desired position
    sensorMotor.setSpeed(100);
    sensorMotor.rotate(-110);

    // rotate 110 degrees anticlockwise to fetch samples
    sensorMotor.setSpeed(70);
    timer.start();
    sensorMotor.rotate(110);
    // stop the timer, so no sample will be fetched during calculation time
    timer.stop();

    if (noCan) {
      //there is no can in the color classification region
      noCan = false;
      return false;
    } else {

      // find out the color with largest number of "hit"
      int result = -1;
      int temp = 0;
      for (int i = 0; i < 4; i++) {
        if (temp < colorCount[i]) {
          temp = colorCount[i];
          result = i + 1;
        }
      }


      // issue beeps based on the result
      switch (result) {
        case 1:
          lcd.drawString("Blue", 0, 2);
          Sound.playTone(500, beepingTime);
          break;
        case 2:
          lcd.drawString("Green", 0, 2);
          Sound.playTone(500, beepingTime);
          Sound.playTone(500, beepingTime);
          break;
        case 3:
          lcd.drawString("Yellow", 0, 2);
          Sound.playTone(500, beepingTime);
          Sound.playTone(500, beepingTime);
          Sound.playTone(500, beepingTime);
          break;
        case 4:
          lcd.drawString("Red", 0, 2);
          Sound.playTone(500, beepingTime);
          Sound.playTone(500, beepingTime);
          Sound.playTone(500, beepingTime);
          Sound.playTone(500, beepingTime);
          break;
      }

      return true;
    }


  }

  /**
   * This is a method from TimerListener interface. It fetches sample from the color sensor with
   * 100Hz. Each sample will be normalized and then compared to the five standard colors. A best
   * match color will be determined and the number of "hit" of that color is increased by 1.
   * 
   */
  @Override
  public void timedOut() {
    // motor rotate 110 degrees, fetch sample from sensor. compare the sample with the 4 colors and
    // calculate euclidean mean
    colorSampleProvider.fetchSample(sample, 0);

    double rgb[] = {sample[0], sample[1], sample[2]};

    double d[] = {0, 0, 0, 0, 0};
    for (int i = 0; i < 5; i++) {
      d[i] = Math.sqrt(Math.pow((normalization(rgb[0], rgb) - canRGBNormalizedMeans[i][0]), 2)
          + Math.pow(((normalization(rgb[1], rgb) - canRGBNormalizedMeans[i][1])), 2)
          + Math.pow(((normalization(rgb[2], rgb) - canRGBNormalizedMeans[i][2])), 2));
    }

    int result = -1;
    double temp = 999;
    for (int i = 0; i < 5; i++) {
      // find the smallest euclidean mean
      if (temp > d[i]) {
        temp = d[i];
        result = i;

      }
    }
    if (result == -1) {
      noCan = true;
      return;
    }
    //number of hit of the color with smallest deviation increased by one
    colorCount[result]++;

  }

  /**
   * This is a helper method used in timedOut(). It normalizes the sample.
   * 
   * @param sample
   * @param rgbReadings
   * @return
   */
  private double normalization(double sample, double[] rgbReadings) {

    return sample / (Math.sqrt(rgbReadings[0] * rgbReadings[0] + rgbReadings[1] * rgbReadings[1]
        + rgbReadings[2] * rgbReadings[2]));
  }
}
