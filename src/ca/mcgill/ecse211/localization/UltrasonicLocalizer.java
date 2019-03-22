package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.entryPoint.Lab5;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

/**
 * This class implements the ultraconic localizer. The theta in the odometer will be corrected after
 * the localization.
 * 
 * @author Snadra Deng
 *
 */
public class UltrasonicLocalizer implements TimerListener {

	  private static float[] usSample;
	  private static SampleProvider usSampleProvider;

	  private static Odometer odo = null;

	  private static double angleA = -1;
	  private static double angleB = -1;

	  private static double currentAngle = -1;
	  private static int distance = -1;

	  private static int k = 3;
	  
	  private static int filter_count = 0;
	  private static int usDistance= -1;
	  private static double previousUsDistance;
	  private static int filter_control = 4;
	  
	  private int ultrasonicSpeed = 220;

	  public UltrasonicLocalizer(SampleProvider usSampleProvider) throws OdometerExceptions {

	    odo = Odometer.getOdometer();
	    this.usSampleProvider = usSampleProvider;
	    usSample = new float[usSampleProvider.sampleSize()];

	    Lab5.leftMotor.setAcceleration(Lab5.ACCELERATION);
	    Lab5.rightMotor.setAcceleration(Lab5.ACCELERATION);

	  }


	  /**
	   * the robot faces away from walls initially, and will move to back wall first and then left wall
	   * after getting two angle readings, the robot will turn to the actual 0 degree and correct the
	   * theta in odometer
	   * 
	   * @throws OdometerExceptions
	   */
	  public void fallingEdge() throws OdometerExceptions {
	    Timer timer = new Timer(30, new UltrasonicLocalizer(usSampleProvider));
	    timer.start();

	    // turn the robot away from the wall
	    // 30 is the d. Since the robot will face away the wall most of the time, d is larger than the
	    // rising edge
	    awayFromWall(true, 30);
	    angleA = toWall(true, 30); // back wall
	    Sound.beep();

	    awayFromWall(false, 30);
	    angleB = toWall(false, 30); // left wall
	    Sound.beep();

	    // stop the timer, so there will be no new reading comes from the ultrasonic sensor
	    timer.stop();

	    double deltaTheta = angleB - angleA;
	    if (angleA < angleB) {
	      // (A+B)/2-225 is the angle between original orientation and the actual 0,
	      // but need to go back to original
	      deltaTheta = (angleA + angleB) / 2 - 225 + (360 - angleB);
	    } else {
	      // (A+B)/2-45 is the angle between original orientation and the actual 45
	      deltaTheta = (angleA + angleB) / 2 - 45 - (angleB);
	    }

	    // turn to the actual 0
	    Lab5.leftMotor.setSpeed(ultrasonicSpeed);
	    Lab5.rightMotor.setSpeed(ultrasonicSpeed);
	    Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaTheta), true);
	    Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaTheta), false);

	    odo.setTheta(0);

	    Lab5.lcd.drawString("Press > to start LightLocalizer", 0, 5);
	  }

	  /**
	   * the robot faces to the walls initially, and will move to the left wall first and then the back
	   * wall after getting two angle readings, the robot will return to the initial 0 degree and then
	   * turn to the actual 0 degree and then correct the theta in the odometer
	   * 
	   * @throws OdometerExceptions
	   */
	  public void risingEdge() throws OdometerExceptions {
	    Timer timer = new Timer(30, new UltrasonicLocalizer(usSampleProvider));
	    timer.start();

	    int d = 17;
	    // since the robot faces the wall at the initial state and will face the wall during the whole
	    // process
	    // the d we used is smaller than the falling edge


	    Lab5.leftMotor.setSpeed(ultrasonicSpeed);
	    Lab5.rightMotor.setSpeed(ultrasonicSpeed);
	    Lab5.leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 20), true);
	    Lab5.rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 20), false);
	    angleB = toWall(true, d); // left wall
	    Sound.beep();

	    Lab5.leftMotor.setSpeed(ultrasonicSpeed);
	    Lab5.rightMotor.setSpeed(ultrasonicSpeed);
	    Lab5.leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 20), true);
	    Lab5.rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 20), false);
	    angleA = toWall(false, d); // back wall
	    Sound.beep();

	    turnTo(0);
	    timer.stop();


	    double deltaTheta = -1;
	    if (angleA < angleB) {
	      deltaTheta = (angleB + angleA) / 2 - 225;
	    } else {
	      deltaTheta = (angleB + angleA) / 2 - 45;
	    }


	    Lab5.leftMotor.setSpeed(ultrasonicSpeed);
	    Lab5.rightMotor.setSpeed(ultrasonicSpeed);
	    Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaTheta), true);
	    Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, deltaTheta), false);

	    odo.setTheta(0);
	    Lab5.lcd.drawString("Press > to start LightLocalizer", 0, 5);

	  }

	  /**
	   * this method turn the robot away from the wall
	   * 
	   * @param direction: true for clockwise, false for anti-clockwise
	   * @param d: distance from the wall
	   */
	  private void awayFromWall(boolean direction, int d) {
	    Lab5.leftMotor.setSpeed(ultrasonicSpeed);
	    Lab5.rightMotor.setSpeed(ultrasonicSpeed);

	    while (distance < d + k) {
	      if (direction) {
	        Lab5.leftMotor.forward();
	        Lab5.rightMotor.backward();
	      } else {
	        Lab5.leftMotor.backward();
	        Lab5.rightMotor.forward();
	      }
	    }
	    Lab5.leftMotor.stop(true);
	    Lab5.rightMotor.stop();
	    Lab5.lcd.drawString("away", 0, 5);
	  }


	  private double toWall(boolean direction, int d) {
	    Lab5.leftMotor.setSpeed(ultrasonicSpeed);
	    Lab5.rightMotor.setSpeed(ultrasonicSpeed);

	    while (distance > d - k) {
	      if (direction) {
	        Lab5.leftMotor.forward();
	        Lab5.rightMotor.backward();
	      } else {
	        Lab5.leftMotor.backward();
	        Lab5.rightMotor.forward();
	      }
	    }
	    Lab5.leftMotor.stop(true);
	    Lab5.rightMotor.stop();
	    Lab5.lcd.drawString("toward", 0, 5);

	    return odo.getXYT()[2]; // return the angle
	  }



	  /**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	  private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }

	  /**
	   * this method calculate the distance that the robot needed to travel when finishing the turn
	   * 
	   * @param radius
	   * @param width
	   * @param angle
	   * @return the distance that the robot needed to travel when finishing the turn
	   * 
	   */
	  private static int convertAngle(double radius, double width, double angle) {
	    // angle >= 0
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }

	  /**
	   * a method from TimerListner interface. it fetches reading from the ultrasonic sensor when time
	   * out and record the distance reading from the ultrasonic sensor in cm
	   */
	  @Override
	  public void timedOut() {
	    // get data from ultrasonic sensor when time out
	    usSampleProvider.fetchSample(usSample, 0);
	    usDistance = (int) (usSample[0] * 100.0); // change unit to cm

	    

	        if (filter_count == 0) {
	          previousUsDistance = usDistance;
	        }

	        if (Math.abs(previousUsDistance - usDistance) < 3) {
	          filter_count++;

	          if (filter_count == 4) {
//	            Sound.beep();
//	            System.out.println("distance: " + usDistance + " theta: " + odo.getXYT()[2]);
	            System.out.println(usDistance);
	            distance = usDistance;
	            // canLocation.add(theta);
	          }

	        } else {
	          filter_count = 0;
	        }
	      

	  }

	  void turnTo(double theta) {

	    double currentTheta = odo.getXYT()[2];
	    double turningAngle = theta - currentTheta;
	    // System.out.println("turningAngle"+turningAngle);

	    if (Math.abs(turningAngle) <= 180) {
	      if (turningAngle <= 0) {
	        // turn left with -turningAngle
	        Lab5.leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -turningAngle), true);
	        Lab5.rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -turningAngle), false);
	      } else {
	        // turn right with turningAngle
	        Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turningAngle), true);
	        Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turningAngle), false);
	      }
	    } else if (Math.abs(turningAngle) > 180) {
	      if (turningAngle < 0) {
	        // turn right with 360+turning angle
	        Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 360 + turningAngle), true);
	        Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 360 + turningAngle),
	            false);
	      } else {
	        // turn left with 360 - turningAngle
	        Lab5.leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 360 - turningAngle), true);
	        Lab5.rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 360 - turningAngle), false);
	      }
	    }

	  }

	}
