package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import ca.mcgill.ecse211.entryPoint.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.controller.*;


/**
 * This class serves to localize the robot at the starting corner using the two light sensors in the
 * back. I basically moves to the intersection of the starting corner and corrects when detecting a
 * grid line in its path. The robot should beep three times when parallel to the wall at the left of
 * the robot
 * 
 * @author Michel Abdel Nour
 * @author Sandra Deng
 *
 */

public class LightLocalizer {

	// Constants
	private static final double WHEEL_RADIUS = 2.1;
	private static int FORWARD_SPEED = Lab5.FORWARD_SPEED;
	private static int TURNING_SPEED = Lab5.TURNING_SPEED;
	private final double TILE_SIZE = Lab5.TILE_SIZE;
	private final double DISTANCE_TO_SENSOR = OdometryCorrection.DISTANCE_TO_SENSOR;

	private Odometer odometer;

	private OdometryCorrection odometerCorrection;

	// declaring lightSensorController objects
	private static LightSensorController leftLightSensor;
	private static LightSensorController rightLightSensor;

	//private double color = 0.2;

	/**
	 * Constructor for the LightLocalizer class
	 * 
	 * @param odometer
	 * @param leftLightSensor
	 * @param rightLightSensor
	 */
	public LightLocalizer(Odometer odometer, LightSensorController leftLightSensor,
			LightSensorController rightLightSensor, OdometryCorrection odometerCorrection) {
		this.odometer = odometer;
		this.odometerCorrection = odometerCorrection;
		this.FORWARD_SPEED = FORWARD_SPEED;
		this.TURNING_SPEED = TURNING_SPEED;
		this.leftLightSensor = leftLightSensor;
		this.rightLightSensor = rightLightSensor;

	}

	public void startLocalize(double x, double y, int theta) {

		// start moving forward
		odometerCorrection.setSpeeds(220, 220);

		odometerCorrection.moveForward();
		
		// recalibrate the robot before correcting on the grid line
		odometerCorrection.slowDown();	
		
		odometerCorrection.moveForward();
		
		// correct on grid linest
		correct();
		
		odometerCorrection.travelDistance(-DISTANCE_TO_SENSOR, 200);
		
		// turning clockwise for direction = true
		odometerCorrection.setSpeeds(220, 220);
		odometerCorrection.turnBy(90, true);

		odometerCorrection.moveForward();
		
		odometerCorrection.slowDown();	
		
		odometerCorrection.moveForward();

		correct();

		odometerCorrection.travelDistance(-DISTANCE_TO_SENSOR, 200);
		odometerCorrection.setSpeeds(200, 200);
		odometerCorrection.turnBy(-90, true);

		// beeps three times when parallel to wall
		odometer.setXYT(x, y, theta);
		System.out.println(odometer.getXYT()[0]+" "+ odometer.getXYT()[1]+" "+odometer.getXYT()[2]);
		Sound.beep();
		Sound.beep();
		Sound.beep();

	}

	/**
	 * This method serves to correct the orientation of the robot with line detection
	 */
	private void correct() {

		boolean rightLineDetected = false;
		boolean leftLineDetected = false;

		// Move the robot until one of the sensors detects a line
		while (!leftLineDetected && !rightLineDetected) {
			if (rightLightSensor.lineDetected()) {
				rightLineDetected = true;
				// Stop the right motor
				odometerCorrection.stopMoving(false, true);

			} else if (leftLightSensor.lineDetected()) {
				leftLineDetected = true;

				// Stop the left motor
				odometerCorrection.stopMoving(true, false);
			}
		}

		// Keep moving the left/right motor until both lines have been detected
		while ((!leftLineDetected || !rightLineDetected)) {
			// If the other line detected, stop the motors
			if (rightLineDetected && leftLightSensor.lineDetected()) {
				leftLineDetected = true;
				odometerCorrection.stopMotors();

			} else if (leftLineDetected && rightLightSensor.lineDetected()) {
				rightLineDetected = true;
				odometerCorrection.stopMotors();
			}
		}

	}

}
