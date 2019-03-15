package ca.mcgill.ecse211.odometer;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.entryPoint.*;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/** 
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * 
 */
public class Odometer extends OdometerData implements Runnable {

	private OdometerData odoData;
	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private double displacementL;
	private double displacementR;
	private int oldLeftMotorTachoCount;
	private int oldRightMotorTachoCount;


	private double[] position;

	private int nbXLines;
	private int nbYLines;
	private static final double TILE_SIZE = 30.48;
	private static final double TRACK = 16.0;
	private static final double WHEEL_RADIUS = 2.1;
	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors and variables once.It
	 * cannot be accessed externally.
	 * 
	 * @param leftMotor left motor of the robot
	 * @param rightMotor right motor of the robot
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
		// manipulation methods
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoData.setXYT(0, 0, 0);

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;

	}

	/**
	 * This method initializes the odometer depending its starting Position
	 * 
	 */
	public void initialize(int startingCorner) {
		switch(startingCorner) {
		case 0: nbXLines = 1; nbYLines = 1; odo.setXYT(nbXLines*TILE_SIZE, nbYLines*TILE_SIZE, 0); break;
		case 1: nbXLines = 14; nbYLines = 1; odo.setXYT(nbXLines*TILE_SIZE, nbYLines*TILE_SIZE, 270); break;
		case 2: nbXLines = 14; nbYLines = 8; odo.setXYT(nbXLines*TILE_SIZE, nbYLines*TILE_SIZE, 180); break;
		case 3: nbXLines = 1; nbYLines = 8; odo.setXYT(nbXLines*TILE_SIZE, nbYLines*TILE_SIZE, 90); break;
		}
		
	}
	/**
	 * This method is meant to ensure only one instance of the odometer is used throughout the code.
	 * 
	 * @param leftMotor left motor of the robot
	 * @param rightMotor right motor of the robot
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			try {
				odo = new Odometer(leftMotor, rightMotor);
			} catch (OdometerExceptions e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be used only if an
	 * odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods provided from the
	 * OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
		double leftDist, rightDist, deltaDist, deltaTheta, dX, dY;

		while (true) {
			
			updateStart = System.currentTimeMillis();

			leftMotorTachoCount = leftMotor.getTachoCount();			
			rightMotorTachoCount = rightMotor.getTachoCount();

			// TODO Calculate new robot position based on tachometer counts

			leftDist = Math.PI * WHEEL_RADIUS * (leftMotorTachoCount - oldLeftMotorTachoCount)/180;
			rightDist = Math.PI * WHEEL_RADIUS * (rightMotorTachoCount - oldRightMotorTachoCount)/180;

			oldLeftMotorTachoCount = leftMotorTachoCount;
			oldRightMotorTachoCount = rightMotorTachoCount;

			deltaDist = 0.5 * (leftDist + rightDist);
			deltaTheta = (leftDist - rightDist) / TRACK;

			//Put the deltaTheta in degrees
			deltaTheta *= (180/Math.PI);

			//Get the value of theta
			double[] values = getXYT();
			double theta = values [2];

			//The sin & cos use radians value
			dX = deltaDist * Math.sin(Math.toRadians(theta + deltaTheta));
			dY = deltaDist * Math.cos(Math.toRadians(theta + deltaTheta));

			// TODO Update odometer values with new calculated values
			odo.update(dX, dY, deltaTheta);
		      
			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}

}