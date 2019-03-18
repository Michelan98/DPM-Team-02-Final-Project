package ca.mcgill.ecse211.navigation;


import ca.mcgill.ecse211.colorClassification.ColorClassification;
import ca.mcgill.ecse211.entryPoint.Lab5;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class NavigationWithObstacle implements TimerListener {

	private static int FORWARD_SPEED = Lab5.FORWARD_SPEED;
	private static int TURNING_SPEED = Lab5.TURNING_SPEED;
	private final double TILE_SIZE = Lab5.TILE_SIZE;

	private double track;
	private double wheelRad;
	private boolean isNavigating;

	private static EV3LargeRegulatedMotor leftMotor = null;
	private static EV3LargeRegulatedMotor rightMotor = null;

	// initialize the ultrasonic sensor
	private static Port sensorPort = LocalEV3.get().getPort("S1");
	private static SensorModes us = new EV3UltrasonicSensor(sensorPort);
	private static SampleProvider sampleProvider = us.getMode("Distance");
	private static float[] sample = new float[sampleProvider.sampleSize()];
	// TODO: filter for us sensor?

	private static TextLCD lcd = null;
	private static EV3LargeRegulatedMotor sensorMotor = null;
	private int TR = -1;

	// (LLX, LLY) and (URX, URY) are from the entry point class
	private int LLY;
	private int LLX;
	private int URX;
	private int URY;


	private static Odometer odo = null;
	private int count = 0; // use it to track which point to go

	private static int flag = 0;
	private Timer timer;

	private int currentDestination[] = {0, 0};


	public NavigationWithObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double track, double wheelRad, int LLX, int LLY, int URX, int URY,
			EV3LargeRegulatedMotor sensorMotor, TextLCD lcd, int TR) throws OdometerExceptions {

		odo = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.track = track;
		this.wheelRad = wheelRad;
		this.TR = TR;
		this.lcd = lcd;
		this.sensorMotor = sensorMotor;

		this.LLX = LLX;
		this.LLY = LLY;
		this.URX = URX;
		this.URY = URY;

		currentDestination[0] = LLX;
		currentDestination[1] = LLY;

		isNavigating = false;

	}



	public void run() throws OdometerExceptions {

		timer =  new Timer(100, new NavigationWithObstacle(leftMotor, rightMotor, track,
				wheelRad, LLX, LLY, URX, URY, sensorMotor, lcd, TR));

		// from (0,0) to (LLX, LLY)
		travelTo(currentDestination[0], currentDestination[1]);
		boolean isACan = false;

		while (count <= (URX - LLX) + 1) {
			// current (x,y)
			double pos[] = odo.getXYT();
			double currentX = pos[0];
			double currentY = pos[1];
			double theta = pos[2];

			double xCoordinate = Math.round(((float) currentX / (float) TILE_SIZE));
			double yCoordinate = Math.round(((float) currentY / (float) TILE_SIZE));

			System.out.println("xCoordinate "+xCoordinate);
			System.out.println("yCoordinate "+yCoordinate);
			System.out.println("count "+ count);

			int isAtURY = -1;


			if (count != 0 && isACan) {
				// go to the next x
				currentDestination[0]++;
				travelTo(currentDestination[0], currentDestination[1]);
			}

			if (yCoordinate == LLY-1 || yCoordinate == LLY) {
				// turn to 0 and check for can
				turnTo(0);

				System.out.println("theta in LLY" + theta);

				sampleProvider.fetchSample(sample, 0);
				System.out.println("sample"+sample[0]*100);
				count++;
				isAtURY = 0;
			} else if (yCoordinate == URY+1) {
				// turn to 180 to check for can
				turnTo(180);

				System.out.println("theta in URY" + theta);

				sampleProvider.fetchSample(sample, 0);
				System.out.println("sample"+sample[0]*100);
				count++;
				isAtURY = 1;
			}


			if (sample[0] * 100 < (URY-LLY)*TILE_SIZE+20 && isAtURY != -1) {
				// found a can
				// go one tile further, in case there are some cans on LLY or URY
				isACan = true;
				if (isAtURY == 0) {
					currentDestination[1] = URY + 1;
				} else if (isAtURY == 1) {
					currentDestination[1] = LLY - 1;
				}

				System.out.println("if "+ currentDestination[0] +","+ currentDestination[1]);
				travelTo(currentDestination[0], currentDestination[1]);

			} else if (isAtURY != -1) {
				// go to the next X coordinate
				currentDestination[0]++;
				System.out.println("else if "+ currentDestination[0] +","+ currentDestination[1]);
				travelTo(currentDestination[0], currentDestination[1]);
				isACan = false;
			} else {
				count++;
				System.out.println("else "+ currentDestination[0] +","+ currentDestination[1]);
				travelTo(currentDestination[0], currentDestination[1]);
				isACan = true;  //TODO: check it...
			}
		}

		// got to (URX, URY)
		currentDestination[0] = URX;
		currentDestination[1] = URY;
		travelTo(currentDestination[0], currentDestination[1]);

	}

	/**
	 * this method cause the robot to travel to the absolute field location (x,y), and the robot will
	 * avoid obstacles it meets during the travel
	 * 
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {


		timer.start();
		// get the position reading from the odometer
		double pos[] = odo.getXYT();
		double currentX = pos[0];
		double currentY = pos[1];
		double theta = pos[2];

		// calculate the distance needed to be traveled
		double distance = Math
				.sqrt(Math.pow((currentX - x * TILE_SIZE), 2) + Math.pow((currentY - y * TILE_SIZE), 2));

		// turn to the destination angle
		turnTo(angleCalculation(currentX, currentY, x * TILE_SIZE, y * TILE_SIZE));

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(wheelRad, distance), true);
		rightMotor.rotate(convertDistance(wheelRad, distance), true);

		while (isNavigating()) {
			//      System.out.println("isNavigating "+flag);
			if (flag == 1) {
				// detect a block
				System.out.println("start color detection");
				timer.stop();
				startColorDetection();
				// finish the colorDetection, start the can avoidance
				canAvoidance();
				// avoided the can continue on the navigation
				count--;
				System.out.println("finish color detection "+ count);
				break;
			}
		}

		timer.stop();

	}

	/**
	 * this method causes the robot to turn to the absolute heading theta the method turn a minimal
	 * angel to its target
	 * 
	 * @param theta
	 */
	void turnTo(double theta) {

		double currentTheta = odo.getXYT()[2];
		double turningAngle = theta - currentTheta;

		leftMotor.setSpeed(TURNING_SPEED);
		rightMotor.setSpeed(TURNING_SPEED);

		if (Math.abs(turningAngle) <= 180) {
			if (turningAngle <= 0) {
				// turn left with -turningAngle
				leftMotor.rotate(-convertAngle(wheelRad, track, -turningAngle), true);
				rightMotor.rotate(convertAngle(wheelRad, track, -turningAngle), false);
			} else {
				// turn right with turningAngle
				leftMotor.rotate(convertAngle(wheelRad, track, turningAngle), true);
				rightMotor.rotate(-convertAngle(wheelRad, track, turningAngle), false);
			}
		} else if (Math.abs(turningAngle) > 180) {
			if (turningAngle < 0) {
				// turn right with 360+turning angle
				leftMotor.rotate(convertAngle(wheelRad, track, 360 + turningAngle), true);
				rightMotor.rotate(-convertAngle(wheelRad, track, 360 + turningAngle), false);
			} else {
				// turn left with 360 - turningAngle
				leftMotor.rotate(-convertAngle(wheelRad, track, 360 - turningAngle), true);
				rightMotor.rotate(convertAngle(wheelRad, track, 360 - turningAngle), false);
			}
		}

	}

	/**
	 * 
	 * @return true if another thread has called travelTo() or turnTo() false otherwise
	 */
	boolean isNavigating() {

		if (leftMotor.isMoving() && rightMotor.isMoving()) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return the distance that the robot needs to travel to the destination
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
	 * 
	 * @param currentX
	 * @param currentY
	 * @param x
	 * @param y
	 * @return angle needs to turn this method calculate the angle that the robot needs to turn when
	 *         traveling to the destination
	 */
	private double angleCalculation(double currentX, double currentY, double x, double y) {

		// theta is in degree
		double dy = y - currentY;
		double dx = x - currentX;
		double dt = 0;
		if (dy >= 0) {
			dt = Math.atan(dx / dy);
		} else if (dy <= 0 && dx >= 0) {
			dt = Math.atan(dx / dy) + Math.PI;
		} else {
			dt = Math.atan(dx / dy) - Math.PI;
		}

		return dt * 180 / Math.PI;
	}

	private void startColorDetection() {

		//Sound.beep();
		int sampleInCm = (int) (sample[0] * 100);
		// TODO: leave 1 cm space??
		//int distance = sampleInCm % 10 - 1 + 10 * (sampleInCm % 10);
		int distance = sampleInCm -2;
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(wheelRad, distance), true);
		rightMotor.rotate(convertDistance(wheelRad, distance), false);
		Sound.beep();
		// reach the can and start the colorDetection

		// final ColorClassification colorClassification = new ColorClassification(sensorMotor, lcd
		// ,TR);
		//
		// new Thread() {
		// public void run() {
		// try {
		// colorClassification.colorClassify();
		// } catch (InterruptedException e) {
		// // e.printStackTrace();
		// }
		// }
		// }.start();

		//TODO: need to shut down the current thread

		// the light sensor is at its original position, need to start can avoidance
		// ensure there is enough space for turning
		leftMotor.rotate(-convertDistance(wheelRad, distance), true);
		rightMotor.rotate(-convertDistance(wheelRad, distance), false);
		flag = 0;

	}

	private void canAvoidance() {

		//turn right 90 degree
		leftMotor.rotate(convertAngle(wheelRad,track, 90), true);
		rightMotor.rotate(-convertAngle(wheelRad,track, 90), false);

		leftMotor.rotate(convertDistance(wheelRad, TILE_SIZE *0.7), true);
		rightMotor.rotate(convertDistance(wheelRad, TILE_SIZE *0.7), false);

		//turn left 90 degree
		leftMotor.rotate(-convertAngle(wheelRad,track, 90), true);
		rightMotor.rotate(convertAngle(wheelRad,track, 90), false);

		leftMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 1.3), true);
		rightMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 1.3), false);

		//turn left 90 degree
		leftMotor.rotate(-convertAngle(wheelRad,track, 90), true);
		rightMotor.rotate(convertAngle(wheelRad,track, 90), false);

		leftMotor.rotate(convertDistance(wheelRad, TILE_SIZE *0.7), true);
		rightMotor.rotate(convertDistance(wheelRad, TILE_SIZE *0.7), false);

		//turn right 90 degree
		leftMotor.rotate(convertAngle(wheelRad,track, 90), true);
		rightMotor.rotate(-convertAngle(wheelRad,track, 90), false);

		// pass through the can continue the navigation
	}

	@Override
	public void timedOut() {
		sampleProvider.fetchSample(sample, 0); // distance from ultrasonic sensor in m
		int distance = (int) (sample[0] * 100); // convert to cm
		//System.out.println("distance in timeout " + distance);

		if (distance < 10 && distance >2) {
			// detect a can, stop the navigation and get ready for color detection
			Sound.beep();
			flag = 1;
			// TODO: is stop() needed?
			leftMotor.stop();
			rightMotor.stop();
		}
	}

}
