package ca.mcgill.ecse211.odometer;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * This class stores and provides thread safe access to the odometer data.
 * 
 */
public class OdometerData {

	// Position parameters
	private volatile double x; // x-axis position
	private volatile double y; // y-axis position
	private volatile double theta; // Head angle

	/*
	 * Class control variables
	 */
	private volatile static int numberOfIntances = 0; // Number of OdometerData
	private static final int MAX_INSTANCES = 1; // Maximum number of OdometerData instances

	/* Thread control tools 
	 * Declaring instances of objects for monitoring the state of the Threads
	 * Since there is a limited Thread count for EV3
	 * 
	 */
	private static Lock lock = new ReentrantLock(true); // Fair lock for
	
	private volatile boolean isResetting = false; // Indicates if a thread is
	
	private Condition doneReset = lock.newCondition(); // Let other threads


	private static OdometerData odoData = null;

	/**
	 * Default constructor. The constructor is private. A factory is used instead such that only one
	 * instance of this class is ever created.
	 */
	protected OdometerData() {
		this.x = 0;
		this.y = 0;
		this.theta = 0;
	}

	/**
	 * OdometerData factory. Returns an OdometerData instance and makes sure that only one instance is
	 * ever created. If the user tries to instantiate multiple objects, the method throws a
	 * MultipleOdometerDataException.
	 * 
	 * @return An OdometerData object
	 * @throws OdometerExceptions
	 */
	public synchronized static OdometerData getOdometerData() throws OdometerExceptions {
		if (odoData != null) { // Return existing object
			return odoData;
		} else if (numberOfIntances < MAX_INSTANCES) { // create object and
			// return it
			odoData = new OdometerData();
			numberOfIntances += 1;
			return odoData;
		} else {
			throw new OdometerExceptions("Only one intance of the Odometer can be created.");
		}

	}

	/**
	 * Return the Odomometer data.
	 * <p>
	 * Writes the current position and orientation of the robot onto the odoData array. odoData[0] =
	 * x, odoData[1] = y; odoData[2] = theta;
	 * 
	 * @param position the array to store the odometer data
	 * @return the odometer data.
	 */
	public double[] getXYT() {
		double[] position = new double[3];
		lock.lock();
		try {
			while (isResetting) { // If a reset operation is being executed, wait
				// until it is over.
				doneReset.await(); // Using await() is lighter on the CPU
				// than simple busy wait.
			}

			position[0] = x;
			position[1] = y;
			position[2] = theta;

		} catch (InterruptedException e) {
			// Print exception to screen
			e.printStackTrace();
		} finally {
			lock.unlock();
		}

		return position;

	}

	/**
	 * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for
	 * odometry.
	 * 
	 * @param dx
	 * @param dy
	 * @param dtheta
	 */
	public void update(double dx, double dy, double dtheta) {
		lock.lock();
		isResetting = true;
		try {
			x += dx;
			y += dy;
			theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates
			// within 360
			// degrees
			isResetting = false; // Done reseting
			doneReset.signalAll(); // Let the other threads know that you are
			// done reseting
		} finally {
			lock.unlock();
		}

	}

	/**
	 * Overrides the values of x, y and theta. Use for odometry correction.
	 * 
	 * @param x the value of x
	 * @param y the value of y
	 * @param theta the value of theta
	 */
	public void setXYT(double x, double y, double theta) {
		lock.lock();
		isResetting = true;
		try {
			this.x = x;
			this.y = y;
			this.theta = theta;
			isResetting = false; // Done reseting
			doneReset.signalAll(); // Let the other threads know that you are
			// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides x. Use for odometry correction.
	 * 
	 * @param x the value of x
	 */
	public void setX(double x) {
		lock.lock();
		isResetting = true;
		try {
			this.x = x;
			isResetting = false; // Done reseting
			doneReset.signalAll(); // Let the other threads know that you are
			// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides y. Use for odometry correction.
	 * 
	 * @param y the value of y
	 */
	public void setY(double y) {
		lock.lock();
		isResetting = true;
		try {
			this.y = y;
			isResetting = false; // Done reseting
			doneReset.signalAll(); // Let the other threads know that you are
			// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides theta. Use for odometry correction.
	 * 
	 * @param theta the value of theta
	 */
	public void setTheta(double theta) {
		lock.lock();
		isResetting = true;
		try {
			this.theta = theta;
			isResetting = false; // Done reseting
			doneReset.signalAll(); // Let the other threads know that you are
			// done reseting
		} finally {
			lock.unlock();
		}
	}
}