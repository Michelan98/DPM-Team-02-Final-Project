package ca.mcgill.ecse211.odometer;


import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.entryPoint.*;

public class OdometryCorrection {
  // Constants
  private static final int FORWARD_SPEED = 150;
  private static final int ROTATE_SPEED = 80;
  private static final double TILE_SIZE = 30.48;
  public static final double DISTANCE_TO_SENSOR = 11.3;
  private static final double THRESHOLD = LightSensorController.THRESHOLD;

  // Left and right light sensors
  private LightSensorController leftLightSensor;
  private LightSensorController rightLightSensor;
  // Left and right motors
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  
  // constants for correction state in Navigation
  public static boolean oneLineDetected;
  public static boolean otherLineDetected;


  private Odometer odometer;

  /**
   * Construction of the odometryCorrection class
   * 
   * @param odometer - odometer of the robot (singleton)
   * @param rightLightSensor - right front light sensor that is used
   * 
   * @param leftLightSensor - left back light sensor that is used
   * @param rightLightSensor - right back light sensor that is used
   * 
   * @param leftMotor - left motor that is used
   * @param rightMotor - right motor that is used
   */
  public OdometryCorrection(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, LightSensorController leftLightSensor,
      LightSensorController rightLightSensor) {
    this.odometer = odometer;
    this.leftLightSensor = leftLightSensor;
    this.rightLightSensor = rightLightSensor;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

  }

  /**
   * This class allows the robot to correct its positioning
   * 
   * @param thetaCorrection - orientation at which you want the robot to be/is
   */
  public void correct(double thetaCorrection) {

    travelDistance(-7, 150);


    boolean rightLineDetected = false;
    boolean leftLineDetected = false;

    while (!leftLineDetected && !rightLineDetected) {
      // double rightSample = rightLS.fetch();
      // double leftSample = leftLS.fetch();
      if (rightLightSensor.fetch() < THRESHOLD) {
        rightLineDetected = true;
        // Stop the right motor

      } else if (leftLightSensor.fetch() < THRESHOLD) {
        leftLineDetected = true;

        // Stop the left motor
      }
    }
    
    oneLineDetected = rightLineDetected;
    otherLineDetected = leftLineDetected;
    		

    // Get the odometer's reading

    // Keep moving the left/right motor until both lines have been detected
    while ((!leftLineDetected || !rightLineDetected)) {
      // If the other line detected, stop the motors
      if (rightLineDetected && leftLightSensor.fetch() < THRESHOLD) {
        leftLineDetected = true;
      } else if (leftLineDetected && rightLightSensor.fetch() < THRESHOLD) {
        rightLineDetected = true;
      }
    }
    
    oneLineDetected = rightLineDetected;
    otherLineDetected = leftLineDetected;
    
    // correcting on theta
    correctOdometer(thetaCorrection);    
    
    // preparing for crossing a grid line after correcting
    resetMotors();  
    setSpeeds(150, 150);


  }

  /**
   * Correct odometer
   * 
   * @param theta - orientation at which you want the robot to be/is
   */
  private void correctOdometer(double theta) {
    // Correction variables
    double xCorrection = 0;
    double yCorrection = 0;
    double thetaCorrection = translateTheta(theta);

    // Correction in X
    if (thetaCorrection == 90 || thetaCorrection == 270) {

      if (thetaCorrection == 90) {
        // Compute the sensors' X position in cm's
        double position = odometer.getXYT()[0] + DISTANCE_TO_SENSOR;

        // Find the X-coordinate of the nearest waypoint to sensorX.
        int correctedPosition = (int) Math.round(position / TILE_SIZE);

        // Get the correct X
        xCorrection = TILE_SIZE * correctedPosition - DISTANCE_TO_SENSOR;

      } else {
        // Compute the sensors' X position in cm's
        double position = odometer.getXYT()[0] - DISTANCE_TO_SENSOR;

        // Find the X-coordinate of the nearest waypoint to sensorX.
        int correctedPosition = (int) Math.round(position / TILE_SIZE);

        // Get the correct X
        xCorrection = TILE_SIZE * correctedPosition + DISTANCE_TO_SENSOR;

      }
      odometer.setX(xCorrection);

      // Correction in Y
    } else if (thetaCorrection == 0 || thetaCorrection == 180) {

      if (thetaCorrection == 0) {
        // Compute the sensors' Y position in cm's
        double position = odometer.getXYT()[1] + DISTANCE_TO_SENSOR;

        // Find the X-coordinate of the nearest waypoint to sensorX.
        int correctedPosition = (int) Math.round(position / TILE_SIZE);

        // Get the correct Y
        yCorrection = TILE_SIZE * correctedPosition - DISTANCE_TO_SENSOR;

        // Get the correct X
        // corrX = intermediateOdo[0] - (dTheta / Math.abs(dTheta) * offset);

      } else {
        // Compute the sensors' Y position in cm's
        double position = odometer.getXYT()[1] - DISTANCE_TO_SENSOR;

        // Find the X-coordinate of the nearest waypoint to sensorX.
        int correctedPosition = (int) Math.round(position / TILE_SIZE);

        // Get the correct Y
        yCorrection = TILE_SIZE * correctedPosition + DISTANCE_TO_SENSOR;

      }
      odometer.setY(yCorrection);
    }

    odometer.setTheta(thetaCorrection);

  }

  /**
   * This method allows to round the angle received
   * 
   * @param theta angle to get rounded
   * @return angle that is rounded to a general angle (integer)
   */
  private double translateTheta(double theta) {
    if (theta > 345 && theta < 15) {
      return 0;
    }
    if (theta < 105 && theta > 75) {
      return 90;
    }
    if (theta < 195 && theta > 165) {
      return 180;
    }
    if (theta < 285 && theta > 255) {
      return 270;
    }
    return 0;
  }

  /**
   * This method resets the motors to stop in order to correct upon crossing a grid line
   */
  public void resetMotors() {
    // reset the motor
    stopMotors();
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.setAcceleration(3000);
    }
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
    }
  }

  /**
   * @param leftSpeed - left motor speed
   * @param rightSpeed - right motor speed
   * 
   */
  public void setSpeeds(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  }

  /**
   * @param distance - distance for motor to "travel"
   * @param speed - speed motors are set to
   * 
   */
  public void travelDistance(double distance, int speed) {

    resetMotors();
    setSpeeds(speed, speed);
    leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
    rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), false);
  }

  /**
   * @param radius - distance for motor to "travel"
   * @param distance - distance for motor to "travel"
   *
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * A method that synchronizes the forward movement of the motors using the SynchronizeWith()
   * method provided in LeJos
   */
  public void moveForward() {
    leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] {rightMotor});
    leftMotor.startSynchronization();
    leftMotor.forward();
    rightMotor.forward();
    leftMotor.endSynchronization();
  }

  /**
   * helper method to stop moving motors
   */
  public void stopMoving(boolean stopLeft, boolean stopRight) {
    leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] {rightMotor});
    leftMotor.startSynchronization();
    if (stopLeft)
      leftMotor.stop();
    if (stopRight)
      rightMotor.stop();
    leftMotor.endSynchronization();
  }

  /**
   * helper for stopping both motors
   * 
   */
  public void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();

  }

  /**
   * This method allows the conversion of a angle to the total rotation of each wheel need to cover
   * that distance.
   * 
   * @param radius radius of wheel
   * @param width track of robot
   * @param angle angle desired to turn the robot by
   * @return the angle the robot needs to turn each wheel to rotate
   */
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * direction = true --> clockwwise direction
   * 
   * @param theta
   * @param direction
   */
  public void turnBy(double theta, boolean direction) {
    if (direction) {
      leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, theta), true);
      rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, theta), false);
    } else {
      leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, theta), true);
      rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, theta), false);
    }
  }
}


