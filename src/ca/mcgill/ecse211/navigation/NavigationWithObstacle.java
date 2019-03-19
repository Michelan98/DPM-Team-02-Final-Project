package ca.mcgill.ecse211.navigation;

import java.util.ArrayList;
import ca.mcgill.ecse211.colorClassification.ColorClassification;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
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

/**
 * This class deals with the navigation system of the robot.
 * 
 * @author Sandra Deng
 *
 */
public class NavigationWithObstacle implements TimerListener, Runnable {
  private static final int FORWARD_SPEED = 150;
  private static final int TURNING_SPEED = 70;
  private static final double TILE_SIZE = 30.48;


  private double track;
  private double wheelRad;
  private boolean isNavigating;

  private static EV3LargeRegulatedMotor leftMotor = null;
  private static EV3LargeRegulatedMotor rightMotor = null;

  private static SampleProvider sampleProvider;
  private static float[] sample;


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
  private static ColorClassification colorClassification = null;

  private int currentDestination[] = {0, 0};
  private boolean isTargetCan = false;

  private boolean aCanOnBoundary = false;
  private static volatile int usDistance;
  private static volatile boolean detected = false;
  private int filter_count = 0;
  private static int previousUsDistance = 0;

  private enum State {
    INIT, TRAVELLING, SCANNING, COLORDETECTION, AVOIDINGCAN, GRABBINGCAN, LEAVING, CORRECTING
  };

  private static State state;
  private final double navigationAccuracy = 3;
  private final double thetaAccuracy = 3;
  private boolean offsetAdded = false;

  private volatile static ArrayList<Double> canLocation; // the angle of can to the robot
  private static ArrayList<Double> dataset;
  
  private static OdometryCorrection odometryCorrection;

  public NavigationWithObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double track, double wheelRad, int LLX, int LLY, int URX, int URY,
      EV3LargeRegulatedMotor sensorMotor, TextLCD lcd, int TR, SampleProvider sampleProvider, OdometryCorrection odometryCorrection)
      throws OdometerExceptions {

    odo = Odometer.getOdometer();
    System.out.println(odo.getXYT()[0] + " " + odo.getXYT()[1] + " " + odo.getXYT()[2]);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

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

    colorClassification = new ColorClassification(sensorMotor, lcd, TR);

    this.sampleProvider = sampleProvider;
    sample = new float[sampleProvider.sampleSize()];

    isNavigating = false;

    canLocation = new ArrayList<Double>();

    // TODO: don't forget to change it to INIT!!
    state = State.INIT;
    dataset = new ArrayList<Double>();
    
    this.odometryCorrection = odometryCorrection;

  }

  /**
   * This method is inherited from the runnable interface. It is the entry point of the thread.
   * State transition happens in this method.
   */
  public void run() {

    try {
      timer = new Timer(100, new NavigationWithObstacle(leftMotor, rightMotor, track, wheelRad, LLX,
          LLY, URX, URY, sensorMotor, lcd, TR, sampleProvider, odometryCorrection));
    } catch (OdometerExceptions e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    timer.start();

    // travel to the center of the searching area
    currentDestination[0] = (LLX + URX) /2;
    currentDestination[1] = (LLY + URY) /2;


    state = State.INIT;


    while (true) {

      switch (state) {
        case INIT: {
          if (!isNavigating()) {
            state = State.TRAVELLING;
          }
          break;
        }
        case TRAVELLING: {
          if (usDistance < 15 && usDistance > 2) {
            state = State.AVOIDINGCAN;
          }else if (OdometryCorrection.oneLineDetected){
            state = State.CORRECTING;
            System.out.println("transit to correcting");
          }
          else if (!checkIfDone(currentDestination[0], currentDestination[1])) {
            travelTo(currentDestination[0], currentDestination[1]);
          } else if (checkIfDone(currentDestination[0], currentDestination[1])) {
            turnTo(0);
            leftMotor.stop();
            rightMotor.stop();
            Sound.beep();
            state = State.SCANNING;
          }

          break;
        }
        case CORRECTING:{
          double theta = odo.getXYT()[2];
          double correctedHeading = 0;
          if((theta <10 && theta >0) || (theta >350 && theta <360)) {
            correctedHeading = 0;
          }
          else if(theta <100 && theta > 80) {
            correctedHeading = 90;
          }
          else if(theta >170 && theta < 190) {
            correctedHeading = 180;
          }
          else if(theta < 280 && theta >260) {
            correctedHeading = 270;
          }
          odometryCorrection.correct(correctedHeading);
          state = State.TRAVELLING;
          System.out.println("going to travelling");
        }
        case SCANNING: {
          scan();
          state = State.COLORDETECTION;
        }
        case COLORDETECTION: {

          for (Double data : canLocation) {
            turnTo(data - 3);
            sampleProvider.fetchSample(sample, 0);
            usDistance = (int) (sample[0] * 100); // convert to cm
            startColorDetection();
            // if (isTargetCan) {
            // state = State.GRABBINGCAN;
            // break;
            // }
          }

          state = State.LEAVING;

          break;

        }
        case GRABBINGCAN: {
          startCanGrabbing();
          state = State.LEAVING;
        }
        case AVOIDINGCAN: {
          canAvoidance();
          state = State.TRAVELLING;
          break;
        }
        case LEAVING: {
          // leave the searching area and go to the tunnel
          currentDestination[0] = URX;
          currentDestination[1] = URY;
          if (usDistance < 15 && usDistance > 2) {
            state = State.AVOIDINGCAN;
          } else if (!checkIfDone(currentDestination[0], currentDestination[1])) {
            travelTo(currentDestination[0], currentDestination[1]);
          } else if (checkIfDone(currentDestination[0], currentDestination[1])) {
            turnTo(0);
            leftMotor.stop();
            rightMotor.stop();
            Sound.beep();
          }
          break;
        }
      }
      if (checkIfDone(currentDestination[0], currentDestination[1]) && state == State.LEAVING) {
        break;
      }

      // try {
      // Thread.sleep(50);
      // } catch (InterruptedException e) {
      // // TODO Auto-generated catch block
      // e.printStackTrace();
      // }
    }
    turnTo(0);
    leftMotor.stop();
    rightMotor.stop();
    Sound.beep();
    timer.stop();
  }


  /**
   * This method cause the robot to travel to the absolute field location (x,y), and the robot will
   * avoid obstacles it meets during the travel.
   * 
   * @param x: the x coordinate of the point
   * @param y: the y coordinate of the point
   */
  public void travelTo(double x, double y) {


    // get the position reading from the odometer
    double pos[] = odo.getXYT();
    double currentX = pos[0];
    double currentY = pos[1];
    double theta = pos[2];


    // run along the line
    double distanceX = x * TILE_SIZE - currentX;
    double distanceY = y * TILE_SIZE - currentY;

    if (Math.abs(distanceX) > navigationAccuracy) {
      if (distanceX < 0 && Math.abs(theta - 270) > thetaAccuracy) {
        turnTo(270);
      } else if (distanceX > 0 && Math.abs(theta - 90) > thetaAccuracy) {
        turnTo(90);
      }

      leftMotor.forward();
      rightMotor.forward();

    } else {
      // x is done
      if (!offsetAdded) {
        // change the 3 to distanceX?
        leftMotor.rotate(convertDistance(wheelRad, 3), true);
        rightMotor.rotate(convertDistance(wheelRad, 3), false);
        offsetAdded = true;
      }
      if (distanceY < 0 && Math.abs(theta - 180) > thetaAccuracy) {
        turnTo(180);
      } else if (distanceY > 0) {
        if (theta > 180 && Math.abs(theta - 360) > thetaAccuracy) {
          turnTo(0);
        } else if (theta < 180 && Math.abs(theta - 0) > thetaAccuracy) {
          turnTo(0);
        }


      }

      leftMotor.forward();
      rightMotor.forward();

    }


  }

  /**
   * This method causes the robot to turn to the absolute heading theta the method turn a minimal
   * angel to its target.
   * 
   * @param theta
   */
  public void turnTo(double theta) {

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
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

  }

  /**
   * This method check whether the robot is navigating.
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
   * This method calculate the distance that the robot needed to travel when finishing the turn.
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
   * This method starts color detection.
   */
  private void startColorDetection() {

    int distance = usDistance;
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // change the way of traveling!!!
    leftMotor.rotate(convertDistance(wheelRad, distance), true);
    rightMotor.rotate(convertDistance(wheelRad, distance), false);
    // reach the can and start the colorDetection

    Thread colorThread = new Thread() {
      public void run() {
        try {
          isTargetCan = colorClassification.colorClassify();
        } catch (InterruptedException e) {
          // e.printStackTrace();
        }
      }
    };

    colorThread.start();
    try {
      colorThread.join();
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    if (isTargetCan) {
      return;
    }

    // go back to the center
    leftMotor.rotate(-convertDistance(wheelRad, distance), true);
    rightMotor.rotate(-convertDistance(wheelRad, distance), false);

  }

  /**
   * This method starts can grabbing process.
   */
  public void startCanGrabbing() {

  }


  /**
   * This method drives the robot to avoid a can.
   */
  private void canAvoidance() {

    leftMotor.setSpeed(TURNING_SPEED);
    rightMotor.setSpeed(TURNING_SPEED);

    // turn right 90 degree
    leftMotor.rotate(convertAngle(wheelRad, track, 90), true);
    rightMotor.rotate(-convertAngle(wheelRad, track, 90), false);

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED + 2);

    leftMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 0.7), true);
    rightMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 0.7), false);

    // leftMotor.setSpeed(TURNING_SPEED);
    // rightMotor.setSpeed(TURNING_SPEED);
    //
    // // turn left 90 degree
    // leftMotor.rotate(-convertAngle(wheelRad, track, 90), true);
    // rightMotor.rotate(convertAngle(wheelRad, track, 90), false);
    //
    // leftMotor.setSpeed(FORWARD_SPEED);
    // rightMotor.setSpeed(FORWARD_SPEED + 2);
    //
    // leftMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 1.5), true);
    // rightMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 1.5), false);
    //
    // leftMotor.setSpeed(TURNING_SPEED);
    // rightMotor.setSpeed(TURNING_SPEED);
    //
    // // turn left 90 degree
    // leftMotor.rotate(-convertAngle(wheelRad, track, 90), true);
    // rightMotor.rotate(convertAngle(wheelRad, track, 90), false);
    //
    // leftMotor.setSpeed(FORWARD_SPEED);
    // rightMotor.setSpeed(FORWARD_SPEED + 2);
    //
    // leftMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 0.7), true);
    // rightMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 0.7), false);
    //
    // leftMotor.setSpeed(TURNING_SPEED);
    // rightMotor.setSpeed(TURNING_SPEED);
    //
    // // turn right 90 degree
    // leftMotor.rotate(convertAngle(wheelRad, track, 90), true);
    // rightMotor.rotate(-convertAngle(wheelRad, track, 90), false);
    //
    // // pass through the can continue the navigation
    // double yCoordinate = Math.round(((float) odo.getXYT()[1] / (float) TILE_SIZE));
    // if (yCoordinate == URY || yCoordinate == LLY)
    // aCanOnBoundary = true;
  }

  /**
   * This method drives the robot to leave the searching area.
   */
  public void leaveSearchingArea() {

  }

  /**
   * This method checks whether the robot has reached the current destination.
   * 
   * @return false if hasn't reached the current destination; true otherwise
   */
  public boolean checkIfDone(double destinationX, double destinationY) {
    double pos[] = odo.getXYT();
    if (Math.abs(destinationX * TILE_SIZE - pos[0]) < navigationAccuracy
        && Math.abs(destinationY * TILE_SIZE - pos[1]) < navigationAccuracy) {
      // y offset
      // leftMotor.rotate(convertDistance(wheelRad, 1.5),true);
      // rightMotor.rotate(convertDistance(wheelRad,1.5), false);
      return true;
    } else {
      return false;
    }
  }



  /**
   * This method fetches data from the ultraconic sensor and update the distance.
   */
  @Override
  public void timedOut() {
    sampleProvider.fetchSample(sample, 0); // distance from ultrasonic sensor in m
    usDistance = (int) (sample[0] * 100); // convert to cm

    if (state == State.SCANNING) {
      int filter_control = 15;
      int boundary = 32 * 2;
      if (usDistance < boundary) {

        if (filter_count == 0) {
          previousUsDistance = usDistance;
          dataset = new ArrayList<Double>();
        }

        if (Math.abs(previousUsDistance - usDistance) < 5) {
          filter_count++;
          dataset.add(odo.getXYT()[2]);

          if (filter_count == 4) {
            Sound.beep();
          }

        } else {
          filter_count = 0;

          if (dataset.size() >= filter_control) {
            // int mid = dataset.size()/2;
            double midTheta = (dataset.get(0) + dataset.get(dataset.size() - 1)) / 2;
            canLocation.add(midTheta);
          }
          dataset = new ArrayList<Double>();
        }
      } else {
        // outside of boundary

        filter_count = 0;

        if (dataset.size() >= filter_control) {
          double midTheta = (dataset.get(0) + dataset.get(dataset.size() - 1)) / 2;
          canLocation.add(midTheta);
        }
        dataset = new ArrayList<Double>();
      }
    }
    previousUsDistance = usDistance;
  }


  /**
   * The robot will rotate 360 degree to detect whether there are cans.
   */
  public void scan() {

    leftMotor.setSpeed(TURNING_SPEED);
    rightMotor.setSpeed(TURNING_SPEED);
    leftMotor.rotate(convertAngle(wheelRad, track, 360), true);
    rightMotor.rotate(-convertAngle(wheelRad, track, 360), false);

    timer.stop();

    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // for (Double data : canLocation) {
    // lcd.drawString(Double.toString(data), 0, 0);
    // System.out.println(Double.toString(data));
    // turnTo(data - 3);
    // Sound.beep();
    // sampleProvider.fetchSample(sample, 0);
    // usDistance = (int) (sample[0] * 100); // convert to cm
    // System.out.println(usDistance);
    // startColorDetection();
    // }



  }

  /**
   * This method will navigate the robot to pass the tunnel and reach the lower left corner of the
   * searching area.
   * 
   * @param LL_X: x coordinate of the lower left corner of the tunnel
   * @param LL_Y: y coordinate of the lower left corner of the tunnel
   * @param UR_X: x coordinate of the upper right corner of the tunnel
   * @param UR_Y: y coordinate of the upper right corner of the tunnel
   * @param corner: corner number of the starting point
   * @param SZ_X: x coordinate of the lower left corner of the searching area
   * @param SZ_Y: y coordinate of the lower left corner of the searching area
   */
  public void navigateToSearchingArea(int LL_X, int LL_Y, int UR_X, int UR_Y, int corner, int SZ_X,
      int SZ_Y) {
    double[][] destinations = {{0, 0}, {0, 0}, {0, 0}};
    if (LL_Y + 2 == UR_Y && LL_X + 1 == UR_X) {
      if (corner == 2 || corner == 3) {
        destinations[0][0] = LL_X + 0.45;
        destinations[0][1] = UR_Y + 0.5;
        destinations[1][0] = LL_X + 0.45;
        destinations[1][1] = LL_Y - 0.5;
      } else if (corner == 1 || corner == 0) {
        destinations[0][0] = LL_X + 0.55;
        destinations[0][1] = LL_Y - 0.5;
        destinations[1][0] = LL_X + 0.55;
        destinations[1][1] = UR_Y + 0.5;
      }
    } else if (LL_Y + 1 == UR_Y && LL_X + 2 == UR_X) {
      if (corner == 1 || corner == 2) {
        destinations[0][0] = UR_X + 0.5;
        destinations[0][1] = UR_Y - 0.45;
        destinations[1][0] = LL_X - 0.5;
        destinations[1][1] = UR_Y - 0.45;
      } else if (corner == 0 || corner == 3) {
        destinations[0][0] = LL_X - 0.5;
        destinations[0][1] = LL_Y + 0.45;
        destinations[1][0] = UR_X + 0.5;
        destinations[1][1] = LL_Y + 0.45;
      }
    }

    destinations[2][0] = SZ_X;
    destinations[2][1] = SZ_Y;

    for (int i = 0; i < 3; i++) {
      while (!checkIfDone(destinations[i][0], destinations[i][1])) {
        travelTo(destinations[i][0], destinations[i][1]);
      }
      leftMotor.rotate(convertDistance(wheelRad, 2), true);
      rightMotor.rotate(convertDistance(wheelRad, 2), false);
    }
    turnTo(0);
    leftMotor.stop(true);
    rightMotor.stop(false);
  }

}

