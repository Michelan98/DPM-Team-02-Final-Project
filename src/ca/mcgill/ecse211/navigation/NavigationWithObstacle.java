package ca.mcgill.ecse211.navigation;

import java.util.ArrayList;
import ca.mcgill.ecse211.WiFi.WiFi;
import ca.mcgill.ecse211.canGrabbing.CanGrabbing;
import ca.mcgill.ecse211.colorClassification.ColorClassification;
import ca.mcgill.ecse211.entryPoint.Lab5;
import ca.mcgill.ecse211.localization.LightLocalizer;
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
  private static final int FORWARD_SPEED = Lab5.FORWARD_SPEED;
  private static final int TURNING_SPEED = Lab5.TURNING_SPEED;
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
  private int SZ_LL_Y;
  private int SZ_LL_X;
  private int SZ_UR_X;
  private int SZ_UR_Y;
  private int TN_LL_X;
  private int TN_LL_Y;
  private int TN_UR_X;
  private int TN_UR_Y;
  private int corner;


  private static Odometer odo = null;
  private int count = 0; // use it to track which point to go

  private static int flag = 0;
  private Timer timer;
  private static ColorClassification colorClassification = null;

  private double currentDestination[] = {0, 0};
  private static double searchPoint[][] = {{-1, -1}};
  private static double boundary = -1;
  private static boolean oneSearchPoint = false;
  private boolean isTargetCan = false;

  private static volatile int usDistance;
  private int filter_count = 0;
  private static int previousUsDistance = 0;

  private enum State {
    INIT, TRAVELLING, SCANNING, COLORDETECTION, AVOIDINGCAN, GRABBINGCAN
  };

  private static State state;
  private final double navigationAccuracy = 2.5;
  private final double thetaAccuracy = 3;

  private volatile static double canLocation; // the angle of can to the robot
  private static ArrayList<Double> dataset;

  private OdometryCorrection odometryCorrection;
  private LightLocalizer lightLocalizer;

  static double[][] destinations = {{0, 0}, {0, 0}, {0, 0}};
  double[] correctionAngles = {0, 0, 0, 0};
  boolean xFirst = true;
  // destinaiton: 0: correction point at the entry of the tunnel, 1: correction point at the exit of
  // the tunnel, 2: ll_sz

  // correction angle: 0: first correction angle, 1: second correction angle, ...



  CanGrabbing canGrabbing = null;
  boolean terminateStateMachine = false;
  private static boolean foundACan = false;

  public NavigationWithObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double track, double wheelRad, int TN_LL_X, int TN_LL_Y, int TN_UR_X, int TN_UR_Y, int LLX,
      int LLY, int URX, int URY, int cornerNum, EV3LargeRegulatedMotor sensorMotor, TextLCD lcd,

      int TR, SampleProvider sampleProvider, OdometryCorrection odometryCorrection,
      LightLocalizer lightLocalizer) throws OdometerExceptions {


    odo = Odometer.getOdometer();
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.leftMotor.setAcceleration(500);
    this.rightMotor.setAcceleration(500);

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    this.track = track;
    this.wheelRad = wheelRad;
    this.TR = 4;
    this.lcd = lcd;
    this.sensorMotor = sensorMotor;

    this.SZ_LL_X = LLX;
    this.SZ_LL_Y = LLY;
    this.SZ_UR_X = URX;
    this.SZ_UR_Y = URY;

    this.TN_LL_X = TN_LL_X;
    this.TN_LL_Y = TN_LL_Y;
    this.TN_UR_X = TN_UR_X;
    this.TN_UR_Y = TN_UR_Y;

    this.corner = cornerNum;

    currentDestination[0] = SZ_LL_X;
    currentDestination[1] = SZ_LL_Y;

    colorClassification = new ColorClassification(sensorMotor, lcd, TR);

    this.sampleProvider = sampleProvider;
    sample = new float[sampleProvider.sampleSize()];

    isNavigating = false;

    canLocation = 0;

    state = State.INIT;
    dataset = new ArrayList<Double>();

    this.odometryCorrection = odometryCorrection;

    this.lightLocalizer = lightLocalizer;


    canGrabbing = new CanGrabbing(Lab5.canGrabbingMotor);
  }

  /**
   * This method is inherited from the runnable interface. It is the entry point of the thread.
   * State transition happens in this method.
   */
  public void run() {

    try {
      timer = new Timer(100,
          new NavigationWithObstacle(leftMotor, rightMotor, track, wheelRad, TN_LL_X, TN_LL_Y,
              TN_UR_X, TN_UR_Y, SZ_LL_X, SZ_LL_Y, SZ_UR_X, SZ_UR_Y, corner, sensorMotor, lcd, TR,
              sampleProvider, odometryCorrection, lightLocalizer));
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    timer.start();


    // travel to the center of the searching area
    currentDestination[0] = searchPoint[0][0];
    currentDestination[1] = searchPoint[0][1];


    double distance = 0;

    while (true) {
      System.out.println("state " + state);

      switch (state) {
        case INIT: {
          if (!isNavigating()) {
            state = State.TRAVELLING;
          }
          break;
        }
        case TRAVELLING: {
          if (!checkIfDone(currentDestination[0], currentDestination[1])) {
            travelTo(currentDestination[0], currentDestination[1], true);
          } else if (checkIfDone(currentDestination[0], currentDestination[1])) {
            turnTo(0);
            leftMotor.stop();
            rightMotor.stop();
            releaseCan();
            state = State.SCANNING;
          }

          break;
        }
        case SCANNING: {
          System.out.println("start scaning");
          scan();
          break;
        }
        case COLORDETECTION: {
          // System.out.println("color detection state");

          sampleProvider.fetchSample(sample, 0);
          usDistance = (int) (sample[0] * 100); // convert to cm
          distance = startColorDetection();
          state = State.GRABBINGCAN;

          break;

        }
        case GRABBINGCAN: {
          startCanGrabbing();
          usDistance = (int) (sample[0] * 100); // convert to cm
          System.out.println("usD " + usDistance);

          if(usDistance <7) {
          // is a can
          terminateStateMachine = true;
          }
          else {
            leftMotor.rotate(-convertDistance(wheelRad, distance), true);
            rightMotor.rotate(-convertDistance(wheelRad, distance), false);
            releaseCan();
            state = State.SCANNING;
          }


          break;
        }
      }
      if (terminateStateMachine) {
        break;
      }


    }
    timer.stop();
  }


  /**
   * This method cause the robot to travel to the absolute field location (x,y), and the robot will
   * avoid obstacles it meets during the travel.
   * 
   * @param x: the x coordinate of the point
   * @param y: the y coordinate of the point
   */
  public void travelTo(double x, double y, boolean xFirst) {


    // get the position reading from the odometer
    double pos[] = odo.getXYT();
    double currentX = pos[0];
    double currentY = pos[1];
    double theta = pos[2];

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // run along the line
    double distanceX = x * TILE_SIZE - currentX;
    double distanceY = y * TILE_SIZE - currentY;

    if (xFirst) {
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
    } else {
      if (Math.abs(distanceY) > navigationAccuracy) {


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


      } else {
        // x is done
        if (distanceX < 0 && Math.abs(theta - 270) > thetaAccuracy) {
          turnTo(270);
        } else if (distanceX > 0 && Math.abs(theta - 90) > thetaAccuracy) {
          turnTo(90);
        }

        leftMotor.forward();
        rightMotor.forward();
      }
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
   * 
   * @return distance: return the distance from the scanning center to the can
   */
  private double startColorDetection() {

    int distance = usDistance;

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(convertDistance(wheelRad, distance), true);
    rightMotor.rotate(convertDistance(wheelRad, distance), false);

    leftMotor.stop(true);
    rightMotor.stop(false);
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

    return distance;

    // go back to the center
    // leftMotor.rotate(-convertDistance(wheelRad, distance), true);
    // rightMotor.rotate(-convertDistance(wheelRad, distance), false);

  }

  /**
   * This method starts can grabbing process.
   */
  public void startCanGrabbing() {
    canGrabbing.grabCan();
  }

  public void releaseCan() {
    canGrabbing.releaseCan();
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
      System.out.println(usDistance + " filter count" + filter_count);
      int filter_control = 6;

      if (usDistance < boundary) {

        if (filter_count == 0) {
          previousUsDistance = usDistance;
          dataset = new ArrayList<Double>();
        }

        if (Math.abs(previousUsDistance - usDistance) < 5) {
          filter_count++;
          dataset.add(odo.getXYT()[2]);

          if (filter_count > filter_control) {
            foundACan = true;
          }

        } else {
          filter_count = 0;

        }
      } else {
        // outside of boundary
        filter_count = 0;
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
    while (!foundACan) {
      leftMotor.forward();
    }
    leftMotor.stop(false);
    foundACan = false;
    state = State.COLORDETECTION;
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
  public void navigateToSearchingArea() {
    System.out.println(destinations[0][0] + " " + destinations[0][1]);
    // travel to the first correction point
    while (!checkIfDone(destinations[0][0], destinations[0][1])) {
      travelTo(destinations[0][0], destinations[0][1], xFirst);
    }


    // do correction at the first correction point
    System.out.println("doing correction");
    turnTo(correctionAngles[0]);
    odometryCorrection.correct(correctionAngles[0]);
    System.out.println(
        "odometer correction" + odo.getXYT()[0] + "," + odo.getXYT()[1] + "," + odo.getXYT()[2]);
    double forwardDistance = TILE_SIZE / 2 - odometryCorrection.DISTANCE_TO_SENSOR;
    leftMotor.rotate(convertDistance(wheelRad, forwardDistance), true);
    rightMotor.rotate(convertDistance(wheelRad, forwardDistance), false);

    turnTo(correctionAngles[1]);
    odometryCorrection.correct(correctionAngles[1]);
    System.out.println(
        "odometer correction" + odo.getXYT()[0] + "," + odo.getXYT()[1] + "," + odo.getXYT()[2]);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(wheelRad,
        3 * TILE_SIZE + (TILE_SIZE - odometryCorrection.DISTANCE_TO_SENSOR)), true);
    rightMotor.rotate(convertDistance(wheelRad,
        3 * TILE_SIZE + (TILE_SIZE - odometryCorrection.DISTANCE_TO_SENSOR)), false);
    // exit the tunnel

    // correction
    odometryCorrection.correct(correctionAngles[1]);
    leftMotor.rotate(-convertDistance(wheelRad, odometryCorrection.DISTANCE_TO_SENSOR), true);
    rightMotor.rotate(-convertDistance(wheelRad, odometryCorrection.DISTANCE_TO_SENSOR), false);
    System.out.println(
        "odometer correction" + odo.getXYT()[0] + "," + odo.getXYT()[1] + "," + odo.getXYT()[2]);
    turnTo((correctionAngles[0] + 180) % 360);
    odometryCorrection.correct((correctionAngles[0] + 180) % 360);
    
    System.out.println(
        "odometer correction" + odo.getXYT()[0] + "," + odo.getXYT()[1] + "," + odo.getXYT()[2]);

    // travel to ll_sz
    System.out.println(destinations[2][0] + " " + destinations[2][1]);
    while (!checkIfDone(destinations[2][0], destinations[2][1])) {
      travelTo(destinations[2][0], destinations[2][1], xFirst);
    }

    turnTo(0);
    leftMotor.stop(true);
    rightMotor.stop(false);
    Sound.beep();
    Sound.beep();
    Sound.beep();

    // close the thing and push cans away
     startCanGrabbing();

  }



  /**
   * This method drives the robot to leave the searching area.
   */
  public void leaveSearchingArea() {
    // travel to the first correction point
    System.out.println("second localization point" + destinations[1][0] + "," + destinations[1][1]);
    while (!checkIfDone(destinations[1][0], destinations[1][1])) {
      travelTo(destinations[1][0], destinations[1][1], !xFirst);
    }

    // do correction at the first correction point
    System.out.println("doing correction");
    turnTo(correctionAngles[2]);
    odometryCorrection.correct(correctionAngles[2]);
    double forwardDistance = TILE_SIZE / 2 - odometryCorrection.DISTANCE_TO_SENSOR;
    leftMotor.rotate(convertDistance(wheelRad, forwardDistance), true);
    rightMotor.rotate(convertDistance(wheelRad, forwardDistance), false);

    turnTo(correctionAngles[3]);
    odometryCorrection.correct(correctionAngles[3]);
    leftMotor.rotate(convertDistance(wheelRad,
        3 * TILE_SIZE + (TILE_SIZE - odometryCorrection.DISTANCE_TO_SENSOR)), true);
    rightMotor.rotate(convertDistance(wheelRad,
        3 * TILE_SIZE + (TILE_SIZE - odometryCorrection.DISTANCE_TO_SENSOR)), false);
    // exit the tunnel

    // travel to starting point
    System.out.println("starting point" + WiFi.localizeX + "," + WiFi.localizeY);
    while (!checkIfDone(WiFi.localizeX, WiFi.localizeY)) {
      travelTo(WiFi.localizeX, WiFi.localizeY, !xFirst);
    }
    leftMotor.rotate(convertDistance(wheelRad, 2), true);
    rightMotor.rotate(convertDistance(wheelRad, 2), false);

    releaseCan();
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();
    Sound.beep();
  }

  /**
   * initialize key points in the path to the searching area and correction angle before and after
   * passing the tunnel
   */
  public void initializePointsAndAngle() {
    int navigationCase = -1;

    int halfOfX = Lab5.BOARD_X / 2;
    int halfOfY = Lab5.BOARD_Y / 2;
    System.out.println(halfOfX + " " + halfOfY);
    if (TN_LL_Y + 2 == TN_UR_Y && TN_LL_X + 1 == TN_UR_X) { // how to handle 1 square tile??
      if (corner == 2 || corner == 3) {
        // case 1
        navigationCase = 1;
        if (TN_LL_X > halfOfX) {
          destinations[0][0] = TN_LL_X;
          destinations[0][1] = TN_UR_Y + 1;
          destinations[1][0] = TN_LL_X;
          destinations[1][1] = TN_LL_Y - 1;
          correctionAngles[0] = 90;
          correctionAngles[1] = 180;
          correctionAngles[2] = 90;
          correctionAngles[3] = 0;
        }
        if (TN_LL_X <= halfOfX) {
          destinations[0][0] = TN_UR_X;
          destinations[0][1] = TN_UR_Y + 1;
          destinations[1][0] = TN_UR_X;
          destinations[1][1] = TN_LL_Y - 1;
          correctionAngles[0] = 270;
          correctionAngles[1] = 180;
          correctionAngles[2] = 270;
          correctionAngles[3] = 0;
        }
      } else if (corner == 1 || corner == 0) {
        // case 4
        navigationCase = 4;
        if (TN_LL_X <= halfOfX) {
          destinations[0][0] = TN_UR_X;
          destinations[0][1] = TN_LL_Y - 1;
          destinations[1][0] = TN_UR_X;
          destinations[1][1] = TN_UR_Y + 1;
          correctionAngles[0] = 270;
          correctionAngles[1] = 0;
          correctionAngles[2] = 270;
          correctionAngles[3] = 180;
        }
        if (TN_LL_X > halfOfX) {
          destinations[0][0] = TN_LL_X;
          destinations[0][1] = TN_LL_Y - 1;
          destinations[1][0] = TN_LL_X;
          destinations[1][1] = TN_UR_Y + 1;
          correctionAngles[0] = 90;
          correctionAngles[1] = 0;
          correctionAngles[2] = 90;
          correctionAngles[3] = 180;
        }

      }
    } else if (TN_LL_Y + 1 == TN_UR_Y && TN_LL_X + 2 == TN_UR_X) {

      if (corner == 1 || corner == 2) {
        // case 2
        navigationCase = 2;
        if (TN_LL_Y <= halfOfY) {
          destinations[0][0] = TN_UR_X + 1;
          destinations[0][1] = TN_UR_Y;
          destinations[1][0] = TN_LL_X - 1;
          destinations[1][1] = TN_UR_Y;
          correctionAngles[0] = 180;
          correctionAngles[1] = 270;
          correctionAngles[2] = 180;
          correctionAngles[3] = 90;
        }
        if (TN_LL_Y > halfOfY) {
          destinations[0][0] = TN_UR_X + 1;
          destinations[0][1] = TN_LL_Y;
          destinations[1][0] = TN_LL_X - 1;
          destinations[1][1] = TN_LL_Y;
          correctionAngles[0] = 0;
          correctionAngles[1] = 270;
          correctionAngles[2] = 0;
          correctionAngles[3] = 90;
        }
      } else if (corner == 0 || corner == 3) {
        // case 3
        navigationCase = 3;
        if (TN_LL_Y < halfOfY) {
          destinations[0][0] = TN_LL_X - 1;
          destinations[0][1] = TN_UR_Y;
          destinations[1][0] = TN_UR_X + 1;
          destinations[1][1] = TN_UR_Y;
          correctionAngles[0] = 180;
          correctionAngles[1] = 90;
          correctionAngles[2] = 180;
          correctionAngles[3] = 270;
        }
        if (TN_LL_Y > halfOfY) {
          destinations[0][0] = TN_LL_X - 1;
          destinations[0][1] = TN_LL_Y;
          destinations[1][0] = TN_UR_X + 1;
          destinations[1][1] = TN_LL_Y;
          correctionAngles[0] = 0;
          correctionAngles[1] = 90;
          correctionAngles[2] = 0;
          correctionAngles[3] = 270;
        }
      }
    }

    // +1 so the robot will not hit the wall if the searching area is right next to the wall
    destinations[2][0] = SZ_LL_X;
    destinations[2][1] = SZ_LL_Y;


    if (navigationCase == 3 || navigationCase == 2) {
      xFirst = false;
    }

    System.out.println(
        "corner" + corner + "travelling to" + destinations[0][0] + " " + destinations[0][1]);

  }

  /**
   * calculate the searching points and boundary of the search
   */
  public void searchPointCalculation() {

    boolean orientation = false;
    double virtual_UR_X = SZ_UR_X;
    double virtual_UR_Y = SZ_UR_Y;
    double virtual_LL_X = SZ_LL_X;
    double virtual_LL_Y = SZ_LL_Y;
    // determine the board's orientation
    if ((SZ_UR_X - SZ_LL_X) / 2 >= (SZ_UR_Y - SZ_LL_Y) / 2) {
      // square or rect
      orientation = true;
    } else {
      // rect
      orientation = false;
    }

    if (SZ_UR_X == Lab5.BOARD_X) {
      virtual_UR_X--;
    }
    if (SZ_LL_X == 0) {
      virtual_LL_X++;
    }
    if (SZ_LL_Y == 0) {
      virtual_LL_Y++;
    }
    if (SZ_UR_Y == Lab5.BOARD_Y) {
      virtual_UR_Y--;
    }

    searchPoint[0][0] = (virtual_UR_X + virtual_LL_X) / 2;
    searchPoint[0][1] = (virtual_UR_Y + virtual_LL_Y) / 2;
    boundary =
        Math.min((virtual_UR_X - virtual_LL_X), (virtual_UR_Y - virtual_LL_Y)) / 2 * TILE_SIZE;


    System.out.println("p1:" + searchPoint[0][0] + "," + searchPoint[0][1] + " boundary" + boundary
        + "oneSearchPoint" + oneSearchPoint);

  }


}
