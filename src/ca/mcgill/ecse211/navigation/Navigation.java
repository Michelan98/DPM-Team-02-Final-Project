package ca.mcgill.ecse211.navigation;

import java.util.ArrayList;
import ca.mcgill.ecse211.WiFi.WiFi;
import ca.mcgill.ecse211.canGrabbing.CanGrabbing;
import ca.mcgill.ecse211.colorClassification.ColorClassification;
import ca.mcgill.ecse211.entryPoint.EntryPoint;
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
public class Navigation implements TimerListener, Runnable {
  private static final int FORWARD_SPEED = EntryPoint.FORWARD_SPEED;
  private static final int TURNING_SPEED = EntryPoint.TURNING_SPEED;
  private static final int RETURN_SPEED = EntryPoint.RETURN_SPEED;
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

  private Timer timer;
  private static ColorClassification colorClassification = null;

  private double currentDestination[] = {0, 0};
  private static double searchPoint[][] = {{-1, -1}};
  private static double boundary = -1;
  private static boolean oneSearchPoint = false;

  private static volatile int usDistance;
  private int filter_count = 0;
  private static int previousUsDistance = 0;

  private enum State {
    INIT, TRAVELLING, SCANNING, COLORDETECTION, GRABBINGCAN
  };

  private static State state;
  private final double navigationAccuracy = 2.5;
  private final double thetaAccuracy = 3;

  private static ArrayList<Double> dataset;

  private OdometryCorrection odometryCorrection;
  private LightLocalizer lightLocalizer;

  // destinaiton: 0: correction point at the entry of the tunnel, 1: correction point at the exit of
  // the tunnel, 2: ll_sz
  static double[][] destinations = {{0, 0}, {0, 0}, {0, 0}};

  // correction angle: 0: first correction angle, 1: second correction angle, 2: first correction
  // angle when returning, 3: second correction angle when returning
  double[] correctionAngles = {0, 0, 0, 0};
  boolean xFirst = true;



  CanGrabbing canGrabbing = null;
  boolean terminateStateMachine = false;
  private static boolean foundACan = false;

  private static boolean corrected = false;

  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
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

    state = State.INIT;
    dataset = new ArrayList<Double>();

    this.odometryCorrection = odometryCorrection;

    this.lightLocalizer = lightLocalizer;


    canGrabbing = new CanGrabbing(EntryPoint.canGrabbingMotor, EntryPoint.sensorMotor);
  }

  /**
   * This method is inherited from the runnable interface. It is the entry point of the thread.
   * State transition happens in this method.
   */
  public void run() {

    try {
      timer = new Timer(100,
          new Navigation(leftMotor, rightMotor, track, wheelRad, TN_LL_X, TN_LL_Y,
              TN_UR_X, TN_UR_Y, SZ_LL_X, SZ_LL_Y, SZ_UR_X, SZ_UR_Y, corner, sensorMotor, lcd, TR,
              sampleProvider, odometryCorrection, lightLocalizer));
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
    timer.start();


    // set the current destination to the center of the searching area
    currentDestination[0] = searchPoint[0][0];
    currentDestination[1] = searchPoint[0][1];

    // distance from the search point to the can
    double distance = 0;

    while (true) {

      switch (state) {
        case INIT: {
          if (!isNavigating()) {
            //start traveling
            state = State.TRAVELLING;
          }
          break;
        }
        case TRAVELLING: {
          if (!checkIfDone(currentDestination[0], currentDestination[1])) {
            travelTo(currentDestination[0], currentDestination[1], true);
          } else if (checkIfDone(currentDestination[0], currentDestination[1])) {
            //has reached the search point
            turnTo(0);
            leftMotor.stop();
            rightMotor.stop();
            releaseCan();
            state = State.SCANNING;
          }

          break;
        }
        case SCANNING: {
          scan();
          break;
        }
        case COLORDETECTION: {

          sampleProvider.fetchSample(sample, 0);
          usDistance = (int) (sample[0] * 100); // convert to cm
          distance = startColorDetection();

          break;

        }
        case GRABBINGCAN: {
          startCanGrabbing();
          usDistance = (int) (sample[0] * 100); // convert to cm

          if (usDistance < 7) {
            // is a can, terminate the state machine
            terminateStateMachine = true;
          } else {
            // didn't grab a can, travel back to the search point and transit to scanning state
            leftMotor.rotate(-convertDistance(wheelRad, distance), true);
            rightMotor.rotate(-convertDistance(wheelRad, distance), false);
            //turn 10 degrees away from the bad position
            leftMotor.rotate(convertAngle(wheelRad, track, 10), true);
            rightMotor.rotate(-convertAngle(wheelRad, track, 10), false);
            releaseCan();
            state = State.SCANNING;
          }

          break;
        }
      }
      if (terminateStateMachine) {
        // terminate the state machine
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

    // set different speed for returning with a can (added weight)
    if (terminateStateMachine) {
      leftMotor.setSpeed(RETURN_SPEED);
      rightMotor.setSpeed(RETURN_SPEED);
    } else {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
    }

    // run along the line
    double distanceX = x * TILE_SIZE - currentX;
    double distanceY = y * TILE_SIZE - currentY;

    if (xFirst) {
      // go along x axis first
      if (Math.abs(distanceX) > navigationAccuracy) {
        if (distanceX < 0 && Math.abs(theta - 270) > thetaAccuracy) {
          turnTo(270);
        } else if (distanceX > 0 && Math.abs(theta - 90) > thetaAccuracy) {
          turnTo(90);
        }


        leftMotor.forward();
        rightMotor.forward();

      } else {
        // x is done, going along y axis now

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
        // go along y axis first

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
        // y is done, going along x axis now


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

    // set different speed for returning with a can (added weight)
    if (terminateStateMachine) {
      leftMotor.setSpeed(RETURN_SPEED);
      rightMotor.setSpeed(RETURN_SPEED);
    } else {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
    }

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

    // travel to the can
    leftMotor.rotate(convertDistance(wheelRad, distance), true);
    rightMotor.rotate(convertDistance(wheelRad, distance), false);

    leftMotor.stop(true);
    rightMotor.stop(false);
    // reach the can and start the colorDetection

    Thread colorThread = new Thread() {
      public void run() {
        try {
          colorClassification.colorClassify();
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

    state = State.GRABBINGCAN;

    // the distance from the search point to the can
    return distance;


  }

  /**
   * This method starts can grabbing process.
   */
  public void startCanGrabbing() {
    canGrabbing.grabCan();
  }

  /**
   * This method releases the can.
   */
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
    // if the current distance to the destination is smaller than the threshold, the robot is
    // considered to has reached the destination
    if (Math.abs(destinationX * TILE_SIZE - pos[0]) < navigationAccuracy
        && Math.abs(destinationY * TILE_SIZE - pos[1]) < navigationAccuracy) {
      corrected = false;
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
    //distance from ultrasonic sensor in m
    sampleProvider.fetchSample(sample, 0); 
    usDistance = (int) (sample[0] * 100); // convert to cm

    if (state == State.SCANNING) {
      // fetch data from ultrasonic sensor during SCANNING state to find a can

      int filter_control = 6;

      if (usDistance < boundary) {
        //usDistance smaller than boundary means there is a can; the boundary filter out the readings from walls
        if (filter_count == 0) {
          previousUsDistance = usDistance;
          dataset = new ArrayList<Double>();
        }

        //this is a filter: if getting 5 consecutive similar readings, the reading is considered as a valid reading
        if (Math.abs(previousUsDistance - usDistance) < 5) {
          filter_count++;
          dataset.add(odo.getXYT()[2]);

          if (filter_count > filter_control) {
            //reset the filter_control, in case there is another scanning
            filter_count = 0;
            foundACan = true;
          }

        } else {
          //inconsistent readings, reset the filter_count
          filter_count = 0;
        }
      } else {
        // outside of boundary, reset the filter_count
        filter_count = 0;
      }
    }
    previousUsDistance = usDistance;
  }


  /**
   * The robot will rotate 360 degree to detect whether there are cans.
   */
  public void scan() {

    leftMotor.setSpeed(EntryPoint.SCANNING_SPEED);
    rightMotor.setSpeed(EntryPoint.SCANNING_SPEED);
    while (!foundACan) {
      //the robot rotates clockwise until it finds a can
      leftMotor.forward();
      rightMotor.backward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    //reset foundACan, in case there is another scanning
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

    // travel to the first correction point
    while (!checkIfDone(destinations[0][0], destinations[0][1])) {
      travelTo(destinations[0][0], destinations[0][1], xFirst);
    }


    // do correction at the first correction point
    turnTo(correctionAngles[0]);
    odometryCorrection.correct(correctionAngles[0]);
    double forwardDistance = TILE_SIZE / 2 - odometryCorrection.DISTANCE_TO_SENSOR;
    leftMotor.rotate(convertDistance(wheelRad, forwardDistance), true);
    rightMotor.rotate(convertDistance(wheelRad, forwardDistance), false);

    turnTo(correctionAngles[1]);
    odometryCorrection.correct(correctionAngles[1]);
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
    turnTo((correctionAngles[0] + 180) % 360);
    odometryCorrection.correct((correctionAngles[0] + 180) % 360);

    // travel to ll_sz
    while (!checkIfDone(destinations[2][0], destinations[2][1])) {
      travelTo(destinations[2][0], destinations[2][1], xFirst);
    }

    turnTo(0);
    lightLocalizer.startLocalize(destinations[2][0] * TILE_SIZE, destinations[2][1] * TILE_SIZE, 0);
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

    // travel to the ll +1 and localize at the point
    while (!checkIfDone(destinations[2][0], destinations[2][1])) {
      travelTo(destinations[2][0], destinations[2][1], !xFirst);
    }
    turnTo(0);
    lightLocalizer.startLocalize(destinations[2][0] * TILE_SIZE, (destinations[2][1]) * TILE_SIZE,
        0);

    // travel to the second correction point
    while (!checkIfDone(destinations[1][0], destinations[1][1])) {
      travelTo(destinations[1][0], destinations[1][1], !xFirst);
    }

    // do correction at the second correction point
    turnTo(correctionAngles[2]);
    odometryCorrection.correct(correctionAngles[2]);
    double forwardDistance = TILE_SIZE / 2 - odometryCorrection.DISTANCE_TO_SENSOR;
    leftMotor.rotate(convertDistance(wheelRad, forwardDistance), true);
    rightMotor.rotate(convertDistance(wheelRad, forwardDistance), false);

    turnTo(correctionAngles[3]);
    odometryCorrection.correct(correctionAngles[3]);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    //passing through the tunnel
    leftMotor.rotate(convertDistance(wheelRad,
        3 * TILE_SIZE + (TILE_SIZE - odometryCorrection.DISTANCE_TO_SENSOR)), true);
    rightMotor.rotate(convertDistance(wheelRad,
        3 * TILE_SIZE + (TILE_SIZE - odometryCorrection.DISTANCE_TO_SENSOR)), false);
    // exit the tunnel

    // correction after the tunnel
    odometryCorrection.correct(correctionAngles[3]);
    leftMotor.rotate(-convertDistance(wheelRad, odometryCorrection.DISTANCE_TO_SENSOR), true);
    rightMotor.rotate(-convertDistance(wheelRad, odometryCorrection.DISTANCE_TO_SENSOR), false);
    turnTo((correctionAngles[2] + 180) % 360);
    odometryCorrection.correct((correctionAngles[2] + 180) % 360);


    // travel to the starting point
    while (!checkIfDone(WiFi.localizeX, WiFi.localizeY)) {
      travelTo(WiFi.localizeX, WiFi.localizeY, !xFirst);
    }
    leftMotor.rotate(convertDistance(wheelRad, 2), true);
    rightMotor.rotate(convertDistance(wheelRad, 2), false);

  }

  /**
   * initialize key points in the path to the searching area and correction angle before and after
   * passing the tunnel. The key points are determined based on the tunnel orientation and the position.
   */
  public void initializePointsAndAngle() {
    int navigationCase = -1;

    int halfOfX = EntryPoint.BOARD_X / 2;
    int halfOfY = EntryPoint.BOARD_Y / 2;
    System.out.println(halfOfX + " " + halfOfY);
    if (TN_LL_Y + 2 == TN_UR_Y && TN_LL_X + 1 == TN_UR_X) {
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
        if (TN_LL_Y < halfOfY) {
          destinations[0][0] = TN_UR_X + 1;
          destinations[0][1] = TN_UR_Y;
          destinations[1][0] = TN_LL_X - 1;
          destinations[1][1] = TN_UR_Y;
          correctionAngles[0] = 180;
          correctionAngles[1] = 270;
          correctionAngles[2] = 180;
          correctionAngles[3] = 90;
        }
        if (TN_LL_Y >= halfOfY) {
          destinations[0][0] = TN_UR_X + 1;
          destinations[0][1] = TN_LL_Y;
          destinations[1][0] = TN_LL_X - 1;
          destinations[1][1] = TN_LL_Y;
          correctionAngles[0] = 0;
          correctionAngles[1] = 270;
          correctionAngles[2] = 0;
          correctionAngles[3] = 90;
        }
        if (TN_LL_Y == 3) {
          destinations[0][1] = TN_LL_Y;
          destinations[1][1] = TN_LL_Y;
          correctionAngles[0] = 0;
          correctionAngles[1] = 270;
          correctionAngles[2] = 0;
          correctionAngles[3] = 90;
        }
        if (TN_LL_Y == 5) {
          destinations[0][1] = TN_UR_Y;
          destinations[1][1] = TN_UR_Y;
          correctionAngles[0] = 180;
          correctionAngles[1] = 270;
          correctionAngles[2] = 180;
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
        if (TN_LL_Y >= halfOfY) {
          destinations[0][0] = TN_LL_X - 1;
          destinations[0][1] = TN_LL_Y;
          destinations[1][0] = TN_UR_X + 1;
          destinations[1][1] = TN_LL_Y;
          correctionAngles[0] = 0;
          correctionAngles[1] = 90;
          correctionAngles[2] = 0;
          correctionAngles[3] = 270;
        }
        if (TN_LL_Y == 3) {
          destinations[0][1] = TN_LL_Y;
          destinations[1][1] = TN_LL_Y;
          correctionAngles[0] = 0;
          correctionAngles[1] = 90;
          correctionAngles[2] = 0;
          correctionAngles[3] = 270;
        }
        if (TN_LL_Y == 5) {
          destinations[0][1] = TN_UR_Y;
          destinations[1][1] = TN_UR_Y;
          correctionAngles[0] = 180;
          correctionAngles[1] = 90;
          correctionAngles[2] = 180;
          correctionAngles[3] = 270;
        }
      }
    }

    //the lower left corner of the search area
    destinations[2][0] = SZ_LL_X;
    destinations[2][1] = SZ_LL_Y;


    if (navigationCase == 3 || navigationCase == 2) {
      //determine whether go along x first or y first based on different cases
      xFirst = false;
    }


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

    if (SZ_UR_X == EntryPoint.BOARD_X) {
      virtual_UR_X--;
    }
    if (SZ_LL_X == 0) {
      virtual_LL_X++;
    }
    if (SZ_LL_Y == 0) {
      virtual_LL_Y++;
    }
    if (SZ_UR_Y == EntryPoint.BOARD_Y) {
      virtual_UR_Y--;
    }

    searchPoint[0][0] = (virtual_UR_X + virtual_LL_X) / 2;
    searchPoint[0][1] = (virtual_UR_Y + virtual_LL_Y) / 2;
    
    //boundary for the scanning
    boundary =
        Math.min((virtual_UR_X - virtual_LL_X), (virtual_UR_Y - virtual_LL_Y)) / 2 * TILE_SIZE;


  }




}
