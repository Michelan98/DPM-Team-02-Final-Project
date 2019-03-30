package ca.mcgill.ecse211.navigation;

import java.util.ArrayList;
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
  private final double navigationAccuracy = 2.5;
  private final double thetaAccuracy = 3;
  private boolean offsetAdded = false;

  private volatile static ArrayList<Double> canLocation; // the angle of can to the robot
  private static ArrayList<Double> dataset;

  private OdometryCorrection odometryCorrection;
  private LightLocalizer lightLocalizer;

  double[][] destinations = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
  boolean xFirst = true;
  // 0: localization point before tunnel, 1: point before tunnel, 2: point after tunnel, 3:
  // localization point after tunnel, 4: lower left corner sz


  CanGrabbing canGrabbing = null;
  boolean terminateStateMachine = false;

  public NavigationWithObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double track, double wheelRad, int TN_LL_X, int TN_LL_Y, int TN_UR_X, int TN_UR_Y, int LLX,
      int LLY, int URX, int URY, int cornerNum, EV3LargeRegulatedMotor sensorMotor, TextLCD lcd,
      int TR, SampleProvider sampleProvider, OdometryCorrection odometryCorrection, LightLocalizer lightLocalizer)
      throws OdometerExceptions {

    odo = Odometer.getOdometer();
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.leftMotor.setAcceleration(500);
    this.rightMotor.setAcceleration(520);

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

    canLocation = new ArrayList<Double>();

    // TODO: don't forget to change it to INIT!!
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
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    timer.start();

    // travel to the center of the searching area
    currentDestination[0] = (SZ_LL_X + SZ_UR_X) / 2;
    currentDestination[1] = (SZ_LL_Y + SZ_UR_Y) / 2;


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

          // else if (OdometryCorrection.oneLineDetected || OdometryCorrection.otherLineDetected){
          // state = State.CORRECTING;
          // System.out.println("transit to correcting");
          // }
          if (!checkIfDone(currentDestination[0], currentDestination[1])) {
            travelTo(currentDestination[0], currentDestination[1], true);
          } else if (checkIfDone(currentDestination[0], currentDestination[1])) {
            turnTo(0);
            leftMotor.stop();
            rightMotor.stop();
            state = State.SCANNING;
          }

          break;
        }
        case CORRECTING: {
          double theta = odo.getXYT()[2];
          double correctedHeading = 0;
          if ((theta < 10 && theta > 0) || (theta > 350 && theta < 360)) {
            correctedHeading = 0;
          } else if (theta < 100 && theta > 80) {
            correctedHeading = 90;
          } else if (theta > 170 && theta < 190) {
            correctedHeading = 180;
          } else if (theta < 280 && theta > 260) {
            correctedHeading = 270;
          }
          odometryCorrection.correct(correctedHeading);
          state = State.TRAVELLING;
          System.out
              .println("odo " + odo.getXYT()[0] + " " + odo.getXYT()[1] + " " + odo.getXYT()[2]);
          System.out.println("going to travelling");
        }
        case SCANNING: {
          releaseCan();
          scan();
          state = State.COLORDETECTION;
        }
        case COLORDETECTION: {

          System.out.println("color detection state");
          System.out.println("size " + canLocation.size());
          for (Double data : canLocation) {
            turnTo(data - 3);
            sampleProvider.fetchSample(sample, 0);
            usDistance = (int) (sample[0] * 100); // convert to cm
            startColorDetection();
            if (isTargetCan) {
              // state = State.GRABBINGCAN;
              // TODO: for beta demo
              Sound.beep();
              Sound.beep();
              Sound.beep();
              Sound.beep();
              Sound.beep();
              Sound.beep();
              Sound.beep();
              Sound.beep();
              Sound.beep();
              Sound.beep();

              break;
            }
          }

          state = State.GRABBINGCAN;

          break;

        }
        case GRABBINGCAN: {
          startCanGrabbing();
//          state = State.LEAVING;
          
          terminateStateMachine = true;
          break;
        }
        case AVOIDINGCAN: {
          canAvoidance();
          state = State.TRAVELLING;
          break;
        }
//        case LEAVING: {
//          // leave the searching area and go to the tunnel
//
//          if (!checkIfDone(currentDestination[0], currentDestination[1])) {
//            travelTo(currentDestination[0], currentDestination[1]);
//          } else if (checkIfDone(currentDestination[0], currentDestination[1])) {
//            // TODO: for beta demo only
//            // turnTo(0);
//            // leftMotor.stop();
//            // rightMotor.stop();
//            // Sound.beep();
//            // Sound.beep();
//            // Sound.beep();
//            // Sound.beep();
//            // Sound.beep();
//          }
//          break;
//        }
      }
      if (terminateStateMachine) {
        break;
      }

      // try {
      // Thread.sleep(50);
      // } catch (InterruptedException e) {
      // // TODO Auto-generated catch block
      // e.printStackTrace();
      // }
    }
//    turnTo(0);
//    leftMotor.stop();
//    rightMotor.stop();
//    Sound.beep();
//    Sound.beep();
//    Sound.beep();
//    Sound.beep();
//    Sound.beep();
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

    if(xFirst) {
    if (Math.abs(distanceX) > navigationAccuracy) {
      if (distanceX < 0 && Math.abs(theta - 270) > thetaAccuracy) {
        turnTo(270);
      } else if (distanceX > 0 && Math.abs(theta - 90) > thetaAccuracy) {
        turnTo(90);
      }

        leftMotor.forward();
        rightMotor.forward();

//      leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] {rightMotor});
//      leftMotor.startSynchronization();
//      leftMotor.forward();
//      rightMotor.forward();
//      leftMotor.endSynchronization();

    } else {
      // x is done
      if (!offsetAdded) {
        // change the 3 to distanceX?
         leftMotor.rotate(convertDistance(wheelRad, distanceX), true);
         rightMotor.rotate(convertDistance(wheelRad, distanceX), false);


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
    else {
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
        if (!offsetAdded) {
           leftMotor.rotate(convertDistance(wheelRad, distanceY), true);
           rightMotor.rotate(convertDistance(wheelRad, distanceY), false);
          offsetAdded = true;
        }
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
    canGrabbing.grabCan();
  }

  public void releaseCan() {
    canGrabbing.releaseCan();
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

    leftMotor.setSpeed(TURNING_SPEED);
    rightMotor.setSpeed(TURNING_SPEED);

    // turn left 90 degree
    leftMotor.rotate(-convertAngle(wheelRad, track, 90), true);
    rightMotor.rotate(convertAngle(wheelRad, track, 90), false);

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED + 2);

    leftMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 1.2), true);
    rightMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 1.2), false);

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
   * This method checks whether the robot has reached the current destination.
   * 
   * @return false if hasn't reached the current destination; true otherwise
   */
  public boolean checkIfDone(double destinationX, double destinationY) {
    double pos[] = odo.getXYT();
    if (Math.abs(destinationX * TILE_SIZE - pos[0]) < navigationAccuracy
        && Math.abs(destinationY * TILE_SIZE - pos[1]) < navigationAccuracy) {
      // y offset
      // leftMotor.rotate(convertDistance(wheelRad, 1.5), true);
      // rightMotor.rotate(convertDistance(wheelRad, 1.5), false);

      // leftMotor.synchronizeWith(new EV3LargeRegulatedMotor[] {rightMotor});
      // leftMotor.startSynchronization();
      // leftMotor.rotate(convertDistance(wheelRad, 1.5), true);
      // rightMotor.rotate(convertDistance(wheelRad, 1.5), false);
      // leftMotor.endSynchronization();
      
      offsetAdded = false;
      //reset offset

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
      System.out.println(usDistance);
      int filter_control = 10;
      // TODO: what if it is not a square?

      int boundary =
          (int) (Math.max(Math.round((SZ_LL_X + SZ_UR_X) / 2), Math.round((SZ_LL_Y + SZ_UR_Y) / 2))
              * TILE_SIZE);
      System.out.println("boundary" + boundary);
      boundary = (int) (2 * TILE_SIZE);
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
          System.out.println("size" + dataset.size());

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
        System.out.println("size" + dataset.size());

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



    // try {
    // Thread.sleep(2000);
    // } catch (InterruptedException e) {
    // e.printStackTrace();
    // }

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
  public void navigateToSearchingArea() {

    int c =-1;
    int localizeTheta = 0;
    if (TN_LL_Y + 2 == TN_UR_Y && TN_LL_X + 1 == TN_UR_X) {
      if (corner == 2 || corner == 3) {
        // case 1
        c = 1;
        destinations[0][0] = TN_LL_X;
        destinations[0][1] = TN_UR_Y + 1;   //first localization point
        destinations[1][0] = TN_LL_X + 0.5; // 0.45
        destinations[1][1] = TN_UR_Y + 1;
        destinations[2][0] = TN_LL_X + 0.5;
        destinations[2][1] = TN_LL_Y - 1;
        destinations[3][0] = TN_LL_X;
        destinations[3][1] = TN_LL_Y - 1;
      } else if (corner == 1 || corner == 0) {
        // case 4
        c = 4;
        destinations[0][0] = TN_LL_X + 1;
        destinations[0][1] = TN_LL_Y - 1;
        destinations[1][0] = TN_LL_X + 0.5; // 0.55
        destinations[1][1] = TN_LL_Y - 1;
        destinations[2][0] = TN_LL_X + 0.5;
        destinations[2][1] = TN_UR_Y + 1;
        destinations[3][0] = TN_LL_X;
        destinations[3][1] = TN_UR_Y + 1;
      }
    } else if (TN_LL_Y + 1 == TN_UR_Y && TN_LL_X + 2 == TN_UR_X) {

      if (corner == 1 || corner == 2) {
        // case 2
        c = 2;
        destinations[0][0] = TN_UR_X + 1;
        destinations[0][1] = TN_UR_Y;
        destinations[1][0] = TN_UR_X + 1;
        destinations[1][1] = TN_UR_Y - 0.5; // 0.45
        destinations[2][0] = TN_LL_X - 1;
        destinations[2][1] = TN_UR_Y - 0.5;
        destinations[3][0] = TN_LL_X - 1;
        destinations[3][1] = TN_UR_Y;
      } else if (corner == 0 || corner == 3) {
        // case 3
        c = 3;
        destinations[0][0] = TN_LL_X - 1;
        destinations[0][1] = TN_LL_Y;
        destinations[1][0] = TN_LL_X - 1;
        destinations[1][1] = TN_LL_Y + 0.5; // 0.45
        destinations[2][0] = TN_UR_X + 1;
        destinations[2][1] = TN_LL_Y + 0.5;
        destinations[3][0] = TN_UR_X + 1;
        destinations[3][1] = TN_UR_Y;
      }
    }

    destinations[4][0] = SZ_LL_X;
    destinations[4][1] = SZ_LL_Y;
    

    if(c == 3 || c ==2) {
      xFirst = false;
    }
    
    System.out.println("corner"+ corner +"travelling to"+destinations[0][0]+" "+ destinations[0][1]);

    // TODO: localize before passing the tunnel
     while (!checkIfDone(destinations[0][0], destinations[0][1])) {
     travelTo(destinations[0][0], destinations[0][1], xFirst);
     }
//     turnTo(0);
//     lightLocalizer.startLocalize((int)destinations[0][0]*TILE_SIZE,
//     (int)destinations[0][1]*TILE_SIZE, 0);
//    // System.out.println(odo.getXYT()[0]+" "+ odo.getXYT()[1]+" "+odo.getXYT()[2]);
//    System.out.println("doing localization");
     
     //do correction
//     double currentTheta = odo.getXYT()[2];
//     if(currentTheta)
     System.out.println("doing correction");
     turnTo(180);
     odometryCorrection.setSpeeds(75, 75);
     odometryCorrection.moveForward();
     odometryCorrection.correct(180);
     double forwardDistance = TILE_SIZE/2 - odometryCorrection.DISTANCE_TO_SENSOR;
     leftMotor.rotate(convertDistance(wheelRad, forwardDistance), true);
     rightMotor.rotate(convertDistance(wheelRad, forwardDistance), false);
     turnTo(270);
     odometryCorrection.setSpeeds(75, 75);
     odometryCorrection.moveForward();
     odometryCorrection.correct(270);
     leftMotor.rotate(convertDistance(wheelRad, 3*TILE_SIZE + (TILE_SIZE - odometryCorrection.DISTANCE_TO_SENSOR)), true);
     rightMotor.rotate(convertDistance(wheelRad, 3*TILE_SIZE + (TILE_SIZE - odometryCorrection.DISTANCE_TO_SENSOR)), false);
     
    for (int i = 3; i < 5; i++) {

      if(i != 3) {
        System.out.println("navigating to the tunnel");
        System.out.println("destinaiton " + destinations[i][0] + " " + destinations[i][1]);
        //skip the second localization point
        
        while (!checkIfDone(destinations[i][0], destinations[i][1])) {
          travelTo(destinations[i][0], destinations[i][1], xFirst);
        }
      
      //offset in y direction
      leftMotor.rotate(convertDistance(wheelRad, 2), true);
      rightMotor.rotate(convertDistance(wheelRad, 2), false);
      }
    }
    turnTo(0);
    leftMotor.stop(true);
    rightMotor.stop(false);
    Sound.beep();
    Sound.beep();
    Sound.beep();


    startCanGrabbing();
    // close the thing and push cans away
  }
  
  /**
   * This method drives the robot to leave the searching area.
   */
  public void leaveSearchingArea() {
    for (int i = 3; i > 0; i--) {
      //skip the first localization point 
      
      System.out.println("navigating back");
      System.out.println("destinaiton " + destinations[i][0] + " " + destinations[i][1]);
      while (!checkIfDone(destinations[i][0], destinations[i][1])) {
        travelTo(destinations[i][0], destinations[i][1], !xFirst);
      }
      leftMotor.rotate(convertDistance(wheelRad, 2), true);
      rightMotor.rotate(convertDistance(wheelRad, 2), false);
      
      if(i == 3) {
        System.out.println("doing localization");
        //localization at the second localization point
        
         turnTo(0);
         lightLocalizer.startLocalize((int)destinations[0][0]*TILE_SIZE,
         (int)destinations[0][1]*TILE_SIZE, 0);
        // System.out.println(odo.getXYT()[0]+" "+ odo.getXYT()[1]+" "+odo.getXYT()[2]);
      }
    }
    //reach the entry of the tunnel
    
    //travel to (0,0)
    while (!checkIfDone(0, 0)) {
      travelTo(0, 0, true);
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
  

}

