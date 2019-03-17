package ca.mcgill.ecse211.navigation;

import java.util.ArrayList;
import ca.mcgill.ecse211.colorClassification.ColorClassification;
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
    INIT, TRAVELLING, SCANNING, COLORDETECTION, AVOIDINGCAN, GRABBINGCAN, LEAVING
  };

  private static State state;
  private final double navigationAccuracy = 3;
  private final double thetaAccuracy = 0.7;
  private boolean xIsDone = false;

  private volatile static ArrayList<Double> canLocation; // the angle of can to the robot
  private static ArrayList<Double> dataset;

  public NavigationWithObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double track, double wheelRad, int LLX, int LLY, int URX, int URY,
      EV3LargeRegulatedMotor sensorMotor, TextLCD lcd, int TR, SampleProvider sampleProvider)
      throws OdometerExceptions {

    odo = Odometer.getOdometer();
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

  }

  /**
   * This method is inherited from the runnable interface. It is the entry point of the thread.
   * State transition happens in this method.
   */
  public void run() {

    try {
      timer = new Timer(100, new NavigationWithObstacle(leftMotor, rightMotor, track, wheelRad, LLX, LLY, URX,
          URY, sensorMotor, lcd, TR, sampleProvider));
    } catch (OdometerExceptions e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    timer.start();

    currentDestination[0] = 3;
    currentDestination[1] = 2;
    // travelTo(currentDestination[0], currentDestination[1]); // half tile away from the searching


    state = State.INIT;


    while (true) {
      lcd.drawString(state + "", 1, 1);
      System.out.println(state + " us distance " + usDistance);

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
          } else if (!checkIfDone(currentDestination[0], currentDestination[1])) {
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
        case SCANNING: {
          scan();
          state = State.COLORDETECTION;
        }
        case COLORDETECTION: {

          for (Double data : canLocation) {
            turnTo(data);
            startColorDetection();
            if (isTargetCan) {
              state = State.GRABBINGCAN;
              break;
            }
          }

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

          break;
        }
      }
//      if (checkIfDone(currentDestination[0], currentDestination[1]))
//        break;
//      try {
//        Thread.sleep(50);
//      } catch (InterruptedException e) {
//        // TODO Auto-generated catch block
//        e.printStackTrace();
//      }
    }
//    Sound.beep();
//    timer.stop();
  }



  // // from (0,0) to (LLX, LLY)
  // travelTo(currentDestination[0], currentDestination[1]);
  // Sound.beep();
  // travelTo(currentDestination[0], --currentDestination[1]);
  //
  // while (count <= (URX - LLX) + 1) {
  // if (found) {
  // travelTo(currentDestination[0], currentDestination[1]);
  // break;
  // }
  // // current (x,y)
  // double pos[] = odo.getXYT();
  // double currentX = pos[0];
  // double currentY = pos[1];
  // double theta = pos[2];
  //
  // double xCoordinate = Math.round(((float) currentX / (float) TILE_SIZE));
  // double yCoordinate = Math.round(((float) currentY / (float) TILE_SIZE));
  //
  // System.out.println("xCoordinate " + xCoordinate);
  // System.out.println("yCoordinate " + yCoordinate);
  // System.out.println("count " + count);
  //
  // // special case
  // if ((yCoordinate == URY && aCanOnBoundary) || (yCoordinate == LLY && aCanOnBoundary)) {
  // count++;
  // }
  //
  // int isAtURY = -1;
  //
  // if (yCoordinate <= LLY - 1) {
  // // turn to 0 and check for can
  //
  // // go to the next x
  // if (count != 0) {
  // currentDestination[0]++;
  // travelTo(currentDestination[0], currentDestination[1]);
  // }
  //
  // turnTo(0);
  //
  // sampleProvider.fetchSample(sample, 0);
  // System.out.println("theta in LLY" + theta);
  // System.out.println("sample " + sample[0]);
  //
  // Sound.beep();
  //
  // count++;
  // isAtURY = 0;
  // } else if (yCoordinate >= URY + 1) {
  // // turn to 180 to check for can
  //
  // // go to the next x
  // if (count != 0) {
  // currentDestination[0]++;
  // travelTo(currentDestination[0], currentDestination[1]);
  // }
  //
  // turnTo(180);
  //
  // // System.out.println("theta in URY" + theta);
  //
  // sampleProvider.fetchSample(sample, 0);
  // Sound.beep();
  // System.out.println("sample" + sample[0] * 100);
  // System.out.println("sample " + sample[0]);
  // count++;
  // isAtURY = 1;
  // }
  //
  //
  // if (sample[0] * 100 < (URY - LLY) * TILE_SIZE + 20 && isAtURY != -1) {
  // // found a can
  // // go one tile further, in case there are some cans on LLY or URY
  // if (isAtURY == 0) {
  // currentDestination[1] = URY + 1;
  // } else if (isAtURY == 1) {
  // currentDestination[1] = LLY - 1;
  // }
  //
  // System.out.println("if " + currentDestination[0] + "," + currentDestination[1]);
  // travelTo(currentDestination[0], currentDestination[1]);
  //
  // } else if (isAtURY == -1) {
  // count++;
  // System.out.println("else " + currentDestination[0] + "," + currentDestination[1]);
  // travelTo(currentDestination[0], currentDestination[1]);
  // }
  // }

  // got to (URX, URY)
  // currentDestination[0] = URX;
  // currentDestination[1] = URY;
  // travelTo(currentDestination[0], currentDestination[1]);


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


    // calculate the distance needed to be traveled
    // double distance = Math
    // .sqrt(Math.pow((currentX - x * TILE_SIZE), 2) + Math.pow((currentY - y * TILE_SIZE), 2));
    //
    //
    // // turn to the destination angle
    // System.out.println(
    // "turning angle" + angleCalculation(currentX, currentY, x * TILE_SIZE, y * TILE_SIZE));
    // System.out.println("travel to " + x + " " + y);
    // turnTo(angleCalculation(currentX, currentY, x * TILE_SIZE, y * TILE_SIZE));

    // run along the line
    double distanceX = x * TILE_SIZE - currentX;
    System.out.println("DISTANCEX" + distanceX);
    double distanceY = y * TILE_SIZE - currentY;
    System.out.println("DISTANCEY" + distanceY);

    if (Math.abs(distanceX) >navigationAccuracy ) {
      if (distanceX < 0 && Math.abs(theta - 270)>thetaAccuracy) {
        turnTo(270);
      }
      else if(distanceX >0 && Math.abs(theta - 90)>thetaAccuracy) {
        turnTo(90);
      }

      leftMotor.forward();
      rightMotor.forward();

    } else {
      //x is done
      xIsDone = true;
      if (distanceY < 0 && Math.abs(theta - 180)>thetaAccuracy) {
        turnTo(180);
      }
      else if(distanceY > 0 && Math.abs(theta - 0)>thetaAccuracy){
        turnTo(0);
      }
      
      leftMotor.forward();
      rightMotor.forward();

    }

    // while (isNavigating()) {
    // if (flag == 1) {
    // // detect a block
    // timer.stop();
    // if (!found) {
    // startColorDetection();
    // }
    // // finish the colorDetection, start the can avoidance
    // canAvoidance();
    // // avoided the can continue on the navigation
    // count--;
    // break;
    // }
    // }


  }

  /**
   * This method causes the robot to turn to the absolute heading theta the method turn a minimal
   * angel to its target.
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
   * This method calculate the turning angle.
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

//    leftMotor.setSpeed(TURNING_SPEED);
//    rightMotor.setSpeed(TURNING_SPEED);
//
//    // turn left 90 degree
//    leftMotor.rotate(-convertAngle(wheelRad, track, 90), true);
//    rightMotor.rotate(convertAngle(wheelRad, track, 90), false);
//
//    leftMotor.setSpeed(FORWARD_SPEED);
//    rightMotor.setSpeed(FORWARD_SPEED + 2);
//
//    leftMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 1.5), true);
//    rightMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 1.5), false);
//
//    leftMotor.setSpeed(TURNING_SPEED);
//    rightMotor.setSpeed(TURNING_SPEED);
//
//    // turn left 90 degree
//    leftMotor.rotate(-convertAngle(wheelRad, track, 90), true);
//    rightMotor.rotate(convertAngle(wheelRad, track, 90), false);
//
//    leftMotor.setSpeed(FORWARD_SPEED);
//    rightMotor.setSpeed(FORWARD_SPEED + 2);
//
//    leftMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 0.7), true);
//    rightMotor.rotate(convertDistance(wheelRad, TILE_SIZE * 0.7), false);
//
//    leftMotor.setSpeed(TURNING_SPEED);
//    rightMotor.setSpeed(TURNING_SPEED);
//
//    // turn right 90 degree
//    leftMotor.rotate(convertAngle(wheelRad, track, 90), true);
//    rightMotor.rotate(-convertAngle(wheelRad, track, 90), false);
//
//    // pass through the can continue the navigation
//    double yCoordinate = Math.round(((float) odo.getXYT()[1] / (float) TILE_SIZE));
//    if (yCoordinate == URY || yCoordinate == LLY)
//      aCanOnBoundary = true;
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
    System.out.println("x " + pos[0] + "y " + pos[1]);
    if (Math.abs(destinationX * TILE_SIZE - pos[0]) < navigationAccuracy
        && Math.abs(destinationY * TILE_SIZE - pos[1]) < navigationAccuracy) {
      System.out.println("is done");
      return true;
    } else {
      System.out.println("not done");
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
      System.out.println(usDistance + " " + odo.getXYT()[2] + " " + filter_count);
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
            System.out.println("usDistance: " + usDistance + " theta: " + odo.getXYT()[2]);
            // canLocation.add(theta);
          }

        } else {
          System.out.println(dataset.size() + " " + filter_count);
          filter_count = 0;

          if (dataset.size() >= filter_control) {
            // int mid = dataset.size()/2;
            double midTheta = (dataset.get(0) + dataset.get(dataset.size() - 1)) / 2;
            System.out.println("mid " + midTheta);
            canLocation.add(midTheta);
            System.out.println("added in if");
          }
          dataset = new ArrayList<Double>();
        }
      } else {
        // outside of boundary
        if (dataset == null) {
          System.out.println("null");
        }
        System.out.println(dataset.size() + " " + filter_count);
        filter_count = 0;

        if (dataset.size() >= filter_control) {
          double midTheta = (dataset.get(0) + dataset.get(dataset.size() - 1)) / 2;
          System.out.println("mid " + midTheta);
          canLocation.add(midTheta);
          System.out.println("added in else");
        }
        dataset = new ArrayList<Double>();
      }
    }
    previousUsDistance = usDistance;
  }



  public void scan() {
//    try {
//      timer = new Timer(20, new Navigation(leftMotor, rightMotor, track, wheelRad, LLX, LLY, URX,
//          URY, sensorMotor, lcd, TR, sampleProvider));
//    } catch (OdometerExceptions e) {
//      // TODO Auto-generated catch block
//      e.printStackTrace();
//    }
//    timer.start();
    leftMotor.setSpeed(TURNING_SPEED);
    rightMotor.setSpeed(TURNING_SPEED);
    leftMotor.rotate(convertAngle(wheelRad, track, 360), true);
    rightMotor.rotate(-convertAngle(wheelRad, track, 360), false);

    timer.stop();

    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    System.out.println("data size" + canLocation.size());
    for (Double data : canLocation) {
      lcd.drawString(Double.toString(data), 0, 0);
      System.out.println(Double.toString(data));
      turnTo(data - 3);
      Sound.beep();
      sampleProvider.fetchSample(sample, 0);
      usDistance = (int) (sample[0] * 100); // convert to cm
      System.out.println(usDistance);
      startColorDetection();
    }



  }

}

