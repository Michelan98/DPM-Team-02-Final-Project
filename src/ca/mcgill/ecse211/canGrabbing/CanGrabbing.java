package ca.mcgill.ecse211.canGrabbing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * This class is responsible for can grabbing.
 * 
 * @author Sandra Deng
 *
 */

public class CanGrabbing {

  private static EV3MediumRegulatedMotor canGrabbingMotor = null;
  private static EV3LargeRegulatedMotor sensorMotor = null;

  /**
   * constructor
   * @param canGrabbingMotor
   * @param sensorMotor
   */
  public CanGrabbing(EV3MediumRegulatedMotor canGrabbingMotor, EV3LargeRegulatedMotor sensorMotor) {
    this.canGrabbingMotor = canGrabbingMotor;
    this.sensorMotor = sensorMotor;
  }

  /**
   * control the medium motor and the sensor motor to grab the can
   */
  public void grabCan() {
    canGrabbingMotor.rotate(90, true);
    sensorMotor.rotate(-30, false);
  }

  /**
   * control the medium motor and the sensor motor to release the can
   */
  public void releaseCan() {
    canGrabbingMotor.rotate(-90, true);
    sensorMotor.rotate(30, false);
  }
}
