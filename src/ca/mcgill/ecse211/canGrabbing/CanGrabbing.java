package ca.mcgill.ecse211.canGrabbing;

import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class CanGrabbing {

  private static EV3MediumRegulatedMotor canGrabbingMotor = null;
  
  public CanGrabbing(EV3MediumRegulatedMotor canGrabbingMotor) {
    this.canGrabbingMotor = canGrabbingMotor;
  }
  
  /**
   * control the medium motor to grab the can
   */
  public void grabCan() {
    canGrabbingMotor.rotate(105);
  }
  
  /**
   * control the medium motor to release the can
   */
  public void releaseCan() {
    canGrabbingMotor.rotate(-105);
  }
}
