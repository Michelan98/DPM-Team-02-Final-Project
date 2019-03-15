package ca.mcgill.ecse211.odometer;

import lejos.hardware.lcd.TextLCD;
import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.*;

public class OdometryDisplay implements Runnable {

	  private Odometer odometer;
	  private TextLCD lcd;
	  private double[] position;
	  // constant for display time
	  private final long DISPLAY_PERIOD = 25;
	  // interrupt constant
	  private long timeout = Long.MAX_VALUE;

	  /**
	   * This is the class constructor
	   * 
	   * @param odoData data of odometer
	   * @throws OdometerExceptions 
	   */
	  public OdometryDisplay(TextLCD lcd) throws OdometerExceptions {
	    odometer = Odometer.getOdometer();
	    this.lcd = lcd;
	  }

	  /**
	   * This is the overloaded class constructor
	   * 
	   * @param odoData data of odometer
	   * @throws OdometerExceptions 
	   */
	  public OdometryDisplay(TextLCD lcd, long timeout) throws OdometerExceptions {
	    odometer = Odometer.getOdometer();
	    this.timeout = timeout;
	    this.lcd = lcd;
	  }

	  /**
	   * Thread run() to periodically display the ododata on the lcd screen
	   */
	  public void run() {
	    
	    lcd.clear();
	    
	    long updateStart, updateEnd;

	    long tStart = System.currentTimeMillis();
	    do {
	      updateStart = System.currentTimeMillis();

	      // Retrieve x, y and Theta information
	      position = odometer.getXYT();
	      
	      // Print x,y, and theta information
	      DecimalFormat numberFormat = new DecimalFormat("######0.00");
	      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
	      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
	      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
	      // this ensures that the data is updated only once every period
	      updateEnd = System.currentTimeMillis();
	      if (updateEnd - updateStart < DISPLAY_PERIOD) {
	        try {
	          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
	        } catch (InterruptedException e) {
	          e.printStackTrace();
	        }
	      }
	    } while ((updateEnd - tStart) <= timeout);

	  }

	}