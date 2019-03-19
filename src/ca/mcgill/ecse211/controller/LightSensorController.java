package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * this class is used to acquire values from the light sensor that
 * are necessary to detect grid lines. The class uses a Thread in 
 * order to continously make measurements.
 * 
 * @author michelabdelnour
 *
 */
public class LightSensorController {
	
	private EV3ColorSensor lightSensor;
	private SensorMode myColor;
	private SampleProvider colorSample;
	private float[] sampleColor ;
	private int lastData;
	private int newData;
	public static final double THRESHOLD = -50;
	private static int base;
	private static boolean firstLine = true;
		
	/**
	 * This is a constructor for this class
	 * @param lightSensor light sensor to use
	 * @param lcd lcd screen on the ev3 block
	 */
	public LightSensorController(EV3ColorSensor lightSensor) {
	  
	     this.lightSensor = lightSensor;
	     colorSample = lightSensor.getMode("Red");
//	     myColor = this.lightSensor.getRedMode();
	  
//		myColor = new EV3ColorSensor(colorPort);
	     sampleColor = new float[colorSample.sampleSize()];
	     
	}
	
	public boolean lineDetected() {
		if(true) {
			lastData = fetch();
		}
		newData = fetch();

      System.out.println("difference"+(newData-lastData));
		if((newData-lastData) < THRESHOLD) {
			try {
				Thread.sleep(15);
			}catch(InterruptedException e) {
				e.printStackTrace();
			}
			
			if ((fetch() - lastData) < THRESHOLD) {
				try {
					Thread.sleep(15);
				}catch(InterruptedException e) {
					e.printStackTrace();
				}
				if((fetch() - lastData) < THRESHOLD) {
					lastData = newData;
					return true;
				}
		
			}else {
				lastData = newData;
				return false;
			}
		}
		lastData = newData;
		return false;
	}
	
	/**
	 * This method fetches samples from the light sensor 
	 * @return color intensity (float)
	 */
	public int fetch() {
		int sensorData;
		colorSample.fetchSample(sampleColor, 0);
		sensorData = (int) (sampleColor[0] * 1000);
		System.out.println(sensorData);
		return sensorData;
	}	
}
