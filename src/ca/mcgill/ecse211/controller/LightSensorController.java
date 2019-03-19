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
	private SensorModes myColor;
	private SampleProvider colorSample;
	private float[] sampleColor ;
	private int lastData;
	private int newData;
	private static final double THRESHOLD = 50;
	private static int base;
	private static boolean firstLine = true;
		
	/**
	 * This is a constructor for this class
	 * @param lightSensor light sensor to use
	 * @param lcd lcd screen on the ev3 block
	 */
	public LightSensorController(Port colorPort) {
		myColor = new EV3ColorSensor(colorPort);
		sampleColor = new float[myColor.sampleSize()];
		colorSample = myColor.getMode("Red");
		this.lightSensor = lightSensor;
	}
	
	public boolean lineDetected(boolean firstLine) {
		if(firstLine) {
			lastData = fetch();
		}
		newData = fetch();
		
		if(Math.abs(newData-lastData)  < THRESHOLD) {
			try {
				Thread.sleep(15);
			}catch(InterruptedException e) {
				e.printStackTrace();
			}
			
			if (Math.abs(fetch() - lastData) < THRESHOLD) {
				try {
					Thread.sleep(15);
				}catch(InterruptedException e) {
					e.printStackTrace();
				}
				if(Math.abs(fetch() - lastData) < THRESHOLD) {
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
		
		return sensorData;
	}	
}
