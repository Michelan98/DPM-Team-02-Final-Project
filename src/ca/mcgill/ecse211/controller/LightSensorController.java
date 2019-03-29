package ca.mcgill.ecse211.controller;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * 
 * This class allows to control the two light sensors to fetch brightness values.
 * The RedMode readings are used to determine those values. By multiplying the 
 * readings by 100, this allows the class to scale up the value of the readings.
 * The drops in brightness will indicate the presence of grid lines essential for 
 * Odometry Correction and LightLocalization.
 * 
 * @author Michel Abdel Nour
 * @author Sandra Deng
 */
public class LightSensorController {
	
	private EV3ColorSensor lightSensor;
	private SampleProvider lightSamples;
	private float[] lsData;
	private float oldReading = -1;
	private float newReading;
	
	private float threshold = 20;
		
	/**
	 * This method is a constructor for this class
	 * @param lightSensor light sensor to use
	 * @param lcd lcd screen on the ev3 block
	 */
	public LightSensorController(EV3ColorSensor lightSensor) {
		this.lightSensor = lightSensor;
		lightSamples = lightSensor.getRedMode();
		lsData = new float[lightSensor.sampleSize()];
	}
	
	/**
	 * This method returns whether a boolean, which is:
	 * 
	 * 	 	true - to indicate a line being detected
	 * 		false - to indicate that no line has been detected
	 * 
	 * @return 
	 */
	public boolean lineDetected() {
		lightSamples.fetchSample(lsData, 0);
		newReading = lsData[0] * 100;
		if(oldReading == -1) {
			oldReading = newReading;
		}
		else if ((100 * Math.abs((newReading - oldReading)/oldReading)) > threshold){
			if(newReading < oldReading) {
				return true;
			}
		}
		return false;
	}	
}