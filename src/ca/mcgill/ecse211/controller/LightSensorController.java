package ca.mcgill.ecse211.controller;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class LightSensorController {

	private EV3ColorSensor lightSensor;
	private SensorMode idColour;
	private float[] colorValue;
	
	public static final double THRESHOLD = 0.30;
	
	private float colorIntensity;
	
	/**
	 * This method is a constructor for this class
	 * @param lightSensor light sensor to use
	 * @param lcd lcd screen on the ev3 block
	 */
	public LightSensorController(EV3ColorSensor lightSensor) {
		this.lightSensor = lightSensor;
		idColour = this.lightSensor.getRedMode();
		colorValue = new float[idColour.sampleSize()];
	}
	
	/**
	 * This method fetches samples from the light sensor 
	 * @return color intensity (float)
	 */
	public float fetch() {
		idColour.fetchSample(colorValue, 0);
		colorIntensity = colorValue[0];
		return colorIntensity;
	}	
}