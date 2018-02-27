package ca.mcgill.ecse211.Localization;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;

public class LightPoller {
	private static EV3ColorSensor colorSensorBlock;
	private static float[] sample = new float[3];
	
	public LightPoller(EV3ColorSensor colorSensorBlock){
		this.colorSensorBlock = colorSensorBlock;
		colorSensorBlock.getRGBMode();
	}
	
	public static void colorCheck(){
		colorSensorBlock.getRGBMode().fetchSample(sample, 0); //get sample data from light sensor
		System.out.println(sample[0] + ", " + sample[1] + ", " + sample[2]);
		if(sample[0] <= 0.08 && sample[1] <= 0.08 && sample[2] <= 0.08){
			//black or blue
		}else if(sample[0] > 0.1 && sample[1] <= 0.08 && sample[2] <= 0.08){
			//red
		}else if(sample[0] >= 0.1 && sample[2] >= 0.1 && sample[2] <= 1){
			//white
		}
	}
}
