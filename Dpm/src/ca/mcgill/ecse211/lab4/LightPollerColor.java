package ca.mcgill.ecse211.lab4;

import java.util.ArrayList;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightPollerColor {

	private static final float COLOR_THRESHOLD = 0.2f;
	private SampleProvider lt;
	private float[] ltdata;
	private int lightVal;
	private float[] desiredColor;
	

//	public LightPollerColor(SampleProvider lt, float[] ltdata) {
//		this.lt = lt;
//		this.ltdata = ltdata;
//	}

	public LightPollerColor(SampleProvider lt, float[] ltdata) {
		this.lt = lt;
		this.ltdata = ltdata;
		// TODO Auto-generated constructor stub
	}

	public void calibrate() {
		lt.fetchSample(ltdata, 0);
		ArrayList<float[]> RGBValues = new ArrayList<float[]>();
		for (int i = 0; i < 50; i++) {
			lt.fetchSample(ltdata, 0);
			System.out.println(ltdata[0] + "\t" + ltdata[1] + "\t" +ltdata[2]);
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		Sound.beep();
	}
	
	private float[] getAverageMeasurement() {
		float[] avg_values = {0,0,0};
		float red_sum = 0, green_sum = 0, blue_sum = 0;
		
		for (int i = 0; i < 10; i ++) {
			lt.fetchSample(ltdata, 0);
			red_sum += ltdata[0];
			green_sum += ltdata[1];
			blue_sum += ltdata[2];
		}
		avg_values[0] = red_sum/10;
		avg_values[1] = green_sum/10;
		avg_values[2] = blue_sum/10;
		return avg_values;
	}
	
	private float normalize(float mean, float stdev, float x) {
		float z = (x-mean)/stdev;
		return z ;
	}
	
	public boolean detectColor(float[] setColor) {
		float[] reading = getAverageMeasurement();
		
		if (withinStdDev(ltdata, setColor)) {
			return true;
		}
		return false;
	}

	private boolean withinStdDev(float[] setColor, float[] measuredColor) {
		if (Math.abs(setColor[0] - measuredColor[0]) < COLOR_THRESHOLD
			&& Math.abs(setColor[1] - measuredColor[1]) < COLOR_THRESHOLD
			&& Math.abs(setColor[2] - measuredColor[2]) < COLOR_THRESHOLD) {
			return true;
		}
		return false;
	}
}
