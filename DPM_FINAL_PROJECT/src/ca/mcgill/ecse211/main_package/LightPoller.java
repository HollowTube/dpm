package ca.mcgill.ecse211.main_package;

import lejos.robotics.SampleProvider;


/**
 * This class handles the data collected by the light sensor used for localization and
 * odometry correction. 
 * 
 * @author tritin
 *
 */
public class LightPoller {

	private SampleProvider lt;
	private float[] ltdata;
	private int lightVal;

	private int prev_light;
	private int current_light;

	private boolean first_time = true;

	/**
	 * Class constructor
	 * 
	 * @param lt
	 * @param ltdata
	 */
	public LightPoller(SampleProvider lt, float[] ltdata) {
		this.lt = lt;
		this.ltdata = ltdata;

	}

	public int getValue() {
		lt.fetchSample(ltdata, 0);
		lightVal = (int) (ltdata[0] * 100);
		return lightVal;

	}
	/**
	 * Boolean method to determine if a light value is lower than the threshold set.
	 * 
	 * @param threshold
	 * @return
	 */
	public boolean lessThan(int threshold) {
		current_light = getValue();

		if (current_light < threshold) {
			return true;
		} else
			return false;
	}

	/**
	 * Boolean method to determine if the data value is reducing. (between tile and line)
	 * 
	 * @param threshold
	 * @return
	 */
	public boolean falling(int threshold) {
		boolean edge;
		current_light = getValue();

		
		if (first_time) {
			prev_light = current_light;
			first_time = false;
		}

		if (current_light < threshold && prev_light > threshold)
			edge = true;
		else {
			edge = false;
		}
			
		prev_light = current_light;
		return edge;
	}

	/**
	 * Boolean method to determine if the data value is rising. (between end of line and next tile) 
	 * 
	 * @param threshold
	 * @return
	 */
	public boolean rising(int threshold) {
		boolean edge;
		current_light = getValue();


		if (first_time) {
			prev_light = current_light;
			first_time = false;
		}

		if (current_light > threshold && prev_light < threshold)
			edge = true;
		else {
			edge = false;
		}
			
		prev_light = current_light;
		return edge;
	}
}
