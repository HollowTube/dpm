package ca.mcgill.ecse211.main_package;

import lejos.robotics.SampleProvider;


/**
 * This class handles the data collected by the light sensor used for localization and
 * odometry correction. It is meant to be used when the robot has to detect black lines on a grid.
 * It has methods detecting rising and falling changes in the data from the light sensor which
 * in turn, mean that a line has been found.
 * 
 * @author Tritin
 *
 */
public class LightPoller {

	private SampleProvider lt;
	private float[] ltdata;
	private int lightVal;
	private final int  THRESHOLD = 5;

	private int prev_light;
	private int current_light;

	private boolean first_time = true;

	/**
	 * This is the class constructor which is comprised of the sample from the light sensor and its value.
	 * 
	 * @param lt Sample from the light sensor
	 * @param ltdata Value of sample
	 */
	public LightPoller(SampleProvider lt, float[] ltdata) {
		this.lt = lt;
		this.ltdata = ltdata;

	}

	public void getValue() {
		lt.fetchSample(ltdata, 0);
		lightVal = (int) (ltdata[0] * 100);
		current_light = lightVal;

	}
	/**
	 * Boolean method to determine if a light value is lower than the threshold set.
	 * The threshold is a value where a black object would return a sample with lower value all the time.
	 * 
	 * @param threshold Threshold value for black line
	 * @return True is lower, False otherwise
	 */
	public boolean lessThan(int threshold) {
		getValue();
		if (current_light < threshold) {
			return true;
		} else
			return false;
	}

	/**
	 * Boolean method to determine if the data value is reducing. (between tile and line)
	 * This method takes the first sample as the previous one and takes the second sample as the current one.
	 * Then, it takes the difference between the previous and current sample and finds
	 * if there is a falling edge there.
	 * The Current value then becomes the previous one for next time.
	 * 
	 * @param threshold Delta value accepted
	 * @return True if falling edge is found, false otherwise
	 */
	public synchronized boolean falling(int threshold) {
		boolean edge;
		double change_in_light;
		getValue();

		if (first_time) {
			prev_light = current_light;
			first_time = false;
		}
		
		change_in_light = current_light-prev_light;
		if (change_in_light < -THRESHOLD) {
			edge = true;
		}
		else {
			edge = false;
		}
			
		prev_light = current_light;
		return edge;
	}

	/**
	 * Boolean method to determine if the data value is rising. (between end of line and next tile)
	 * This method takes the first sample as the previous one and takes the second sample as the current one.
	 * Then, it takes the difference between the previous and current sample and finds
	 * if there is a rising edge there.
	 * The Current value then becomes the previous one for next time.  
	 * 
	 * @param threshold Delta value accepted
	 * @return True if rising edge is found, false otherwise
	 */
	public boolean rising(int threshold) {
		boolean edge;
		double change_in_light;
		getValue();

		if (first_time) {
			prev_light = current_light;
			first_time = false;
		}
		change_in_light = current_light-prev_light;
		if (change_in_light > threshold) {
			edge = true;
		}
		else {
			edge = false;
		}
			
		prev_light = current_light;
		return edge;
	}
	
}
