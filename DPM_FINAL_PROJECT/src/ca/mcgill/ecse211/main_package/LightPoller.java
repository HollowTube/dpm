package ca.mcgill.ecse211.main_package;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class handles the data collected by the light sensor used for
 * localization and odometry correction. It is meant to be used when the robot
 * has to detect black lines on a grid. It has methods detecting rising and
 * falling changes in the data from the light sensor which in turn, mean that a
 * line has been found.
 * 
 * @author Tritin
 *
 */
public class LightPoller {

	private SampleProvider lt;
	private float[] ltdata;
	private float lightVal;
	private final float THRESHOLD = 2.0f;
	private float prev_light;
	private float current_light;
	private float baseline;

	private boolean first_time = true;

	/**
	 * This is the class constructor which is comprised of the sample from the light
	 * sensor and its value.
	 * 
	 * @param lt
	 *            Sample from the light sensor
	 * @param ltdata
	 *            Value of sample
	 */
	public LightPoller(SampleProvider lt, float[] ltdata) {
		this.lt = lt;
		this.ltdata = ltdata;
		this.baseline = getBaseline();

	}

	/**
	 * made to run in the constructor, returns the baseline value of the game board(i.e. not the black line)
	 * @return
	 */
	private float getBaseline() {
		for (int i = 0; i < 5; i++) {
			lt.fetchSample(ltdata, 0);
			lightVal += (ltdata[0] * 100);
		}
		return lightVal / 5;
	}

	
	/**
	 * updates the value of the current light and the prev light
	 */
	public void getValue() {
//		lightVal = 0;
		lt.fetchSample(ltdata, 0);
		lightVal = (ltdata[0]*100);
//		for (int i = 0; i < 3; i++) {
//			lt.fetchSample(ltdata, 0);
//			lightVal += (ltdata[0] * 100);
//		}

		if (first_time) {
			prev_light = lightVal;
			first_time = false;
		} else {
			prev_light = current_light;
		}
		current_light = lightVal ;
		System.out.println(current_light);
	}

	/**
	 * Boolean method to determine if a light value is lower than the threshold set.
	 * The threshold is a value where a black object would return a sample with
	 * lower value all the time.
	 * 
	 * @param threshold
	 *            Threshold value for black line
	 * @return True is lower, False otherwise
	 */
	public boolean lessThan(float threshold) {
//		getValue();
		lt.fetchSample(ltdata, 0);
		current_light = (ltdata[0] * 100);
//		System.out.println(current_light);
		if (current_light < threshold) {
//			Sound.beep();
			return true;
		} else
			return false;
	}

	/**
	 * Boolean method to determine if the data value is reducing. (between tile and
	 * line) This method takes the first sample as the previous one and takes the
	 * second sample as the current one. Then, it takes the difference between the
	 * previous and current sample and finds if there is a falling edge there. The
	 * Current value then becomes the previous one for next time.
	 * 
	 * @param threshold
	 *            Delta value accepted
	 * @return True if falling edge is found, false otherwise
	 */
	public boolean falling() {
		boolean edge;
		float change_in_light;

		change_in_light = current_light - prev_light;
		if (change_in_light < -THRESHOLD) {
			edge = true;
		} else {
			edge = false;
		}
		return edge;
	}

	/**
	 * Boolean method to check if the light sensor value is under the baseline
	 * measured at the calibration stage
	 * 
	 * @return
	 */
	public boolean underBaseline() {

		for (int i = 0; i < 3; i++) {
			lt.fetchSample(ltdata, 0);
			lightVal += (ltdata[0] * 100);
		}
		if (baseline - lightVal/3 > 3) {
			return true;
		} else
			return false;

	}

	/**
	 * Boolean method to determine if the data value is rising. (between end of line
	 * and next tile) This method takes the first sample as the previous one and
	 * takes the second sample as the current one. Then, it takes the difference
	 * between the previous and current sample and finds if there is a rising edge
	 * there. The Current value then becomes the previous one for next time.
	 * 
	 * @param threshold
	 *            Delta value accepted
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
		change_in_light = current_light - prev_light;
		if (change_in_light > threshold) {
			edge = true;
		} else {
			edge = false;
		}

		prev_light = current_light;
		return edge;
	}

}
