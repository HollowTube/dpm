/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.Odometer;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private static final double SQUARE_LENGTH = 30.48;
	private static final double ANGLE_THRESHOLD = 20;
	private static final double LIGHTSENS_OFFSET = 3.5;
	private static final double LIGHTSENS_THRESHOLD = 10;
	private Odometer odometer;
	private SampleProvider lt;
	private float[] ltdata;
	private int lightVal;

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection(SampleProvider lt, float[] ltdata) throws OdometerExceptions {
		this.lt = lt;
		this.ltdata = ltdata;
		this.odometer = Odometer.getOdometer();

	}

	public boolean correctionTrigger() {
		lt.fetchSample(ltdata, 0);
		lightVal = (int) (ltdata[0] * 100);
		if (lightVal < LIGHTSENS_THRESHOLD) { // triggers when a low amount of light is reflected (i.e. it detects
												// black)
			// Sound.beep();
			return true;
		}
		return false;
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		double position[];
		double head = 0;
		int currentYQuad, currentXQuad;
		double newy, newx = 0;
		while (true) {
			correctionStart = System.currentTimeMillis();
			// TODO variable lightsensor offset

			if (correctionTrigger()) {
				position = odometer.getXYT(); // get current position and heading from odometer
				head = position[2]; // get current heading

				if (head > 350 || head < 10) {// going up
					currentYQuad = (int) ((position[1] + 10) / SQUARE_LENGTH); // calculates its vertical tile location(
																				// i.e 1 or 2 tiles up from the defined
																				// origin),
					// the + 10 is for the cases where the robot is lagging behind the actual value,
					// facilitates the floor division.
					newy = SQUARE_LENGTH * currentYQuad - LIGHTSENS_OFFSET; // plugs in the correct y value
					odometer.setY(newy); // sets the correct y value in the display
					Sound.buzz(); // audio feedback for easier troubleshooting
				} else if (Math.abs(head - 180) < ANGLE_THRESHOLD) {// going down , same as above but in other direction
					currentYQuad = (int) ((position[1] + 10) / SQUARE_LENGTH);
					newy = SQUARE_LENGTH * currentYQuad + LIGHTSENS_OFFSET;
					odometer.setY(newy);
					Sound.buzz();
				}
				// going right
				else if ((Math.abs(head - 90) < ANGLE_THRESHOLD)) { // calculates its horizontal tile location (i.e. 1
																	// or 2 tiles left or right from the defined origin
					currentXQuad = (int) ((position[0] + 10) / SQUARE_LENGTH);
					newx = SQUARE_LENGTH * currentXQuad - LIGHTSENS_OFFSET;// calculates its actual x position, includes
																			// the offset from the light sensor
					odometer.setX(newx);
					Sound.beep();
				} // going left
				else if ((Math.abs(head - 270) < ANGLE_THRESHOLD)) {
					currentXQuad = (int) ((position[0] + 10) / SQUARE_LENGTH);
					newx = SQUARE_LENGTH * currentXQuad + LIGHTSENS_OFFSET;
					odometer.setX(newx);
					Sound.beep();
				}
			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
