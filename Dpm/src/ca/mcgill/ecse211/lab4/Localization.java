package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import java.util.*;

/**
 * This class is used to perform localization using both the UltraSonic and light sensors.
 * It allows the robot to now its position and orientation and therefore correct itself accordingly.
 */
public class Localization {
	static boolean measuring = true;

/**
* Variables needed
*/	
	private final static int EDGE_TRIGGER = 150;;
	private final static int MARGIN = 10;
	private final static double LIGHT_OFFSET = 7.73;
	private static Odometer odometer;
	private static MotorControl motorcontrol;

	double current_dist, prev_dist, change_inDist = 0;
	double theta, itheta = 0;
	boolean in_margin = false;
/**
* This is the class constructor
* 
* @throws OdometerExceptions 
*/
	public Localization() throws OdometerExceptions {
		Localization.odometer = Odometer.getOdometer();
		Localization.motorcontrol = Lab4.motorControl;
	}

/**
 * Runs the class
 * @throws OdometerExceptions
 */
	public void go() throws OdometerExceptions {

		localize_heading();

	}
/**
 * Method to know if a rising edge is detected
 * @return
 */
	public boolean rising_edge() {
		boolean edge_detected, in_margin = false;

		current_dist = Lab4.usPoller.getDist();

		if (current_dist > EDGE_TRIGGER + MARGIN && prev_dist < EDGE_TRIGGER - MARGIN) {

			theta = odometer.getXYT()[2];
			edge_detected = true;

		} else if (in_margin && current_dist > EDGE_TRIGGER + MARGIN) {

			theta = (average_angle(odometer.getXYT()[2], itheta));
			in_margin = false;
			edge_detected = true;

		} else if (Math.abs(current_dist - EDGE_TRIGGER) < MARGIN) {

			in_margin = true;
			itheta = odometer.getXYT()[2];
			edge_detected = false;

		} else {

			itheta = 0;
			in_margin = false;
			edge_detected = false;

		}
		prev_dist = current_dist;
		return edge_detected;
	}
	/**
	 * Method to know if a falling edge is detected
	 * @return
	 */
	public boolean falling_edge() {
		boolean edge_detected, in_margin = false;

		current_dist = Lab4.usPoller.getDist();

		if (current_dist < EDGE_TRIGGER - MARGIN && prev_dist > EDGE_TRIGGER + MARGIN) {

			theta = odometer.getXYT()[2];
			edge_detected = true;
			Sound.beep();
		} else if (in_margin && current_dist < EDGE_TRIGGER - MARGIN) {
			theta = (average_angle(odometer.getXYT()[2], itheta));

			in_margin = false;
			Sound.beep();
			edge_detected = true;
		} else if (Math.abs(current_dist - EDGE_TRIGGER) < MARGIN) {
			in_margin = true;
			itheta = odometer.getXYT()[2];

			edge_detected = false;
		} else {
			itheta = 0;
			in_margin = false;

			edge_detected = false;
		}
		prev_dist = current_dist;
		return edge_detected;
	}

/**
 * Uses the light sensor to get the position of the robot in space
 */
	private void light_localization() {
		double diff_x, diff_y, x_value, y_value, heading_correction = 0;
		motorcontrol.dime_turn(360, 70, false);
		ArrayList<Double> angles = new ArrayList<Double>();
		while (motorcontrol.isMoving()) {

			if (Lab4.lightPoller.falling(20)) {
				angles.add(odometer.getXYT()[2]);
				Sound.beep();
			}
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		}
		
		
		// double x_value = (angles.get(1)-angles.get(0));
		// double y_value = (angles.get(3)-angles.get(1));
		diff_x = angles.get(2) - angles.get(0);
		diff_y = angles.get(1) - angles.get(3);
		x_value = -LIGHT_OFFSET * (Math.cos(diff_x / 2 * Math.PI / 180));
		y_value = -LIGHT_OFFSET * (Math.cos(diff_y / 2 * Math.PI / 180));
		
		heading_correction = 90 + (360-(diff_y))/2 + 125 - angles.get(2);	
		odometer.setX(heading_correction);
		odometer.setY((360-(diff_y))/2);

	}
	/**
	 * Method to find the right orientation (0 degree) after light sensor localization
	 */
	void localize_heading() {
		int count = 0;
		double new_zero=0;
		ArrayList<Double> angles = new ArrayList<Double>();
		motorcontrol.leftMotor(70);
		motorcontrol.rightMotor(-70);
		
		while(!Lab4.lightPoller.falling(20)) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		}
		motorcontrol.stop();
		angles.add(odometer.getXYT()[2]);
		Sound.beep();
		motorcontrol.dime_turn(10, 70, true);
		motorcontrol.leftMotor(-70);
		motorcontrol.rightMotor(70);
		
		

		while(count<2 ) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
			if(Lab4.lightPoller.falling(20)) {
				Sound.beep();
				count++;
			}
		}
		motorcontrol.stop();
		angles.add(odometer.getXYT()[2]);
		new_zero = GetAngleAverage(angles.get(0), angles.get(1));
		//new_zero = (360-angles.get(1))
		turn_to_heading(new_zero);
		//odometer.setTheta(0);
		
	}
	/**
	 * Method to get the angle between two different angle readings, even when overlapping happens
	 * @param a
	 * @param b
	 * @return
	 */
    static double GetAngleAverage(double a, double b)
    {
        a = a % 360;
        b = b % 360;

        double sum = a + b;
        if (sum > 360 && sum < 540)
        {
            sum = sum % 180;
        }
        return sum / 2;
    }

    /**
     * Method that runs rising edge localization
     * @throws OdometerExceptions
     */
	private void localize_rising() throws OdometerExceptions {
		double theta1 = -1;
		double theta2 = -1;
		double finalHead = 0;

		current_dist = Lab4.usPoller.getDist();
		prev_dist = current_dist;

		do {
			motorcontrol.leftMotor(70);
			motorcontrol.rightMotor(-70);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		} while (!rising_edge());
		theta1 = theta;

		motorcontrol.stop();
		Sound.buzz();

		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here
		}

		do {
			motorcontrol.rightMotor(70);
			motorcontrol.leftMotor(-70);

			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		} while (!rising_edge());

		motorcontrol.stop();
		Sound.buzz();

		sleep(2000);

		theta2 = theta;

		double head_correction;
		if (theta1 > theta2) {
			head_correction = 45 - average_angle(theta1, theta2);

		} else {
			head_correction = 225 - average_angle(theta1, theta2);
		}

		finalHead = (odometer.getXYT()[2] + (360 + head_correction) % 360) % 360;
		odometer.setTheta(finalHead);
	}
	
	 /**
     * Method that runs falling edge localization
     * @throws OdometerExceptions
     */
	private void localize_falling() throws OdometerExceptions {
		double theta1 = -1;
		double theta2 = -1;
		double finalHead = 0;

		odometer = Odometer.getOdometer();

		current_dist = Lab4.usPoller.getDist();
		prev_dist = current_dist;

		do {
			motorcontrol.leftMotor(70);
			motorcontrol.rightMotor(-70);
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		} while (!falling_edge());
		theta1 = theta;

		motorcontrol.stop();
		Sound.buzz();

		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here
		}

		do {
			motorcontrol.rightMotor(70);
			motorcontrol.leftMotor(-70);

			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		} while (!falling_edge());

		motorcontrol.stop();
		Sound.buzz();

		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here
		}

		theta2 = theta;

		double head_correction;
		if (theta1 > theta2) {
			head_correction = 45 - average_angle(theta1, theta2);

		} else {
			head_correction = 225 - average_angle(theta1, theta2);
		}

		finalHead = (odometer.getXYT()[2] + (360 + head_correction) % 360) % 360;
		odometer.setTheta(finalHead);
	}

	/**
	 * Method to make robot turn itself to the right orientation
	 * @param finalHead
	 */
	private void turn_to_heading(double finalHead) {

		double initial_heading, turning_angle;

		initial_heading = odometer.getXYT()[2];

		turning_angle = min_angle(initial_heading, finalHead);

		motorcontrol.dime_turn(turning_angle, 100, true);
		motorcontrol.stop();

	}

	/**
	 * Method to obtain the minimum angle, takes in consideration overlapping
	 * @param ihead
	 * @param fhead
	 * @return
	 */
	public static double min_angle(double ihead, double fhead) {
		double theta;
		theta = (fhead + 360 - ihead) % (360);
		if (theta < 180)
			return theta;
		else
			return theta - 360;
	}

	/**
	 *Average angle calculation method
	 * @param a
	 * @param b
	 * @return
	 */
	private double average_angle(double a, double b) {
		double sum = a + b;
		if (sum > 360) {
			return (sum - 360) / 2;
		} else
			return sum / 2;
	}

	/**
	 * sleep thread for a time wanted
	 * @param time
	 */
	private void sleep(int time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// there is nothing to be done here
		}
	}
}
