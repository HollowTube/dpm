package ca.mcgill.ecse211.main_package;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.navigation.Navigator;

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
	private final static int EDGE_TRIGGER = 80;
	private final static int MARGIN = 10;
	private final static double LIGHT_OFFSET = 7.7;
	private final static int SCANNING_SPEED = 100;
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
	public Localization(MotorControl motorcontrol) throws OdometerExceptions {
		Localization.odometer = Odometer.getOdometer();
		Localization.motorcontrol = motorcontrol;
	}

/**
 * @throws InterruptedException
 */
public void head_to_origin()  {
	Button.waitForAnyPress();
	
	motorcontrol.forward(100, 100);
	while(!Lab5.lightPollerReflected.falling(20)) {
		sleep(50);
	}
	motorcontrol.stop();
	motorcontrol.leftRot(LIGHT_OFFSET,false);
	Sound.beep();
	odometer.setY(0);
	
	motorcontrol.dime_turn(-90);
	
	motorcontrol.forward(100, 100);
	while(!Lab5.lightPollerReflected.falling(20)) {
		sleep(50);
	}
	motorcontrol.stop();
	motorcontrol.leftRot(LIGHT_OFFSET, false);
	
	
	odometer.setX(0);
	motorcontrol.dime_turn(60);
	localize_heading();
}
/**
 * Method to know if a rising edge is detected
 * @return
 */
	private boolean rising_edge() {
		boolean edge_detected, in_margin = false;

		current_dist = Lab5.usPoller.getDist();

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
	private boolean falling_edge() {
		boolean edge_detected, in_margin = false;

		current_dist = Lab5.usPoller.getDist();

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
	
public void light_localization() {
		double diff_x, diff_y, x_value, y_value, heading_correction = 0;
		motorcontrol.dime_turn(360);
		ArrayList<Double> angles = new ArrayList<Double>();
		while (motorcontrol.isMoving()) {

			if (Lab5.lightPollerReflected.falling(20)) {
				angles.add(odometer.getXYT()[2]);
				Sound.beep();
			}
			sleep(50);
		}
		
		diff_x = angles.get(2) - angles.get(0);
		diff_y = angles.get(1) - angles.get(3);
		x_value = -LIGHT_OFFSET * (Math.cos(diff_x / 2 * Math.PI / 180));
		y_value = -LIGHT_OFFSET * (Math.cos(diff_y / 2 * Math.PI / 180));
		
		heading_correction = 90 + (360-(diff_y))/2 + 125 - angles.get(2);	
		odometer.setX(heading_correction);
		odometer.setY(y_value);
	}
	/**
	 * Method to find the right orientation (0 degree) after light sensor localization
	 */
	public void localize_heading() {
		int count = 0;
		double new_zero = 0;
		ArrayList<Double> angles = new ArrayList<Double>();
		
		motorcontrol.leftMotor(SCANNING_SPEED);
		motorcontrol.rightMotor(-SCANNING_SPEED);
		
		while(!Lab5.lightPollerReflected.falling(20)) sleep(50);
		
		
		motorcontrol.stop();
		angles.add(odometer.getXYT()[2]);
		Sound.beep();
		
		motorcontrol.dime_turn(10);
		motorcontrol.leftMotor(-SCANNING_SPEED);
		motorcontrol.rightMotor(SCANNING_SPEED);
		
		while(count<2 ) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
			if(Lab5.lightPollerReflected.falling(20)) {
				Sound.beep();
				count++;
			}
		}
		motorcontrol.stop();
		angles.add(odometer.getXYT()[2]);
		new_zero = GetAngleAverage(angles.get(0), angles.get(1));
		//new_zero = (360-angles.get(1))
		turn_to_heading(new_zero+180+20);
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
	public void localize_rising() throws OdometerExceptions {
		double theta1 = -1;
		double theta2 = -1;
		double finalHead = 0;
		double head_correction;


		current_dist = Lab5.usPoller.getDist();
		prev_dist = current_dist;

		do {
			motorcontrol.leftMotor(SCANNING_SPEED);
			motorcontrol.rightMotor(-SCANNING_SPEED);

			sleep(50);
		} while (!rising_edge());
		theta1 = theta;

		motorcontrol.stop();
		Sound.buzz();

		sleep(2000);


		rotate_until_rising(true);

		motorcontrol.stop();
		Sound.buzz();

		sleep(2000);

		theta2 = theta;

		head_correction = get_heading_correction(theta1, theta2);

		finalHead = (odometer.getXYT()[2] + (360 + head_correction) % 360) % 360;
		odometer.setTheta(finalHead);
	}

	/**rotates until sensor dete
	 * 
	 */
	private void rotate_until_rising(boolean clockwise) {
		do {
			motorcontrol.rightMotor(SCANNING_SPEED);
			motorcontrol.leftMotor(-SCANNING_SPEED);

			sleep(50);
		} while (!rising_edge());
	}

	/**
	 * @param theta1
	 * @param theta2
	 * @return
	 */
	private double get_heading_correction(double theta1, double theta2) {
		double head_correction;
		if (theta1 > theta2) {
			head_correction = 45 - average_angle(theta1, theta2);

		} else {
			head_correction = 225 - average_angle(theta1, theta2);
		}
		return head_correction;
	}	
	 /**
     * Method that runs falling edge localization
     * @throws OdometerExceptions
     */
	public void localize_falling() throws OdometerExceptions {
		double theta1 = -1;
		double theta2 = -1;

		odometer = Odometer.getOdometer();

		current_dist = Lab5.usPoller.getDist();
		prev_dist = current_dist;

		do {
			motorcontrol.leftMotor(SCANNING_SPEED);
			motorcontrol.rightMotor(-SCANNING_SPEED);
			sleep(20);
		} while (!falling_edge());
		theta1 = theta;

		motorcontrol.stop();
		Sound.buzz();

		sleep(2000);

		do {
			motorcontrol.rightMotor(SCANNING_SPEED);
			motorcontrol.leftMotor(-SCANNING_SPEED);
				sleep(20);
		} while (!falling_edge());

		motorcontrol.stop();
		Sound.buzz();

		sleep(2000);

		theta2 = theta;

		double head_correction = get_heading_correction(theta1,theta2);

		odometer.update(0,0,head_correction);
		turn_to_heading(0);
	}

	/**
	 * Method to make robot turn itself to the right orientation
	 * @param finalHead
	 */
	private void turn_to_heading(double finalHead) {

		double initial_heading, turning_angle;

		initial_heading = odometer.getXYT()[2];

		turning_angle = min_angle(initial_heading, finalHead);

		motorcontrol.dime_turn(-turning_angle);
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
