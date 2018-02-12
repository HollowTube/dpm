package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import java.util.*;

public class Localization {
	static boolean measuring = true;

	private final static int EDGE_TRIGGER = 110;;
	private final static int MARGIN = 10;
	private static Odometer odometer;
	private static MotorControl motorcontrol;

	double current_dist, prev_dist, change_inDist = 0;
	double theta, itheta = 0;
	boolean in_margin = false;

	public Localization() throws OdometerExceptions {
		Localization.odometer = Odometer.getOdometer();
		Localization.motorcontrol = Lab4.motorControl;
	}

	public void go() throws OdometerExceptions {
		motorcontrol.dime_turn(-360, 70, false);
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
		
		double x_value = -9*(Math.cos((angles.get(0)-angles.get(2))/2));
		double y_value = -9*(Math.cos((angles.get(1)-angles.get(3))/2));
		odometer.setX(x_value);
		odometer.setY(y_value);

	}

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

	private void light_localization() {

		motorcontrol.rightMotor(-70);
		motorcontrol.rightMotor(70);

	}

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

	private void turn_to_heading(double finalHead) {

		double initial_heading, turning_angle;

		initial_heading = odometer.getXYT()[2];

		turning_angle = min_angle(initial_heading, finalHead);

		motorcontrol.dime_turn(turning_angle, 100, true);
		motorcontrol.stop();

	}

	public static double min_angle(double ihead, double fhead) {
		double theta;
		theta = (fhead + 360 - ihead) % (360);
		if (theta < 180)
			return theta;
		else
			return theta - 360;
	}

	private double average_angle(double a, double b) {
		double sum = a + b;
		if (sum > 360) {
			return (sum - 360) / 2;
		} else
			return sum / 2;
	}

	private void sleep(int time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// there is nothing to be done here
		}
	}
}
