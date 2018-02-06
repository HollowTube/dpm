package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
//import lejos.hardware.motor.Motor;
import lejos.robotics.RegulatedMotor;
//import ca.mcgill.ecse211.odometer.Odometer;
//import ca.mcgill.ecse211.odometer.OdometerExceptions;
//import lejos.hardware.Button;
//import lejos.hardware.Sound;

public class Navigation {
	private static Odometer odometer;
	private static double position[];
	// private static double newheading;
	private static final int FORWARD_SPEED = 250;
	private static final int SLOW_SPEED = 100;
	private static final int ROTATE_SPEED = 75;
	private static final double RANGE_THRESHOLD = 0.5;
	private static final int HEADING_THRESHOLD = 1;
	private static final double TILE_SIZE = 30.48;
	private static final double P_CONST = 5;
	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	static double leftRadius = 2.2;
	static double rightRadius = 2.2;
	static double track = 17;
	public static double final_heading = 0;
	public static double absolute_distance = 0;
	private static int left_speed;
	private static int right_speed;

	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
		Navigation.odometer = Odometer.getOdometer();
		Navigation.leftMotor = leftMotor;
		Navigation.rightMotor = rightMotor;
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(1000);
		}
	}

	/**
	 * This method makes the robot travel to a certain coordinate, it will also hand
	 * control to another method if an obstacle is ever detected. It depends on the
	 * odometer data to get the correct heading
	 * 
	 * @param xf
	 * @param yf
	 * 
	 * @throws OdometerExceptions
	 */
	public boolean travelTo(double xf, double yf) throws OdometerExceptions {
		double xi, yi, dx, dy, initial_heading, turning_angle, prev_angle = 0;

		// robot sets initial position here
		position = odometer.getXYT();
		xi = position[0];
		yi = position[1];
		initial_heading = position[2];
		dx = (xf * TILE_SIZE) - xi;
		dy = (yf * TILE_SIZE) - yi;

		final_heading = getHeading(dx, dy);
		turning_angle = min_angle(initial_heading, final_heading);
		absolute_distance = euclidian_error(dx, dy);

		prev_angle = turning_angle;

		turnto(turning_angle);

		leftMotor.forward();
		rightMotor.forward();

		do {

			if (Lab3.pcontrol.obstacleDetected()) {
				leftMotor.stop();
				rightMotor.stop();
				
				
				Lab3.pcontrol.avoid();
				

			}

			// if (Math.abs(turning_angle) > HEADING_THRESHOLD+10 && absolute_distance > 10)
			// {
			// turnto(turning_angle);
			// }
			position = odometer.getXYT();
			xi = position[0];
			yi = position[1];
			initial_heading = position[2];

			dx = (xf * TILE_SIZE) - xi;
			dy = (yf * TILE_SIZE) - yi;

			absolute_distance = euclidian_error(dx, dy);
			final_heading = getHeading(dx, dy);
			turning_angle = (min_angle(initial_heading, final_heading));

			if (absolute_distance < 10) {
				left_speed = SLOW_SPEED;
				right_speed = SLOW_SPEED;
			} else {
				left_speed = FORWARD_SPEED;
				right_speed = FORWARD_SPEED;
			}

			if (Math.abs(prev_angle) > HEADING_THRESHOLD && absolute_distance > 10) {
				angle_correction(prev_angle);
			}
			prev_angle = turning_angle;
			leftMotor.setSpeed(left_speed);
			rightMotor.setSpeed(right_speed);

			leftMotor.forward();
			rightMotor.forward();

			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		} while (absolute_distance > RANGE_THRESHOLD);

		leftMotor.rotate(convertDistance(leftRadius, 2), true);
		rightMotor.rotate(convertDistance(rightRadius, 2), false);

		// Math.abs(dx) > RANGE_THRESHOLD && Math.abs(dy) > RANGE_THRESHOLD
		leftMotor.stop(true);
		rightMotor.stop(false);
		// Sound.beep();
		// Sound.beep();

		return true;
	}

	/**
	 * This method returns the euclidean distance
	 * 
	 * @param dx
	 * @param dy
	 * @return
	 */
	private static double euclidian_error(double dx, double dy) {
		return Math.sqrt(dx * dx + dy * dy);
	}

	/**
	 * This method makes the robot turn clockwise or counterclockwise on a dime for
	 * a certain amount of degrees.
	 * 
	 * @param theta
	 */
	public static void turnto(double theta) {
		double absTheta = Math.abs(theta);
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		if (theta > 0) {
			leftMotor.rotate(convertAngle(leftRadius, track, absTheta), true);
			rightMotor.rotate(-convertAngle(rightRadius, track, absTheta), false);
		} else {
			leftMotor.rotate(-convertAngle(leftRadius, track, absTheta), true);
			rightMotor.rotate(convertAngle(rightRadius, track, absTheta), false);
		}
	}

	/**
	 * This method gives the heading of the next way point, that is, what angle
	 * should the robot be at in order to arrive at the location quickly
	 * 
	 * @param dx
	 * @param dy
	 * @return heading in degrees
	 */
	private static double getHeading(double dx, double dy) {
		double angle;
		if (dy > 0) {
			angle = (Math.atan(dx / dy) + Math.PI * 2) % (Math.PI * 2);
		} else {
			angle = (Math.atan(dx / dy) + Math.PI);
		}
		return angle * 180 / Math.PI;
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method uses a simple p-controller to correct differences between the
	 * desired heading, and actual desired heading
	 * 
	 * @param turning_angle
	 */
	private static void angle_correction(double turning_angle) {
		int correction;
		correction = (int) (P_CONST * turning_angle);
		left_speed = left_speed + correction;
		right_speed = right_speed - correction;
	}

	/**
	 * This method returns the smallest angle between 2 headings it wll return a
	 * negative to turn counterclockwise and return positive for clockwise
	 * 
	 * @param ihead
	 * @param fhead
	 * @return Angle in degrees
	 */
	private static double min_angle(double ihead, double fhead) {
		double theta;
		theta = (fhead + 360 - ihead) % (360);
		if (theta < 180)
			return theta;
		else
			return theta - 360;
	}

	private static void clearObstacle() {
		boolean hasObstacle = true;
		

	}
}
