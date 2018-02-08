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
	private double position[];
	// private double newheading;
	private final int FORWARD_SPEED = 250;
	private final int SLOW_SPEED = 100;
	private final int ROTATE_SPEED = 75;
	private final double RANGE_THRESHOLD = 0.5;
	private final int HEADING_THRESHOLD = 1;
	private final double TILE_SIZE = 30.48;
	private final double P_CONST = 5;
	private final double ERRORTOL = 3;

	public final int WALLDIST = 35;// Distance to wall * 1.4 (cm) accounting for sensor angle
	public final int MAXCORRECTION = 100; // Bound on correction to prevent stalling
	public final long SLEEPINT = 100; // Display update 2Hz
	public final int MAXDIST = 150; // Max value of valid distance
	public final int FILTER_OUT = 20; // Filter threshold 17
	public static double wallDist;

	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	double leftRadius = 2.2;
	double rightRadius = 2.2;
	double track = 17;
	public static double final_heading = 0;
	public static double absolute_distance = 0;
	public static double dx = 0;
	public static double dy = 0;
	private int left_speed;
	private int right_speed;
	public static boolean obstacleDetected = true;

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
	public void travelTo(double xf, double yf) throws OdometerExceptions {
		double xi, yi, initial_heading, turning_angle, prev_angle = 0;
		absolute_distance = 0;
		final_heading = 0;

		// robot sets initial position here
		position = odometer.getXYT();
		xi = position[0];
		yi = position[1];
		initial_heading = position[2];
		dx = (xf * TILE_SIZE) - xi;
		dy = (yf * TILE_SIZE) - yi;

		final_heading = getHeading(dx, dy);
		turning_angle = min_angle(initial_heading, final_heading);

		turnto(turning_angle);

		// odometer.setXYT(xi, yi, final_heading);

		leftMotor.forward();
		rightMotor.forward();

		do {

			if (Lab3.pcontrol.getDistance() < 50) {
				leftMotor.stop(true);
				rightMotor.stop();

				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}

				move_out();

				Sound.beep();
				position = odometer.getXYT();
				xi = position[0];
				yi = position[1];
				initial_heading = position[2];
				dx = (xf * TILE_SIZE) - xi;
				dy = (yf * TILE_SIZE) - yi;

				final_heading = getHeading(dx, dy);
				turning_angle = min_angle(initial_heading, final_heading);

				turnto(turning_angle);

			}

			position = odometer.getXYT();
			xi = position[0];
			yi = position[1];
			initial_heading = position[2];

			dx = (xf * TILE_SIZE) - xi;
			dy = (yf * TILE_SIZE) - yi;

			absolute_distance = euclidian_error(dx, dy);
			final_heading = getHeading(dx, dy);
			turning_angle = (min_angle(initial_heading, final_heading));

			//slows down the robot as it approaches its destination
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

		// leftMotor.rotate(convertDistance(leftRadius, 2), true);
		// rightMotor.rotate(convertDistance(rightRadius, 2), false);

		leftMotor.stop(true);
		rightMotor.stop(false);
		// odometer.setXYT(xf * TILE_SIZE, xf * TILE_SIZE, final_heading);
		return;

	}

	/**
	 * This method returns the euclidean distance
	 * 
	 * @param dx
	 * @param dy
	 * @return
	 */
	private double euclidian_error(double dx, double dy) {
		return Math.sqrt(dx * dx + dy * dy);
	}

	/**
	 * This method makes the robot turn clockwise or counterclockwise on a dime for
	 * a certain amount of degrees.
	 * 
	 * @param theta
	 */
	public void turnto(double theta) {
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
	private double getHeading(double dx, double dy) {
		double angle;
		if (dy > 0) {
			angle = (Math.atan(dx / dy) + Math.PI * 2) % (Math.PI * 2);
		} else {
			angle = (Math.atan(dx / dy) + Math.PI);
		}
		return angle * 180 / Math.PI;
	}

	private int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method uses a simple p-controller to correct differences between the
	 * desired heading, and actual desired heading
	 * 
	 * @param turning_angle
	 */
	private void angle_correction(double turning_angle) {
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
	private double min_angle(double ihead, double fhead) {
		double theta;
		theta = (fhead + 360 - ihead) % (360);
		if (theta < 180)
			return theta;
		else
			return theta - 360;
	}

	private void turn_to_heading(double xf, double yf) {

		double initial_heading, turning_angle, xi, yi;
		position = odometer.getXYT();
		xi = position[0];
		yi = position[1];
		initial_heading = position[2];
		dx = (xf * TILE_SIZE) - xi;
		dy = (yf * TILE_SIZE) - yi;

		final_heading = getHeading(dx, dy);
		turning_angle = min_angle(initial_heading, final_heading);

		turnto(turning_angle);

		// odometer.setXYT(xi, yi, final_heading);

		leftMotor.forward();
		rightMotor.forward();
	}

	private void wall_correction(double distError) {
		// Sound.buzz();
		if (Math.abs(distError) <= ERRORTOL) { // Case 1: Error in bounds, no correction
			left_speed = FORWARD_SPEED;
			right_speed = FORWARD_SPEED;
			leftMotor.setSpeed(left_speed); // If correction was being applied on last
			rightMotor.setSpeed(right_speed); // update, clear it
		}

		else if (distError > 0) { // Case 2: positive error, move away from wall
			left_speed = FORWARD_SPEED - 150;
			right_speed = FORWARD_SPEED + 150;
			leftMotor.setSpeed(left_speed);
			rightMotor.setSpeed(right_speed);
		}

		else if (distError < 0) { // Case 3: negative error, move towards wall
			left_speed = FORWARD_SPEED + 150;
			right_speed = FORWARD_SPEED - 150;
			leftMotor.setSpeed(left_speed);
			rightMotor.setSpeed(right_speed);
		}

	}

	/**
	 * This method makes the robot turn away from the detected wall and move a bit
	 * forward to avoid the obstacle
	 */
	private void move_out() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(leftRadius, track, 90.0), true);
		rightMotor.rotate(-convertAngle(rightRadius, track, 90.0), false);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(leftRadius, 1 * TILE_SIZE), true);
		rightMotor.rotate(convertDistance(rightRadius, 1 * TILE_SIZE), false);

		leftMotor.forward();
		rightMotor.forward();
	}
}
