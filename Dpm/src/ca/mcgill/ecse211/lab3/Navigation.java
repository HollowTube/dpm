package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.robotics.RegulatedMotor;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class Navigation {
	private static Odometer odometer;
	private static double position[];
	private static double newheading;
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 75;
	private static final int RANGE_THRESHOLD = 2;
	private static final int HEADING_THRESHOLD = 3;
	private static final double TILE_SIZE = 30.48;
	private static final double P_CONST = 1.5;
	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	static double leftRadius = 2.2;
	static double rightRadius = 2.2;
	static double track = 17;
	public static double final_heading;
	public static double absolute_distance;

	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius,
			double rightRadius, double track) throws OdometerExceptions {
		this.odometer = Odometer.getOdometer();
		Navigation.leftMotor = leftMotor;
		Navigation.rightMotor = rightMotor;
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(1000);
		}
		// Navigation.leftRadius = leftRadius;
		// this.rightRadius = rightRadius;
		// this.track = track;
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
	public static boolean travelTo(double xf, double yf) throws OdometerExceptions {
		double xi, yi, dx, dy, initial_heading, turning_angle;

		// robot sets initial position here
		position = odometer.getXYT();
		xi = position[0];
		yi = position[1];
		initial_heading = position[2];
		dx = (xf * TILE_SIZE) - xi;
		dy = (yf * TILE_SIZE) - yi;

		final_heading = getHeading(dx, dy);
		turning_angle = (min_angle(initial_heading, final_heading));

		turnto(turning_angle);

		leftMotor.forward();
		rightMotor.forward();

		do {

			position = odometer.getXYT();
			xi = position[0];
			yi = position[1];
			initial_heading = position[2];
			dx = (xf * TILE_SIZE) - xi;
			dy = (yf * TILE_SIZE) - yi;
			// Sound.beep();
			// newheading = getHeading(dx, dy);

			final_heading = getHeading(dx, dy);
			turning_angle = (min_angle(initial_heading, final_heading));
			
			absolute_distance = euclidian_error(dx,dy);
			
			if (Math.abs(turning_angle) > HEADING_THRESHOLD) {
				angle_correction(turning_angle);
			} else {
				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);
			}
			if (obstacleDetected()) {
				// TODO add wallfollower, return control after coast is clear
			}

			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		} while (absolute_distance > 5);

		// Math.abs(dx) > RANGE_THRESHOLD && Math.abs(dy) > RANGE_THRESHOLD
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();
		Sound.beep();
		return true;
	}
/**
 * This method returns the euclidean distance 
 * @param dx
 * @param dy
 * @return
 */
	private static double euclidian_error(double dx, double dy) {
		return Math.sqrt(dx*dx + dy*dy);
	}
	
	private static boolean obstacleDetected() {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * This method makes the robot turn clockwise or counterclockwise on a dime for
	 * a certain amount of degrees.
	 * 
	 * @param theta
	 */
	public static void turnto(double theta) {
		double absTheta = Math.abs(theta);

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
		leftMotor.setSpeed(FORWARD_SPEED - correction);
		leftMotor.setSpeed(FORWARD_SPEED + correction);

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
}
