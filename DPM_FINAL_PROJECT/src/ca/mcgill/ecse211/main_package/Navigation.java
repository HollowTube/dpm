package ca.mcgill.ecse211.main_package;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
//import lejos.hardware.motor.Motor;
import lejos.robotics.RegulatedMotor;

public class Navigation {
	private static Odometer odometer;
	private static MotorControl motorcontrol;

	private double position[];
	// private double newheading;
	private final int FORWARD_SPEED = 150;
	private final int HEADING_THRESHOLD = 1;


	public static double final_heading = 0;
	public static double absolute_distance = 0;
	public static double dx = 0;
	public static double dy = 0;
	private int left_speed;
	private int right_speed;
	private double heading_error;

	public Navigation() throws OdometerExceptions {
		Navigation.odometer = Odometer.getOdometer();
		Navigation.motorcontrol = MotorControl.getMotor();
	}
	
	private double[] get_position() {
		return odometer.getXYT();
	}

	// travels straight
	public void travelTo(double xf, double yf) {
		position = get_position();
		motorcontrol.setLeftSpeed(left_speed);
		motorcontrol.setRightSpeed(right_speed);
		motorcontrol.forward();
		heading_error = position[2] - getHeading(xf - position[0], yf - position[1]);
		if (Math.abs(heading_error) > HEADING_THRESHOLD) {
			angle_correction(heading_error);
		} else {
			left_speed = FORWARD_SPEED;
			right_speed = FORWARD_SPEED;
		}
	}

	/**
	 * This method returns the euclidian distance
	 * 
	 * @param dx
	 * @param dy
	 * @return
	 */
	private static double euclidian_error(double dx, double dy) {
		double error = Math.sqrt(dx * dx + dy * dy);
		// System.out.println(error);
		return error;
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

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method uses a simple bang bang controller to correct differences between the
	 * desired heading, and actual desired heading
	 * 
	 * @param turning_angle
	 */
	private void angle_correction(double turning_angle) {
		int correction = 5;
		if(turning_angle<0) {
			left_speed = FORWARD_SPEED-correction;
			right_speed = FORWARD_SPEED+correction;
		}
		
		else {
			left_speed = FORWARD_SPEED+correction;
			right_speed = FORWARD_SPEED-correction;
		}
	}

	/**
	 * This method returns the smallest angle between 2 headings it will return a
	 * negative to turn counterclockwise and return positive for clockwise
	 * 
	 * @param ihead
	 * @param fhead
	 * @return Angle in degrees
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
	 * this method calculates the smallest angle to rotate to the correct heading
	 * and then turns on a dime to reach it
	 * 
	 * @param xf
	 * @param yf
	 */
	public void turn_to_heading(double xf, double yf) {
		double position[];
		double dx, dy;
		double initial_heading, turning_angle, xi, yi;
		position = odometer.getXYT();
		xi = position[0];
		yi = position[1];
		initial_heading = position[2];

		dx = (xf) - xi;
		dy = (yf) - yi;
		Sound.beep();
		final_heading = getHeading(dx, dy);
		turning_angle = min_angle(initial_heading, final_heading);
		motorcontrol.dime_turn(turning_angle);
	}

	
	public boolean destination_reached(double xf, double yf) {
		double[] position = get_position();
		if (euclidian_error(xf - position[0], yf - position[1]) < 0.75) {
			return true;
		}
		return false;
	}

}
