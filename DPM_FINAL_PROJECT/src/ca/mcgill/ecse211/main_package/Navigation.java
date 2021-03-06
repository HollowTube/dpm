package ca.mcgill.ecse211.main_package;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

/**
 * This class is used for the robot's navigation in a grid. It
 * obtains the correct heading, rotates the robot towards it and moves the robot
 * to the next point.
 * This class provides the methods necessary for all movements in the field after each localization.
 * 
 * @author Tritin 
 */
public class Navigation {
	private static Odometer odometer;
	private static MotorControl motorcontrol;

	private final int FORWARD_SPEED = 150;

	private double[] position;
	private static double final_heading = 0;
	private int left_speed;
	private int right_speed;

	public Navigation() throws OdometerExceptions {
		Navigation.odometer = Odometer.getOdometer();
		Navigation.motorcontrol = MotorControl.getMotor();
	}

	/**
	 * Convenience method for receiving values from odometer
	 * @return double array size 3 from Odometer object
	 */
	private double[] get_position() {
		return odometer.getXYT();
	}

	/**
	 * This method returns the Euclidean distance.
	 * 
	 * @param dx Change in x
	 * @param dy Change in y
	 * @return Euclidean error, distance
	 */
	public static double euclidian_error(double dx, double dy) {
		double error = Math.sqrt(dx * dx + dy * dy);
		return error;
	}

	/**
	 * This method gives the heading of the next way point, that is, what angle
	 * should the robot turn to in order to arrive at the location quickly
	 * 
	 * @param dx Change in x
	 * @param dy Change in y
	 * @return heading
	 */
	public double getHeading(double dx, double dy) {
		double angle;
		if (dy >= 0.1) {
			angle = (Math.atan(dx / dy) + Math.PI * 2) % (Math.PI * 2);
		}else if(-0.1 < dy && dy<0.1){
			if(dx>0){
				return 90;
			}
			else return 270;
		}
		else {
			angle = (Math.atan(dx / (dy-0.01)) + Math.PI);
		}
		return angle * 180 / Math.PI;
	}

	/**
	 * This method uses a simple bang bang controller to correct differences between
	 * the desired heading, and actual desired heading
	 * 
	 * @param turning_angle
	 */
	private void angle_correction(double turning_angle) {
		int correction = 10;
		if (turning_angle < 0) {
			left_speed = FORWARD_SPEED - correction;
			right_speed = FORWARD_SPEED + correction;
		}

		else {
			left_speed = FORWARD_SPEED + correction;
			right_speed = FORWARD_SPEED - correction;
		}
		motorcontrol.setLeftSpeed(left_speed);
		motorcontrol.setRightSpeed(right_speed);
		motorcontrol.forward();
	}
	
	
	
	/**
	 * Method to turn to a 90 degree offset from a coordinate from current heading.
	 * 
	 * @param xf x coordinate
	 * @param yf y coordinate
	 */
	public void offset90(double xf, double yf) {
		double newAngle;
		position = odometer.getXYT();
		newAngle = (getHeading(xf - position[0], yf-position[1])+270) % 360;
		turn_to_angle(newAngle);
		
		
	}

	/**
	 * This method returns the smallest angle between 2 headings it will return a
	 * negative to turn counterclockwise and return positive for clockwise.
	 * 
	 * @param ihead initial heading of robot
	 * @param fhead desired heading of robot
	 * @return Angle Minimum angle between the two headings
	 */
	private static double min_angle(double ihead, double fhead) {
		double theta;
		theta = (fhead + 360 - ihead) % (360);
		if (theta < 180)
			return theta;
		else	
			return theta - 360;
	}

	/**
	 * This method calculates the smallest angle to rotate for a desired heading and turns on a
	 * dime to it.
	 * 
	 * @param angle Angle position of destination point
	 */
	public void turn_to_angle(double angle) {
		double initial_heading, turning_angle;
		initial_heading = odometer.getXYT()[2];
		final_heading = angle;
		turning_angle = min_angle(initial_heading, final_heading);
		motorcontrol.dimeTurn(turning_angle);
	}

	/**
	 * This method calculates the smallest angle to rotate to the correct heading
	 * and then turns on a dime to reach it, inputs are the desired x and y
	 * positions
	 * 
	 * @param xf x coordinate of destination point
	 * @param yf y coordinate of destination point
	 */
	public void turn_to_destination(double xf, double yf) {
		double position[];
		double dx, dy;
		double initial_heading, turning_angle, xi, yi;
		position = odometer.getXYT();
		xi = position[0];
		yi = position[1];
		initial_heading = position[2];

		dx = (xf) - xi;
		dy = (yf) - yi;
		final_heading = getHeading(dx, dy);
		turning_angle = min_angle(initial_heading, final_heading);
		motorcontrol.dimeTurn(turning_angle);
	}
	
	/**
	 * This boolean method indicates to the robot if a destination is reached yet.
	 * (within some error.)
	 * 
	 * @param xf x coordinate of destination
	 * @param yf y coordinate of destination
	 * @return True if reached, false otherwise
	 */
	public boolean destination_reached(double xf, double yf) {
		double[] position = get_position();	
		
		if (Math.abs(xf - position[0]) < 1.5 && Math.abs(yf - position[1]) < 1.5) {
			return true;
		}
		return false;
	}

}
