package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;

/**
 * This class handles all of the motor control. If a method wants to access the
 * motors, it should do so through this class
 * 
 * @author tritin
 *
 */
public class MotorControl {
	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	private double bridge_radius = 2.05;
	private double bridge_track = 14.45;
	
	private double tunnel_radius = 2.05;
	private double tunnel_track = 14.45;
	
	private double radius = 2.05;
	private double track = 14.45;
	private final int ROTATE_SPEED = 100;
	private static Odometer odometer;
	private static MotorControl motorcontrol = null;

	public MotorControl(EV3LargeRegulatedMotor leftmotor, EV3LargeRegulatedMotor rightmotor) {
		MotorControl.leftMotor = leftmotor;
		MotorControl.rightMotor = rightmotor;
		// MotorControl.odometer = Odometer.getOdometer();
	}

	public synchronized static MotorControl getMotor(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		if (motorcontrol != null) { // Return existing object
			return motorcontrol;
		} else { // create object and return it
			motorcontrol = new MotorControl(leftMotor, rightMotor);
			return motorcontrol;
		}

	}

	public synchronized static MotorControl getMotor() {

		if (motorcontrol == null) {
			System.out.println("motor Control not be initialized");
		}
		return motorcontrol;
	}

	/**
	 * This method makes the robot turn 90 degrees clockwise. It then moves some
	 * distance "dist" and does another 90 degree turn counter-clockwise. Finally it
	 * moves some distance "dist" again. You can also set the speed at which it
	 * accomplishes this
	 * 
	 * @param ROTATE_SPEED
	 * @param dist
	 */
	public void go_around(int rotate_speed, int dist) {
		leftMotor.setSpeed(rotate_speed);
		rightMotor.setSpeed(rotate_speed);

		leftMotor.rotate(convertAngle(radius, track, 90.0), true);
		rightMotor.rotate(-convertAngle(radius, track, 90.0), false);

		leftMotor.rotate(convertDistance(radius, dist), true);
		rightMotor.rotate(convertDistance(radius, dist), false);

		leftMotor.rotate(-convertAngle(radius, track, 90.0), true);
		rightMotor.rotate(convertAngle(radius, track, 90.0), false);

		leftMotor.rotate(convertDistance(radius, dist), true);
		rightMotor.rotate(convertDistance(radius, dist), false);
	}

	/**
	 * This method moves the bot forward and sets the left and right motor speeds
	 * independently
	 * 
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void forward(int leftSpeed, int rightSpeed) {

		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);

		leftMotor.forward();
		rightMotor.forward();
	}
	
	
	/**
	 * gives the forward command to both motors
	 */
	public void forward() {
		leftMotor.forward();
		rightMotor.forward();
	}
	
	

	/**
	 * This method stops the left and right motors almost simultaneously
	 */
	public void stop() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * This method makes the robot turn on a dime for a certain amount of degrees,
	 * positive rotation means clockwise ,negative rotation means counter-clockwise
	 * 
	 * @param rotation
	 */
	public void dime_turn(double rotation) {

		if (rotation < 0) {
			leftMotor.rotate(-convertAngle(radius, track, Math.abs(rotation)), true);
			rightMotor.rotate(convertAngle(radius, track, Math.abs(rotation)), false);
		} else {
			leftMotor.rotate(convertAngle(radius, track, Math.abs(rotation)), true);
			rightMotor.rotate(-convertAngle(radius, track, Math.abs(rotation)), false);
		}
	}

	public boolean isMoving() {
		if (leftMotor.isMoving() || rightMotor.isMoving())
			return true;
		return false;
	}

	public void leftMotor(int speed) {
		leftMotor.setSpeed(speed);
		if (speed > 0)
			leftMotor.forward();
		else
			leftMotor.backward();

	}

	public void rightMotor(int speed) {
		rightMotor.setSpeed(speed);
		if (speed > 0)
			rightMotor.forward();
		else
			rightMotor.backward();

	}

	public void setLeftSpeed(int speed) {
		leftMotor.setSpeed(speed);
	}

	public void setRightSpeed(int speed) {
		rightMotor.setSpeed(speed);
	}

	// public void turn_to_heading(double finalHead) {
	//
	// double initial_heading, turning_angle;
	//
	// initial_heading = odometer.getXYT()[2];
	//
	// turning_angle = Navigation.min_angle(initial_heading, finalHead);
	//
	// dime_turn(turning_angle, 100, true);
	// stop();
	//
	// }

	/**
	 * Works exactly like the default rotate function for the left motor execpt you only need to input
	 * the distance you want
	 * 
	 * @param rotation
	 * @param block
	 */
	public void leftRot(double distance, boolean block) {
		if(distance>0) {
			leftMotor.rotate(convertDistance(radius, Math.abs(distance)), block);
		}
		else {
			leftMotor.rotate(-convertDistance(radius, Math.abs(distance)), block);
		}
	}

	public void rightRot(double distance, boolean block) {
		if(distance>0) {
			rightMotor.rotate(convertDistance(radius, Math.abs(distance)), block);
		}
		else {
			rightMotor.rotate(-convertDistance(radius, Math.abs(distance)), block);
		}
	}

	private int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public void turnto(double theta) {
		double absTheta = Math.abs(theta);
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		if (theta > 0) {
			leftMotor.rotate(convertAngle(radius, track, absTheta), true);
			rightMotor.rotate(-convertAngle(radius, track, absTheta), false);
		} else {
			leftMotor.rotate(-convertAngle(radius, track, absTheta), true);
			rightMotor.rotate(convertAngle(radius, track, absTheta), false);
		}
	}
}
