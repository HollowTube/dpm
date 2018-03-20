package ca.mcgill.ecse211.main_package;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
/**
 * This class handles all of the motor control. If a method wants to access the
 * motors, it should do so through this class.
 * 
 * @author tritin
 *
 */
public class MotorControl {
	public static RegulatedMotor leftMotor;
	public static RegulatedMotor rightMotor;

	private static double radius;
	private static double track;
	private final int ROTATE_SPEED = 150;
	private final int PATH_SPEED = 200;
	private static MotorControl motorcontrol = null;

	
	
	public MotorControl(EV3LargeRegulatedMotor leftmotor, EV3LargeRegulatedMotor rightmotor, double radius, double track) {
		MotorControl.leftMotor = leftmotor;
		MotorControl.rightMotor = rightmotor;
		MotorControl.radius = radius;
		MotorControl.track = track;
	}

	public synchronized static MotorControl getMotor(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, double radius, double track) {
		leftMotor.setSpeed(120);
		rightMotor.setSpeed(120);
		leftMotor.setAcceleration(1000);
		rightMotor.setAcceleration(1000);
		if (motorcontrol != null) { // Return existing object
			return motorcontrol;
		} else { // create object and return it
			motorcontrol = new MotorControl(leftMotor, rightMotor, radius, track);
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
	public void goAround(int rotate_speed, int dist) {
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
	public void moveSetDistance(double path_distance) {
		leftMotor.rotate(convertDistance(radius, path_distance), true); // travel straight
		rightMotor.rotate(convertDistance(radius, path_distance), false);
	}

	/**
	 * moves left motor forwards
	 */
	public void leftForwards() {
		leftMotor.forward();
	}
	/**
	 * moves left motor backwards
	 */
	public void leftBackwards(){
		leftMotor.backward();
	}
	/**
	 * Moves right motor forwards
	 */
	public void rightForwards() {
		rightMotor.forward();
	}
	/*
	 * Moves right motor backwards
	 */
	public void rightBackwards(){
		rightMotor.backward();
	}
	/**
	 * gives the forward command to both motors
	 */
	public void forward() {
		leftMotor.forward();
		rightMotor.forward();
	}
	public void backward(){
		leftMotor.backward();
		rightMotor.backward();
	}

	/**
	 * left motor stops
	 */
	public void leftStop() {
		leftMotor.stop(true);
	}

	/**
	 * right motor stops
	 */
	public void rightStop() {
		rightMotor.stop(true);
	}

	/**
	 * This method stops the left and right motors almost simultaneously
	 */
	public void stop() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	/*
	 * This method makes the robot turn clockwise 
	 */
	public void turnCW(){
		leftMotor.forward();
		rightMotor.backward();
	}
	/*
	 * This method makes the robot turn counter-clockwise 
	 */
	public void turnCCW(){
		leftMotor.backward();
		rightMotor.forward();
	}
	/**
	 * Set accelearation of the motors
	 */
	public void setAcceleration(int acceleration){
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
	}
	/**
	 * This method makes the robot turn on a dime for a certain amount of degrees,
	 * positive rotation means clockwise ,negative rotation means counter-clockwise
	 * 
	 * @param rotation
	 */
	public void dimeTurn(double rotation) {

		if (rotation < 0) {
			leftMotor.rotate(-convertAngle(radius, track, Math.abs(rotation)), true);
			rightMotor.rotate(convertAngle(radius, track, Math.abs(rotation)), false);
		} else {
			leftMotor.rotate(convertAngle(radius, track, Math.abs(rotation)), true);
			rightMotor.rotate(-convertAngle(radius, track, Math.abs(rotation)), false);
		}
	}

	/**
	 * Boolean method to detect if the robot is moving or not.
	 * 
	 * @return true or false following condition
	 */
	public boolean isMoving() {
		if (leftMotor.isMoving() || rightMotor.isMoving())
			return true;
		return false;
	}

	/**
	 * Method to set the speed for the left motor.
	 * 
	 * @param speed
	 */
	public void leftMotor(int speed) {
		leftMotor.setSpeed(speed);
		if (speed > 0)
			leftMotor.forward();
		else
			leftMotor.backward();
	}
	
	/**
	 * Method to set the speed for the right motor.
	 * 
	 * @param speed
	 */
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
	 * Works exactly like the default rotate function for the left motor except you
	 * only need to input the distance you want
	 * 
	 * @param rotation
	 * @param block
	 */
	public void leftRot(double distance, boolean block) {
		if (distance > 0) {
			leftMotor.rotate(convertDistance(radius, Math.abs(distance)), block);
		} else {
			leftMotor.rotate(-convertDistance(radius, Math.abs(distance)), block);
		}
	}

	/**
	 * Works exactly like the default rotate function for the right motor except you
	 * only need to input the distance you want.
	 * 
	 * @param distance
	 * @param block
	 */
	public void rightRot(double distance, boolean block) {
		if (distance > 0) {
			rightMotor.rotate(convertDistance(radius, Math.abs(distance)), block);
		} else {
			rightMotor.rotate(-convertDistance(radius, Math.abs(distance)), block);
		}
	}

	private int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Method to change the heading of the robot towards the proper destination.
	 * This is done using the angle received as a parameter.
	 * 
	 * @param theta
	 * @author Alexandre
	 */
	public void turnTo(double theta) {
		// if theta > 180 || theta < -180
		// then turn 360-theta or 360-Math.abs(theta)
		if (theta >= 180) {
			// turn negative direction (left)
			theta = (360 - theta);
			setLeftSpeed(ROTATE_SPEED);
			setRightSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(radius, track, theta), true);
			rightMotor.rotate(convertAngle(radius, track, theta), false);
		} else if (theta < -180) {
			// turn positive direciton (right)
			theta = 360 - Math.abs(theta);
			setLeftSpeed(ROTATE_SPEED);
			setRightSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(radius, track, theta), true);
			rightMotor.rotate(-convertAngle(radius, track, theta), false);
		} else {
			if (theta < 0) {
				// negative, turn left
				setLeftSpeed(ROTATE_SPEED);
				setRightSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(radius, track, Math.abs(theta)), true);
				rightMotor.rotate(convertAngle(radius, track, Math.abs(theta)), false);
			} else {
				// positive, turn right
				setLeftSpeed(ROTATE_SPEED);
				setRightSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(radius, track, theta), true);
				rightMotor.rotate(-convertAngle(radius, track, theta), false);
				}
			}
	}
	public void setRadius(double new_radius) {
		this.radius = new_radius;
	}
	
	public void setTrack(double new_track) {
		this.track = new_track;
	}
}