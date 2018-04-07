package ca.mcgill.ecse211.main_package;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.RegulatedMotor;
/**
 * This class handles all of the motor control. If a any method wants to access the
 * motors, it should do so through this class. 
 * This class handles all the movements for the robot; moving forward,moving backwards and rotating.
 * 
 * @author Tritin
 *
 */
public class MotorControl {
	public static RegulatedMotor leftMotor;
	public static RegulatedMotor rightMotor;
	public static RegulatedMotor usMotor;

	private static double radius;
	private static double track;
	private final int ROTATE_SPEED = 150;
	private final int PATH_SPEED = 200;
	private static MotorControl motorcontrol = null;

	
	/**
	 * This is the class constructor.
	 * 
	 * @param leftmotor Left motor
	 * @param rightmotor Right motor
	 * @param radius Wheel radius
	 * @param track Robot track
	 */
	public MotorControl(EV3LargeRegulatedMotor leftmotor, EV3LargeRegulatedMotor rightmotor,EV3MediumRegulatedMotor usMotor, double radius, double track) {
		MotorControl.leftMotor = leftmotor;
		MotorControl.rightMotor = rightmotor;
		MotorControl.usMotor = usMotor;
		MotorControl.radius = radius;
		MotorControl.track = track;
	}

	public synchronized static MotorControl getMotor(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, EV3MediumRegulatedMotor usMotor, double radius, double track) {
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.setAcceleration(1750);
		rightMotor.setAcceleration(1750);
		if (motorcontrol != null) { // Return existing object
			return motorcontrol;
		} else { // create object and return it
			motorcontrol = new MotorControl(leftMotor, rightMotor,usMotor, radius, track);
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
	 * @param ROTATE_SPEED Speed at which the wheels rotate
	 * @param dist Distance travel
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
	 * This method moves the robot forward and sets the left and right motor speeds independently.
	 * 
	 * @param leftSpeed Speed left motor
	 * @param rightSpeed Speed right motor
	 */
	public void moveSetDistance(double path_distance) {
		leftMotor.rotate(convertDistance(radius, path_distance), true); // travel straight
		rightMotor.rotate(convertDistance(radius, path_distance), false);
	}
	/**
	 * Moves left motor forwards
	 */
	public void leftForwards() {
		leftMotor.forward();
	}
	/**
	 * Moves left motor backwards
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
	 * This method gives the forward command to both motors
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
	 * Stops left motor
	 */
	public void leftStop() {
		leftMotor.stop(true);
	}

	/**
	 * Stops right motor
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
	/**
	 * This method makes the Ultrasonic turn clockwise 
	 */
	public void turnCW(){
		usMotor.rotate(-91,false);
	}
	/**
	 * This method makes the Ultrasonic turn counter-clockwise 
	 */
	public void turnCCW(){
		usMotor.rotate(90,false);
	}
	public void rotateCCW() {
		leftMotor.backward();
		rightMotor.forward();
	}
	public void rotateCW() {
		rightMotor.backward();
		leftMotor.forward();
	}
	/**
	 * This method sets acceleration of the motors
	 */
	public void setAcceleration(int acceleration){
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
	}
	/**
	 * This method makes the robot turn on a dime for a certain amount of degrees,
	 * positive rotation means clockwise, negative rotation means counter-clockwise.
	 * 
	 * @param rotation Angle of rotation
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
	 * @return True is robot is moving, false otherwise
	 */
	public boolean isMoving() {
		if (leftMotor.isMoving() || rightMotor.isMoving())
			return true;
		return false;
	}

	/**
	 * Method to set the speed for the left motor and moves forward (or backwards if negative)
	 * 
	 * @param speed Rotation Speed
	 */
	public void leftMotor(int speed) {
		leftMotor.setSpeed(speed);
		if (speed > 0)
			leftMotor.forward();
		else
			leftMotor.backward();
	}
	
	/**
	 * Method to set the speed for the right motor and moves forward (or backwards if negative)
	 * 
	 * @param speed Rotation Speed
	 */
	public void rightMotor(int speed) {
		rightMotor.setSpeed(speed);
		if (speed > 0)
			rightMotor.forward();
		else
			rightMotor.backward();
	}

	/**
	 * Method to set the speed for the left motor
	 * 
	 * @param speed Rotation Speed
	 */
	public void setLeftSpeed(int speed) {
		leftMotor.setSpeed(speed);
	}
	
	/**
	 * Method to set the speed for the right motor
	 * 
	 * @param speed Rotation Speed
	 */

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
	 * This method works exactly like the default rotation function for the left motor, except only
	 * the distance is needed as input.
	 * 
	 * @param distance Its value
	 * @param block True or false value
	 */
	public void leftRot(double distance, boolean block) {
		if (distance > 0) {
			leftMotor.rotate(convertDistance(radius, Math.abs(distance)), block);
		} else {
			leftMotor.rotate(-convertDistance(radius, Math.abs(distance)), block);
		}
	}

	/**
	 * This method works exactly like the default rotate function for the right motor except only
	 * the distance is needed as input.
	 * 
	 * @param distance Distance travel
	 * @param block True or false value
	 */
	public void rightRot(double distance, boolean block) {
		if (distance > 0) {
			rightMotor.rotate(convertDistance(radius, Math.abs(distance)), block);
		} else {
			rightMotor.rotate(-convertDistance(radius, Math.abs(distance)), block);
		}
	}

	/**
	 * This method takes in the rotation you want the robot to do and
	 * converts it into a distance for the wheels to rotate in opposite direction.
	 * 
	 * @param radius Wheel radius
	 * @param width track
	 * @param angle Rotation angle
	 * @return Value for wheel rotation
	 */
	private int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This method takes how far the robot is traveling
	 * and converts it to the necessary wheel rotation.
	 * 
	 * @param radius Wheel radius
	 * @param distance Travel distance
	 * @return Value for wheel rotation
	 */
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
	/**
	 * Method to change the radius of the wheels.
	 * 
	 * @param new_radius New Radius for different wheels
	 */
	public void setRadius(double new_radius) {
		this.radius = new_radius;
	}
	
	/**
	 * Method to change the track of the robot.
	 * 
	 * @param new_track New robot track is it changes configuration
	 */
	public void setTrack(double new_track) {
		this.track = new_track;
	}
}