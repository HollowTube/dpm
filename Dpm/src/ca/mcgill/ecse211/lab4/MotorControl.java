package ca.mcgill.ecse211.lab4;

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
	private double radius;
	private double track;

	public MotorControl(EV3LargeRegulatedMotor leftmotor, EV3LargeRegulatedMotor rightmotor, double wheelRad,
			double track) {
		MotorControl.leftMotor = leftmotor;
		MotorControl.rightMotor = rightmotor;
		this.radius = wheelRad;
		this.track = track;
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
	 * This method stops the left and right motors almost simultaneously
	 */
	public void stop() {
		leftMotor.stop(false);
		rightMotor.stop();
	}
	/**
	 * This method makes the robot turn on a dime for a certain amount of degrees, positive rotation means clockwise
	 * ,negative rotation means counter-clockwise
	 * The block parameter determines if the program should wait for the movement 
	 * to finish. Set true if the program should wait for the movement to end
	 * 
	 * @param rotation
	 * @param rotate_speed
	 * @param block
	 */
	public void dime_turn(double rotation, int rotate_speed, boolean block) {
		leftMotor.setSpeed(rotate_speed);
		rightMotor.setSpeed(rotate_speed);
		
		if (rotation > 0) {
			leftMotor.rotate(convertAngle(radius, track, rotation), true);
			rightMotor.rotate(-convertAngle(radius, track, rotation), !block);
		} else {
			leftMotor.rotate(-convertAngle(radius, track, rotation), true);
			rightMotor.rotate(convertAngle(radius, track, rotation), !block);
		}
	}
	public boolean isMoving() {
		if(leftMotor.isMoving() || rightMotor.isMoving()) return true;
		return false;
	}

	private int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

}
