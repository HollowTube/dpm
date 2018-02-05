package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class Pcontrol {

	// Class Constants
	public static final int SINTERVAL = 100; // A 10Hz sampling rate
	public static final double PROPCONST = 2.6; // Proportionality constant 2.4
	public static final int WALLDIST = 35;// Distance to wall * 1.4 (cm) accounting for sensor angle
	public static final int FWDSPEED = 170; // Forward speed (deg/sec)170
	public static final int MAXCORRECTION = 100; // Bound on correction to prevent stalling
	public static final long SLEEPINT = 100; // Display update 2Hz
	public static final int ERRORTOL = 2; // Error tolerance (cm)
	public static final int MAXDIST = 150; // Max value of valid distance
	public static final int FILTER_OUT = 12; // Filter threshold 17
	// Class Variables
	public static int wallDist = 0; // Measured distance to wall
	public static int distError = 0; // Error
	public static int leftSpeed = FWDSPEED; // Vehicle speed
	public static int rightSpeed = FWDSPEED;
	public static int filterControl = 0;
	public static int finalDist = 0;
	static RegulatedMotor leftMotor;
	static RegulatedMotor rightMotor;
	private static float[] sampleUS;
	static SampleProvider myDistance;

	
	public Pcontrol(SampleProvider us, float[] usdata, EV3LargeRegulatedMotor leftMotor,
		EV3LargeRegulatedMotor rightMotor) {
		Pcontrol.leftMotor = leftMotor;
		Pcontrol.rightMotor = rightMotor;
		Pcontrol.myDistance = us;
		Pcontrol.sampleUS = usdata;
	}
	

	public static boolean obstacleDetected() {
		myDistance.fetchSample(sampleUS, 0); // Read latest sample in buffer
		wallDist = (int) (sampleUS[0] * 100.0); // Convert from MKS to CGS; truncate to int
		if (wallDist < 40) {
			return true;
		} else {
			return false;
		}
	}

	public static void avoid() {
		int diff;
		boolean hasObstacle = true;

		if (wallDist < 40) {
			hasObstacle = true;
		}
		while (hasObstacle) {
			myDistance.fetchSample(sampleUS, 0); // Read latest sample in buffer
			wallDist = (int) (sampleUS[0] * 100.0); // Convert from MKS to CGS; truncate to int
			if (wallDist >= MAXDIST && filterControl < FILTER_OUT) {
				// bad value, do not set the distance var, however do increment the
				// filter value
				finalDist = WALLDIST;
				filterControl++;
			} else if (wallDist >= MAXDIST) {
				// We have repeated large values, so there must actually be nothing
				// there: leave the distance alone
				finalDist = MAXDIST;
			} else {
				// distance went below 255: reset filter and leave
				// distance alone.
				hasObstacle = false;
				break;

			}
			if (wallDist >= MAXDIST) {
				wallDist = MAXDIST;
			}
			distError = WALLDIST - finalDist; // Compute error term

			// Controller Actions

			if (Math.abs(distError) <= ERRORTOL) { // Case 1: Error in bounds, no correction
				leftSpeed = FWDSPEED;
				rightSpeed = FWDSPEED;
				leftMotor.setSpeed(leftSpeed); // If correction was being applied on last
				rightMotor.setSpeed(rightSpeed); // update, clear it
			}

			else if (distError > 0) { // Case 2: positive error, move away from wall
				diff = calcProp(distError); // Get correction value and apply
				leftSpeed = FWDSPEED + diff;
				rightSpeed = FWDSPEED - diff;
				leftMotor.setSpeed(leftSpeed);
				rightMotor.setSpeed(rightSpeed);
			}

			else if (distError < 0) { // Case 3: negative error, move towards wall
				diff = calcProp(distError); // Get correction value and apply
				leftSpeed = FWDSPEED - diff;
				rightSpeed = FWDSPEED + diff;
				leftMotor.setSpeed(leftSpeed);
				rightMotor.setSpeed(rightSpeed);
			}
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	//
	// This method is used to implement your particular control law. The default
	// here is to alter motor speed by an amount proportional to the error. There
	// is some clamping to stay within speed limits.

	static int calcProp(int diff) {

		int correction;

		// PROPORTIONAL: Correction is proportional to magnitude of error

		diff = Math.abs(diff);

		correction = (int) (PROPCONST * (double) diff);
		if (correction >= FWDSPEED) {
			correction = MAXCORRECTION;
		}

		return correction;
	}

}
