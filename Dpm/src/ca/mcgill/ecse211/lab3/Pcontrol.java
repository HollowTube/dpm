package ca.mcgill.ecse211.lab3;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class Pcontrol {

	// Class Constants
	public final int SINTERVAL = 100; // A 10Hz sampling rate
	public final double PROPCONST = 2.6; // Proportionality constant 2.4
	public final int WALLDIST = 35;// Distance to wall * 1.4 (cm) accounting for sensor angle
	public final int FWDSPEED = 100; // Forward speed (deg/sec)170
	public final int MAXCORRECTION = 100; // Bound on correction to prevent stalling
	public final long SLEEPINT = 100; // Display update 2Hz
	public final int ERRORTOL = 2; // Error tolerance (cm)
	public final int MAXDIST = 150; // Max value of valid distance
	public final int FILTER_OUT = 12; // Filter threshold 17
	// Class Variables
	public static int wallDist = 0; // Measured distance to wall
	public int distError = 0; // Error
	public int leftSpeed = FWDSPEED; // Vehicle speed
	public int rightSpeed = FWDSPEED;
	public int filterControl = 0;
	public int finalDist = 0;
	RegulatedMotor leftMotor;
	RegulatedMotor rightMotor;
	private float[] sampleUS;
	SampleProvider myDistance;

	public Pcontrol(SampleProvider us, float[] usdata, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.myDistance = us;
		this.sampleUS = usdata;
	}

	public boolean obstacleDetected() {
		myDistance.fetchSample(sampleUS, 0); // Read latest sample in buffer
		wallDist = (int) (sampleUS[0] * 100.0); // Convert from MKS to CGS; truncate to int
		if (wallDist < 40) {
			return true;
		} else {
			return false;
		}
	}

	public int avoid() {

		myDistance.fetchSample(sampleUS, 0); // Read latest sample in buffer
		wallDist = (int) (sampleUS[0] * 100.0); // Convert from MKS to CGS; truncate to int

		return wallDist;

	}

	//
	// This method is used to implement your particular control law. The default
	// here is to alter motor speed by an amount proportional to the error. There
	// is some clamping to stay within speed limits.

	int calcProp(int diff) {

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
