package ca.mcgill.ecse211.lab3;

import lejos.robotics.SampleProvider;

public class USPoller {

	// Class Variables
	private float[] sampleUS;
	SampleProvider myDistance;
	double wallDist;

	public USPoller(SampleProvider us, float[] usdata) {

		this.myDistance = us;
		this.sampleUS = usdata;
	}

	/** 
	 * this method returns true if the on object is detected within the threshold distance
	 * @param threshold
	 * @return boolean
	 */
	public boolean obstacleDetected(double threshold) {
		myDistance.fetchSample(sampleUS, 0); // Read latest sample in buffer
		wallDist = (int) (sampleUS[0] * 100.0); // Convert from MKS to CGS; truncate to int
		if (wallDist < threshold) {
			return true;
		} else {
			return false;
		}
	}
}
