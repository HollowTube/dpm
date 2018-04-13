package ca.mcgill.ecse211.main_package;

import lejos.robotics.SampleProvider;


/**
 * This class handles the data received by the Ultrasonic Sensor for detecting objects or obstacles.
 * 
 * @author Tritin
 *
 */
public class myUSPoller {

	// Class Variables
	private float[] sampleUS;
	SampleProvider myDistance;
	double wallDist;


	/**
	 * Class Constructor.
	 * 
	 * @param us Sample from Ultrasonic Sensor
	 * @param usdata Value of sample
	 */
	public myUSPoller(SampleProvider us, float[] usdata) {
		this.myDistance = us;
		this.sampleUS = usdata;
	}

	/**
	 * Method to retrieve a reading from the Ultrasonic Sensor filtering out distances greater than 250cm.
	 * @return Ultrasonic Distance reading in cm
	 */
	public synchronized double getDist() {
		myDistance.fetchSample(sampleUS, 0); // Read latest sample in buffer
		wallDist = (int) (sampleUS[0] * 100.0); // Convert from MKS to CGS; truncate to int

		if (wallDist > 250) {
			return 250;
		}
		return wallDist;
	}

	/**
	 * this method returns true if an object is detected within the threshold
	 * distance. 
	 * 
	 * @param threshold Distance threshold for object detection 
	 * @return boolean True if object detected, false otherwise
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
