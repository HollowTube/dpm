package ca.mcgill.ecse211.lab4;

import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;

public class myUSPoller {

	// Class Variables
	
	private final int BUFFER_SIZE = 5;
	private float[] sampleUS;
	SampleProvider myDistance;
	double wallDist;
	//MedianFilter myfilter = new MedianFilter(myDistance, BUFFER_SIZE);

	public  myUSPoller(SampleProvider us, float[] usdata) {
		this.myDistance = us;	
		this.sampleUS = usdata;
	}
	public double getDist() {
		myDistance.fetchSample(sampleUS, 0); // Read latest sample in buffer
		wallDist = (int) (sampleUS[0] * 100.0); // Convert from MKS to CGS; truncate to int
		
		if(wallDist > 250) {
			return 250;
		}
		return wallDist;
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
//	public double getMedian(int filtersize) {
//		double dist = 0;
//		myfilter.fetchSample(sampleUS, 0);
//		dist = sampleUS[0];
//		return dist;
//	}
}
