package ca.mcgill.ecse211.main_package;

import java.util.HashMap;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.robotics.SampleProvider;

public class Search {
	
	private static Odometer odo;
	private static Navigation nav;
	private static myUSPoller fuck;
	
	public Search(Odometer odo, Navigation nav, myUSPoller fuck){
		this.odo = odo;
		this.nav = nav;
		this.fuck = fuck;
	}
	
/**
 * Param takes the maximum number of blocks in search area to keep 2D array size consistent.
 * Method: gets the distance and angle at every degree for a 90 degree turn
 * - iterates through the 90 angles to find a falling edge (first detected a block) then
 * rising edge (stopped detecting block)
 * - calculates the index in between falling and rising edge to get dist and angle of block
 * - calculates x and y IN CENTIMETERS
 * 
 * INCOMPLETE:- NEED TO CALCULATE TIME FOR BOT TO COMPLETE 90˚ TURN AT CERTAIN SPEED TO GET READINGS AT EVERY DEG
 * - DECIDE WHERE TO PUT MOTOR FUNCTIONALITY, I HAVE NO CLUE
 * @param maxBlocks
 * @return
 */
	public double[][] getBlocks(int maxBlocks){
		int i = 0;
		double distance[] = new double[90];
		double angle[] = new double[90];
		
		while(i < 90){ //get all distances and angles for 90 degrees, 1 degree intervals
			distance[i] = fuck.getDist();
			angle[i] = odo.getXYT()[2];
		}

		double blockazz[][] = new double[5][2];
		int blockctr = 0;
		boolean fallingEdge = false, risingEdge = false;
		double fallingDist = 0, risingDist = 0;
		double fallingAngle = 0, risingAngle = 0;
		int avgDist = 0, avgAngle = 0;
		double x,y;
		for(int j=1; j<90; j++){
			if(!fallingEdge){
				if(distance[j] < distance[j-1]){
					fallingDist = j;
					fallingAngle = j;
					fallingEdge = true;
				}
			}else if(!risingEdge){
				if(distance[j] > distance[j-1]){
					risingDist = j-1;
					risingAngle = j-1;
					risingEdge = true;
				}
			}else{
				//both falling and rising found
				avgDist = (int) (fallingDist + risingDist)/2;
				avgAngle = (int) (fallingAngle + risingAngle)/2;
				y = Math.sin(90-angle[avgAngle])*distance[avgDist];
				x = Math.cos(90-angle[avgAngle])*distance[avgDist];
				blockazz[blockctr][0] = x;
				blockazz[blockctr][1] = y;
				blockctr++;
				fallingEdge = false;
				risingEdge = false;
			}
		}
	return blockazz;
	}
}
