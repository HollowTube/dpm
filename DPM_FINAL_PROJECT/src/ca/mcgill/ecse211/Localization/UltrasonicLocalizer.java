package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.odometer.*;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.main_package.*;

public class UltrasonicLocalizer {
	private static double wall_distance = 40;
	private static double noise_margin = 2;
	private final Odometer odo;
	private MotorControl motorcontrol;
	private SampleProvider sampleProv;
	private final int type;
	public float[] usData = new float[100];
	public double[] xyt = new double[3];
	public double[] edgeDetect = new double[2];
	public double[] wallSpots = new double[2];
	
	/**
	 * Class contructor
	 * @author Matthew Caminiti and Alexandre Coulombe
	 * @param odo
	 * @param us
	 * @param type
	 * @param motorcontrol
	 */
	public UltrasonicLocalizer(Odometer odo, SampleProvider us, int type, MotorControl motorcontrol){
		this.odo=odo;
		this.sampleProv = us;
		this.type=type;
		this.motorcontrol = motorcontrol;
	}
	/**
	 * @author Matthew Caminiti and Alexandre Coulombe
	 * Method to use the ultrasonic sensor to make the robot face 0 degrees.
	 * Method calls either fallingEdge or risingEdge to get two values for the array of doubles edgeDetect.
	 * The values are then used to correct the odometer angle to its actual angle and turn the robot to face 0 degrees.
	 * Back up conditional to turn 180 degrees if the robot corrects itself in the wrong direction and faces the wall.
	 */
	public void Localize(){
		if(this.type==1){
			fallingEdge();  //performs falling edge ultrasonic localization
		}else if(this.type==2){
			risingEdge();  //performs rising edge ultrasonic localization
		}
		//if back wall angle < left wall angle
		if(edgeDetect[1] < edgeDetect[0]){
			double correctionAngle = 45-((edgeDetect[0]+edgeDetect[1])%360)/2;
			odo.setTheta(edgeDetect[1]+correctionAngle);
			try{
				Thread.sleep(500);
			} catch (Exception e){}
			motorcontrol.turnto(-odo.getXYT()[2]);
		}
		else{
			double correctionAngle = 45-((edgeDetect[0]+edgeDetect[1])%360)/2;
			odo.setTheta(edgeDetect[1]+correctionAngle);
			try{
				Thread.sleep(500);
			} catch (Exception e){}
			motorcontrol.turnto(-odo.getXYT()[2]);
		}
		if(getDistance() < 40) {//if angles were passing over 360, use wall to correct theta and orientation
			odo.setTheta(180);
			motorcontrol.turnto(180);
		}
	}
	
	/**
	 * @author Matthew Caminiti
	 * Method to get values that go into a margin of wall distances.
	 *  FALLING EDGE LOGIC:
	 *   - do 360 spin clockwise to find the closest distance reading to set the wall_distance
	 *   - continue spinning clockwise until sensor reads over threshold of 255 (not facing a wall)
	 *   - slow down and begin spinning counter-clockwise to find first falling edge
	 *   - read and beep at what point the sensor reads a value entering the wall_distance + noise
	 *   - reverse and beep at what point the sensor reads a value entering the wall_distance + noise
	 *   - exit fallingEdge method and determine correction angle
	 */
	public void fallingEdge(){
		motorcontrol.setLeftSpeed(175);
		motorcontrol.setRightSpeed(175);
		motorcontrol.leftforward();
		motorcontrol.rightbackward();
		while(getDistance() < 150){} //spin until pointing away

		motorcontrol.setLeftSpeed(150); //begin slowly turning left to find a wall
		motorcontrol.setRightSpeed(150);
		motorcontrol.leftbackward();
		motorcontrol.rightforward();
		
		while(getDistance() >= (wall_distance + noise_margin)){} //let spin until distance enters margin
		
		motorcontrol.stop();
		
		edgeDetect[0] = odo.getXYT()[2]; //entry point angle
		
		motorcontrol.leftforward(); //reverse to find next falling edge
		motorcontrol.rightbackward();
		
		try{
			Thread.sleep(1000);
		}catch(Exception e){}
		
		while(getDistance() >= (wall_distance + noise_margin)){} //let spin until distance enters margin, stop when wall is found
		edgeDetect[1] = odo.getXYT()[2]; //entry point angle
		motorcontrol.stop();
	}
	
	/**
	 * @author Matthew Caminiti
	 * Method to get values that go into a margin of wall distances.
	 * RISING EDGE LOGIC:
	 * - do 360 spin clockwise to find the closest distance reading to set the wall_distance
	 * - continue spinning clockwise until sensor reads over threshold of 255 (not facing a wall)
	 * - slow down and begin spinning counter-clockwise to find rising falling edge
	 * - read and beep at what point the sensor reads a value exiting the wall_distance - noise
	 * - continue same spin and beep at what point the sensor reads a value entering the wall_distance - noise
	 * - exit fallingEdge method and determine correction angle
	 */
	public void risingEdge(){
		motorcontrol.setLeftSpeed(175);
		motorcontrol.setRightSpeed(175);
		motorcontrol.leftforward();
		motorcontrol.rightbackward();
		
		while(getDistance() < 150){} //spin until pointing towards
		motorcontrol.setLeftSpeed(150);
		motorcontrol.setRightSpeed(150);
		motorcontrol.leftbackward();
		motorcontrol.rightforward();
		
		while(getDistance() >= (wall_distance - noise_margin)){} //let spin until distance enters margin
		motorcontrol.stop();
		edgeDetect[0] = odo.getXYT()[2]; //exit point angle
		motorcontrol.leftbackward(); //reverse to find next falling edge
		motorcontrol.rightforward();
		try{
			Thread.sleep(1000);
		}catch(Exception e){}
		
		while(getDistance() <= (wall_distance - noise_margin)){} //let spin until distance enters margin
		edgeDetect[1] = odo.getXYT()[2]; //entry point angle
		motorcontrol.stop();
	}
	
	/**
	 * Method to get the distance between the ultrasonic sensor and the object in front of it
	 * @return distance from object
	 */
	public float getDistance(){
		sampleProv.fetchSample(usData, 0);

		if(usData[0]*100 > 255){
			return 255;
		}
		return usData[0]*100;
	}
}