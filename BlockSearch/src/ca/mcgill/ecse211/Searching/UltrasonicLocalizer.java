package ca.mcgill.ecse211.Searching;

import ca.mcgill.ecse211.Odometry.Odometer;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {
	private static double wall_distance = 40;
	private static double noise_margin = 2;
	private final Odometer odo;
	private final Navigation nav;
	private final EV3UltrasonicSensor us;
	private SampleProvider sampleProv;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;
	//private final SampleProvider sampleProv;
	private final int type;
	public float[] usData = new float[100];
	public double[] xyt = new double[3];
	public double[] edgeDetect = new double[2];
	public double[] wallSpots = new double[2];
	private int dTheta;
	private double lowestDist = 1000;
	private double temp = 0;
	private double oldAvg = 0;
	private double numReads = 0;
	
	public UltrasonicLocalizer(Odometer odo, Navigation nav, EV3UltrasonicSensor us, int type, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		this.odo=odo;
		this.nav=nav;
		this.us= us;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sampleProv = us.getDistanceMode();
		//this.sampleProv = sampleProv;
		this.type=type;
		//this.us.enable();
	}
	public void Localize(){
		if(this.type==1){
			fallingEdge();
		}else if(this.type==2){
			risingEdge();
		}
		//if back wall angle < left wall angle
		if(edgeDetect[1] < edgeDetect[0]){
			double correctionAngle = 45-((edgeDetect[0]+edgeDetect[1])%360)/2;
			odo.setTheta(edgeDetect[1]+correctionAngle);
			//odo.setTheta(edgeDetect[1]+correctionAngle);
			try{
				Thread.sleep(500);
			} catch (Exception e){
				SearchLab.noisemaker.systemSound(4);
			}
			nav.turnTo(-odo.getXYT()[2]);
		}
		else{
			double correctionAngle = 45-((edgeDetect[0]+edgeDetect[1])%360)/2;
			odo.setTheta(edgeDetect[1]+correctionAngle);
			try{
				Thread.sleep(500);
			} catch (Exception e){
				SearchLab.noisemaker.systemSound(4);
			}
			//odo.setTheta(odo.getXYT()[2]+correctionAngle);
			nav.turnTo(-odo.getXYT()[2]);
			//nav.turnTo(180);
			//odo.setTheta(0);
		}
		if(getDistance() < 40) {//if angles were passing over 360˚, use wall to correct theta and orientation
			odo.setTheta(180);
			nav.turnTo(180);
		}
		
		
		SearchLab.noisemaker.systemSound(3);
	}
	public void fallingEdge(){
		/*
		 * FALLING EDGE LOGIC:
		 * - do 360 spin clockwise to find the closest distance reading to set the wall_distance
		 * - continue spinning clockwise until sensor reads over threshold of 255 (not facing a wall)
		 * - slow down and begin spinning counter-clockwise to find first falling edge
		 * - read and beep at what point the sensor reads a value entering the wall_distance + noise
		 * - reverse and beep at what point the sensor reads a value entering the wall_distance + noise
		 * - exit fallingEdge method and determine correction angle
		 * */
		leftMotor.setSpeed(125);
		rightMotor.setSpeed(125);
		leftMotor.forward();
		rightMotor.backward();
		System.out.println("");
		System.out.println("");
		System.out.println("");
		while(getDistance() < 150){} //spin until pointing away

		leftMotor.setSpeed(70); //begin slowly turning left to find a wall
		rightMotor.setSpeed(70);
		leftMotor.backward();
		rightMotor.forward();
		
		while(getDistance() >= (wall_distance + noise_margin)){} //let spin until distance enters margin
		
		leftMotor.stop(true);
		rightMotor.stop();
		
		edgeDetect[0] = odo.getXYT()[2]; //entry point angle
		//System.out.println("wall1 = " + edgeDetect[0]);
		//Lab4.noisemaker.systemSound(1); //found left wall
		
		leftMotor.forward(); //reverse to find next falling edge
		rightMotor.backward();
		
		try{
			Thread.sleep(2000);
		}catch(Exception e){
			SearchLab.noisemaker.systemSound(4); //sleep didnt work
		}
		
		while(getDistance() >= (wall_distance + noise_margin)){} //let spin until distance enters margin
		edgeDetect[1] = odo.getXYT()[2]; //entry point angle
		//Lab4.noisemaker.systemSound(1); //found second wall
		//System.out.println("wall2 = " + edgeDetect[1]);
		leftMotor.stop(true);
		rightMotor.stop();
	}
	public void risingEdge(){
		/*
		 * RISING EDGE LOGIC:
		 * - do 360 spin clockwise to find the closest distance reading to set the wall_distance
		 * - continue spinning clockwise until sensor reads over threshold of 255 (not facing a wall)
		 * - slow down and begin spinning counter-clockwise to find rising falling edge
		 * - read and beep at what point the sensor reads a value exiting the wall_distance - noise
		 * - continue same spin and beep at what point the sensor reads a value entering the wall_distance - noise
		 * - exit fallingEdge method and determine correction angle
		 * */
		leftMotor.setSpeed(125);
		rightMotor.setSpeed(125);
		leftMotor.forward();
		rightMotor.backward();
		
		while(getDistance() < 150){} //spin until pointing towards

		leftMotor.setSpeed(70);
		rightMotor.setSpeed(70);
		leftMotor.backward();
		rightMotor.forward();
		
		while(getDistance() >= (wall_distance - noise_margin)){} //let spin until distance enters margin
		
		leftMotor.stop(true);
		rightMotor.stop();
		
		edgeDetect[0] = odo.getXYT()[2]; //exit point angle
		
		leftMotor.backward(); //reverse to find next falling edge
		rightMotor.forward();
		
		try{
			Thread.sleep(2000);
		}catch(Exception e){
			SearchLab.noisemaker.systemSound(4); //sleep didnt work
		}
		
		while(getDistance() <= (wall_distance - noise_margin)){} //let spin until distance enters margin
		edgeDetect[1] = odo.getXYT()[2]; //entry point angle

		leftMotor.stop(true);
		rightMotor.stop();
	}
	public float getDistance(){
		sampleProv.fetchSample(usData, 0);

		if(usData[0]*100 > 255){
			return 255;
		}
		return usData[0]*100;
	}
}
