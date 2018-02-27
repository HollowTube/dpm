package ca.mcgill.ecse211.Searching;

import java.util.Arrays;

import ca.mcgill.ecse211.Odometry.Odometer;
import ca.mcgill.ecse211.Odometry.OdometerData;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Searching implements Runnable{

	private static double xyt[] = new double[3];
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	Odometer odometer;
	public OdometerData odoData;
	private Navigation nav;
	private static EV3ColorSensor colorSensorBlock;
	private static EV3UltrasonicSensor usSensorLeft;
	private static EV3UltrasonicSensor usSensorFront;
	private SampleProvider sampleProvLeft;
	private SampleProvider sampleProvFront;
	public float[] usData = new float[100];
	public float[] sample = new float[3];
	private static float frontDist;
	private static float dist;
	private static float[] blockData = new float[5];
	private static float[] DataSet = new float[5];
	private static int DetectCount = 0;
	private static float median;
	
	public Searching(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer, Navigation nav, EV3UltrasonicSensor usSensorLeft, EV3UltrasonicSensor usSensorFront, EV3ColorSensor colorSensorBlock){
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.nav = nav;
		this.usSensorLeft = usSensorLeft;
		this.usSensorFront = usSensorFront;
		this.sampleProvFront = usSensorFront.getDistanceMode();
		this.sampleProvLeft = usSensorLeft.getDistanceMode();
		this.colorSensorBlock = colorSensorBlock;
	}

	/**
	 * Old method from squaredriver to convert distance.
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public float getDistanceLeft(){
		sampleProvLeft.fetchSample(usData, 0);

		if(usData[0]*100 > 255){
			return 255;
		}
		return usData[0]*100;
	}
	public float getDistanceFront(){
		sampleProvFront.fetchSample(usData, 0);

		if(usData[0]*100 > 255){
			return 255;
		}
		return usData[0]*100;
	}

	public void run(){
		while(true){
			dist = getDistanceLeft();
			blockData[DetectCount%5]=dist;
			DetectCount++;
			for(int i=0; i<5;i++){
				DataSet[i] = blockData[i];
			}
			Arrays.sort(DataSet);
			median= DataSet[2];
			if(median < 50 && median > 8){ //if the left US reads a block or false positive
				
				Lab5.foundBlock = true;
				Lab5.noisemaker.systemSound(1);
				//block has been detected
				leftMotor.stop(true);
				rightMotor.stop();
				nav.turnTo(-90); //turn right on a dime to get front US to face the block
				try{
					Thread.sleep(2000); //wait for turn to complete
				}catch (Exception e){}
				
				frontDist = getDistanceFront();
				if(frontDist < 50 && frontDist > 8){//check if front US sensor also reads a block
					xyt = odometer.getXYT();
					leftMotor.forward();
					rightMotor.forward();
					while(true){ //as the bot is approaching the block, repeatedly poll the light sensor
						//-----------------COLOR DETECTION-----------------------------
						colorSensorBlock.getRGBMode().fetchSample(sample, 0);
						if(sample[0] > 0.01){ //UNFINISHED COLOR DETECTION
							leftMotor.stop(true);
							rightMotor.stop();
							Lab5.noisemaker.systemSound(3);
							break;
						}
						//---------------------------------------------------------------
					}
					nav.turnToPoint(xyt[0], xyt[1]);
					try{
						Thread.sleep(3000);
					}catch (Exception e){}
					nav.travelTo(xyt[0], xyt[1]);
					try{
						Thread.sleep(3000);
					}catch (Exception e){}
					Lab5.wpCtr--;
					Lab5.foundBlock = false;
				}else{ //WAS A FALSE POSITIVE
					median = 0;
					Lab5.wpCtr--;
					Lab5.foundBlock = false;
				}
				//decrement Waypoints waypoint counter to hopefully get it to turn to
									//and head to the proper waypoint
			}
		}
	}
}
