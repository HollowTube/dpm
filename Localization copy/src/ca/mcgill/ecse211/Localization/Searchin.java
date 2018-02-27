package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.Odometry.Odometer;
import ca.mcgill.ecse211.Odometry.OdometerData;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Searchin implements Runnable{

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

	public Searchin(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer, Navigation nav, EV3UltrasonicSensor usSensorLeft, EV3UltrasonicSensor usSensorFront, EV3ColorSensor colorSensorBlock){
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
	 * Avoid osbstacle by moving right, frorward, left, forward.
	 * 
	 */
	public void avoid(){
		nav.turnTo(90);
		leftMotor.rotate(convertDistance(2.15, 8));
		leftMotor.rotate(convertDistance(2.15, 8));
		nav.turnTo(-90);
		leftMotor.rotate(convertDistance(2.15, 8));
		leftMotor.rotate(convertDistance(2.15, 8));
		//turn right
		//move straight
		//turn left
		//move straight
		//move to waypoint
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
			float dist = getDistanceLeft();
			if(dist < 50 && dist > 8){
				//block has been detected
				leftMotor.stop(true);
				rightMotor.stop();
				Lab4.foundBlock = true;
				nav.turnTo(-90); //turn right on a dime
				try{
					Thread.sleep(1500);
				}catch (Exception e){
					
				}
				float frontDist = getDistanceFront();
				if(frontDist < 50 && dist > 8){
					leftMotor.forward();
					rightMotor.forward();
					while(true){
						colorSensorBlock.getRGBMode().fetchSample(sample, 0);
						if(sample[0] > 0.01){
							leftMotor.stop(true);
							rightMotor.stop();
							Lab4.noisemaker.systemSound(3);
							break;
						}
					}
				}else{ //WAS A FALSE POSITIVE -- TURN BACK TO CORRECT DIRECTION
					nav.turnTo(90); //turn right on a dime
					try{
						Thread.sleep(1500);
					}catch (Exception e){
						
					}
					leftMotor.forward();
					rightMotor.forward();
					try{
						Thread.sleep(1000);
					}catch (Exception e){
						
					}
					leftMotor.stop(true);
					rightMotor.stop();
					
					//Lab4.noisemaker.systemSound(0);
				}
			}
		}
	}
}
