package ca.mcgill.ecse211.Searching;

import ca.mcgill.ecse211.Odometry.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

public class LightLocalizer {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private int ROTATE_SPEED=100;
	private static int LightDiff=100;
	private static double Light_Sensor_Position = 7.8;
	private Odometer odo;
	private Navigation nav;
	private EV3ColorSensor LSensor;
	private float[] LSData;
	private float lastLSData;
	private double[] angles;
	private double[] xyt;
	private int lineCount=0;
	public float[] sample = new float[3];
	
	public LightLocalizer(Odometer odo, Navigation nav, 
							EV3ColorSensor LSensor, EV3LargeRegulatedMotor leftMotor, 
							EV3LargeRegulatedMotor rightMotor){
		this.odo=odo;     //initiate the LightLocalizer
		this.nav=nav;
		this.LSensor=LSensor;
		this.leftMotor=leftMotor;
		this.rightMotor=rightMotor;
		LSensor.getRGBMode();
	}
	public void Localize(){
		//move forward until y = 0 line
		leftMotor.forward();
		rightMotor.forward();
		while(!lineDetected()){}
		leftMotor.stop(true); //stop when y = 0 detected
		rightMotor.stop();
		odo.setY(-Light_Sensor_Position); // set position to offset of light sensor (y)
		nav.turnTo(90); //turn 90˚ to face x = 0 line
		leftMotor.forward(); //move forward until x = 0 line
		rightMotor.forward();
		while(!lineDetected()){} //stop when x = 0 line detected
		leftMotor.stop(true);
		rightMotor.stop();
		odo.setX(-Light_Sensor_Position); // set position to offset of light sensor (x)
		nav.turnTo(315);
		leftMotor.rotate(nav.convertDistance(SearchLab.WHEEL_RAD, 12.8), true);	//travel to (0,0) coordinate
		rightMotor.rotate(nav.convertDistance(SearchLab.WHEEL_RAD, 12.8), false);
		//nav.turnTo(315);
		leftMotor.stop(true);
		rightMotor.stop();
		leftMotor.setSpeed(35);
		rightMotor.setSpeed(35);
		leftMotor.backward(); //spin to find x = 0 line to correct theta
		rightMotor.forward();
		while(!lineDetected()){} //stop when x = 0 line detected
		leftMotor.stop(true);
		rightMotor.stop();
		odo.setTheta(0); //set theta to 0 degrees
	}
	public boolean lineDetected(){
	    LSensor.getRGBMode().fetchSample(sample, 0); //get sample data from light sensor
	    if(sample[0] <= 0.08 && sample[1] <= 0.08 && sample[2] <= 0.08){ //check if the RGB value corresponds to black
	    	return true;   //if color black detected, return true
	    }else return false; //if no color black detected, return false
	}
}
