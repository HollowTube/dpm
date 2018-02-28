package ca.mcgill.ecse211.Searching;

import ca.mcgill.ecse211.Odometry.Odometer;
import ca.mcgill.ecse211.Odometry.OdometerData;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Navigation extends OdometerData implements UltrasonicController{
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 90;
	private static double path_distance;
	public double Wheel_Radius;
	public double track;
	private static double xyt[] = new double[3];
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor spinMotor;
	Odometer odometer;
	public OdometerData odoData;
	
	//initiate the Navigation system and give the class the motors and odometer to use
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
						double Wheel_Radius, double track, Odometer odometer){
		this.Wheel_Radius=Wheel_Radius;
		this.track=track;
		this.odometer=odometer;
		this.leftMotor=leftMotor;
		this.rightMotor=rightMotor;
		this.spinMotor = spinMotor;
		//set motor acceleration
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
	    	motor.stop();
		    //motor.setAcceleration(1000);
	    }
		//Set up the odometer to receive data
	    try{
	    	odoData = odometer.getOdometerData();   //synchronize with the odometer data
	    } catch (Exception e){
	    	System.out.println("odoData NOT WORKING");
	    }
	}
	
	/**
	 * 
	 * Travels to the specified coordinates, parameter are in cm
	 * @param x
	 * @param y
	 */
	void travelTo(double x, double y){
		xyt = SearchLab.odometer.getXYT();			//get current position
		path_distance= Math.pow(Math.pow(x-xyt[0],2)+Math.pow(y-xyt[1],2), 0.5); //calculate length of path
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(Wheel_Radius, path_distance), true);	//travel straight
		rightMotor.rotate(convertDistance(Wheel_Radius, path_distance), false);
		leftMotor.stop(true);
		rightMotor.stop();
	}
	
	/**
	 * Turns the robot to the specificed coordinates, provided in cm
	 * 
	 * @param x
	 * @param y
	 */
	void turnToCoord(double x, double y){
		xyt = SearchLab.odometer.getXYT();
		if(x >= 0 && y >= 0){										//angles are from the y-axis instead of x-axis
			turnTo((180/Math.PI)*Math.atan2(x-(Math.round(xyt[0]/SearchLab.TILE_SIZE)), y-(Math.round(xyt[1]/SearchLab.TILE_SIZE)))-xyt[2]);  		//angles in positive xy-plane
		}else if(y < 0){							
			turnTo(180+(180/Math.PI)*Math.atan2(x-(Math.round(xyt[0]/SearchLab.TILE_SIZE)), y-(Math.round(xyt[1]/SearchLab.TILE_SIZE)))-xyt[2]);	//angles in the negative y-plane
		}else{
			turnTo(360+(180/Math.PI)*Math.atan2(x-(Math.round(xyt[0]/SearchLab.TILE_SIZE)), y-(Math.round(xyt[1]/SearchLab.TILE_SIZE)))-xyt[2]); 	//angles in negative x-plane and positive y-plane
		}
		//turnTo((180/Math.PI)*Math.atan2(x-(Math.round(xyt[0]/Lab4.TILE_SIZE)), y-(Math.round(xyt[1]/Lab4.TILE_SIZE))));
	}
	
	/**
	 * Turn to the specific coordinates provided by x, y relative to the robot's current position
	 * x, y should be in cm, not coordinates
	 * @param x
	 * @param y
	 */
	void turnToPoint(double x, double y){
		xyt = SearchLab.odometer.getXYT();
		double currentX = xyt[0];
		double currentY = xyt[1];
		double path_angle = (180/Math.PI)*Math.atan2(x-(Math.round(currentX/SearchLab.TILE_SIZE)), y-(Math.round(currentY/SearchLab.TILE_SIZE)));
		turnTo(path_angle-xyt[2]);
	}
	
	/**
	 * turnTo(double theta)
	 * 
	 * Turns the robot to the desired heading from the 0˚ forward position of the robot.
	 * 
	 * @param theta
	 */
	void turnTo(double theta){
		//if theta > 180 || theta < -180
		//then turn 360-theta or 360-Math.abs(theta)
		/*
		 * With the following logic the robot should always make the optimal turn. Optimal
		 * as in turning the least degrees necessary.
		 * */
		if(theta >= 180){
			//turn negative direction (left)
			theta = (360-theta); //since it should turn more than 180, turn the smalled direction, i.e 360-theta
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(Wheel_Radius, track, theta), true);
			rightMotor.rotate(convertAngle(Wheel_Radius, track, theta), false);
		}else if(theta < -180){
			//turn positive direciton (right)
			theta = 360-Math.abs(theta); //same logic as previous if statement except in opposite case
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(Wheel_Radius, track, theta), true);
			rightMotor.rotate(-convertAngle(Wheel_Radius, track, theta), false);
		}else{
			if(theta < 0){
				//negative, turn left
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(Wheel_Radius, track, Math.abs(theta)), true); //used Math.abs to not turn in opposite diretion
				rightMotor.rotate(convertAngle(Wheel_Radius, track, Math.abs(theta)), false);
			}else{
				//positive, turn right
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(convertAngle(Wheel_Radius, track, theta), true);
				rightMotor.rotate(-convertAngle(Wheel_Radius, track, theta), false);
			}
		}
	}
	boolean isNavigating(){
		return leftMotor.isMoving();
	}
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	 }
	 public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	 }
	 
	@Override
	public void processUSData(int distance) {
		//Old unnecessary method, left in for potential tests
	}

	@Override
	public int readUSDistance() {
		//Old unnecessary method, left in for potential tests
		return 0;
	}
}