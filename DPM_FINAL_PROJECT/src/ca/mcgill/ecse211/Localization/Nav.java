package ca.mcgill.ecse211.Localization;

import ca.mcgill.ecse211.main_package.*;
import ca.mcgill.ecse211.odometer.*;

public class Nav extends OdometerData {
	private static final int FORWARD_SPEED = 250;
	private static double path_distance;
	public double Wheel_Radius;
	public double track;
	private double TILE_SIZE =30.48;
	private static double xyt[] = new double[3];
	private MotorControl motorcontrol; 
	Odometer odometer;
	public OdometerData odoData;

	/* initiate the Navigation system and give the class the motors and odometer to
	 * use
	 * */
	public Nav(MotorControl motorcontrol, double Wheel_Radius, double track,Odometer odometer) {
		this.Wheel_Radius = Wheel_Radius;
		this.track = track;
		this.odometer = odometer;
		this.motorcontrol = motorcontrol;
		/* set motor acceleration*/
		motorcontrol.setAcceleration(1000);
		/* Set up the odometer to receive data*/
		try {
			odoData = odometer.getOdometerData(); // synchronize with the odometer data
		} catch (Exception e) {
			System.out.println("odoData NOT WORKING");
		}
	}

	void travelTo(double x, double y) {
		xyt = odometer.getXYT(); // get current position
		path_distance = Math.pow(Math.pow(x - xyt[0], 2) + Math.pow(y - xyt[1], 2), 0.5); // calculate length of path
		motorcontrol.setLeftSpeed(FORWARD_SPEED);
		motorcontrol.setRightSpeed(FORWARD_SPEED);
		motorcontrol.forward(path_distance);   //move forward by the distance calculated
	}

	void turnToCoord(double x, double y){
		xyt = odometer.getXYT();
		if(x >= 0 && y >= 0){										//angles are from the y-axis instead of x-axis
			turnTo((180/Math.PI)*Math.atan2(x-(Math.round(xyt[0]/TILE_SIZE)), y-(Math.round(xyt[1]/TILE_SIZE)))-xyt[2]);  		//angles in positive xy-plane
		}else if(y < 0){							
			turnTo(180+(180/Math.PI)*Math.atan2(x-(Math.round(xyt[0]/TILE_SIZE)), y-(Math.round(xyt[1]/TILE_SIZE)))-xyt[2]);	//angles in the negative y-plane
		}else{
			turnTo(360+(180/Math.PI)*Math.atan2(x-(Math.round(xyt[0]/TILE_SIZE)), y-(Math.round(xyt[1]/TILE_SIZE)))-xyt[2]); 	//angles in negative x-plane and positive y-plane
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
		xyt = odometer.getXYT();
		double currentX = xyt[0];
		double currentY = xyt[1];
		double path_angle = (180/Math.PI)*Math.atan2(x-(Math.round(currentX/TILE_SIZE)), y-(Math.round(currentY/TILE_SIZE)));
		turnTo(path_angle-xyt[2]);
	}
	void turnTo(double theta) {
		motorcontrol.turnto(theta);   //turn by the angle input 
	}

	boolean isNavigating() {
		return motorcontrol.isMoving();   //verify if motor is moving
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
