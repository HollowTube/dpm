package ca.mcgill.ecse211.Localization;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Nav extends OdometerData{
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static double path_distance;
	public double Wheel_Radius;
	public double track;
	private static double xyt[] = new double[3];
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	Odometer odometer;
	public OdometerData odoData;
	
	//initiate the Navigation system and give the class the motors and odometer to use
	public Nav(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
						double Wheel_Radius, double track, Odometer odometer){
		this.Wheel_Radius=Wheel_Radius;
		this.track=track;
		this.odometer=odometer;
		this.leftMotor=leftMotor;
		this.rightMotor=rightMotor;
		//set motor acceleration
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
	    	motor.stop();
		    motor.setAcceleration(1000);
	    }
		//Set up the odometer to receive data
	    try{
	    	odoData = odometer.getOdometerData();   //synchronize with the odometer data
	    } catch (Exception e){
	    	System.out.println("odoData NOT WORKING");
	    }
	}
	
	void travelTo(double x, double y){
		xyt=odometer.getXYT();			//get current position
		path_distance= Math.pow(Math.pow(x-xyt[0],2)+Math.pow(y-xyt[1],2), 0.5); //calculate length of path
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(Wheel_Radius, path_distance), true);	//travel straight
		rightMotor.rotate(convertDistance(Wheel_Radius, path_distance), false);
	}
	void turnTo(double theta){
		//if theta > 180 || theta < -180
		//then turn 360-theta or 360-Math.abs(theta)
		if(theta >= 180){
			//turn negative direction (left)
			theta = (360-theta);
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(Wheel_Radius, track, theta), true);
			rightMotor.rotate(convertAngle(Wheel_Radius, track, theta), false);
		}else if(theta < -180){
			//turn positive direciton (right)
			theta = 360-Math.abs(theta);
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(Wheel_Radius, track, theta), true);
			rightMotor.rotate(-convertAngle(Wheel_Radius, track, theta), false);
		}else{
			if(theta < 0){
				//negative, turn left
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.rotate(-convertAngle(Wheel_Radius, track, Math.abs(theta)), true);
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
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	 }
	 private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	 }
}
