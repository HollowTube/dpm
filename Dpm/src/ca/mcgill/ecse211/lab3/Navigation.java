package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.robotics.RegulatedMotor;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;

public class Navigation {
	private static Odometer odometer;
	private static double position[];
	private static double newheading;
	private static final int FORWARD_SPEED = 100;
	private static final int RANGE_THRESHOLD = 2;
	private static final int HEADING_THRESHOLD = 2;
	 private static final double TILE_SIZE = 30.48;
	  static RegulatedMotor leftMotor;
	  static RegulatedMotor rightMotor; 
	  static double leftRadius;
	  static double rightRadius;
	  static double track;
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double track) throws OdometerExceptions {
	    this.odometer = Odometer.getOdometer();
	    Navigation.leftMotor = leftMotor;
	    Navigation.rightMotor = rightMotor;
	    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.setAcceleration(1000);
		    }
	    Navigation.leftRadius = leftRadius;
	    this.rightRadius = rightRadius;
	    this.track = track;
	  }
	  /**
	   * This method makes the robot travel to a certain coordinate, 
	   * it will also hand control to another method if an obstacle is ever detected.
	   * It depends on the odometer data to get the correct heading
	   * 
	   * @param xf
	   * @param yf

	 * @throws OdometerExceptions 
	   */
	  public static boolean travelTo(double xf, double yf) throws OdometerExceptions {

			double xi,yi,dx,dy,initial_heading,final_heading, turning_angle;

			 do {
					
				 position = odometer.getXYT();
				 xi = position[0];
				 yi = position[1];
				 initial_heading = position[2];
				 dx = xf * TILE_SIZE - xi;
				 dy = yf * TILE_SIZE - yi;
				 
				 newheading = getHeading(dx,dy);
				 final_heading = (newheading + Math.PI * 2 ) % 360;
				 turning_angle = (min_angle(initial_heading, final_heading);
				 if (turning_angle < HEADING_THRESHOLD) {
					 turnto(turning_angle);
				 }
				 if(obstacleDetected()) {
					 //TODO add wallfollower, return control after coast is clear
				 }
				 //TODO convert angle for newheading

				 leftMotor.setSpeed(FORWARD_SPEED);
				 rightMotor.setSpeed(FORWARD_SPEED);
			 }
			 while(Math.abs(dx)< RANGE_THRESHOLD && Math.abs(dy)< RANGE_THRESHOLD );
				 leftMotor.stop();
				 rightMotor.stop();
				 return true;
	  }
	  
	  private static boolean obstacleDetected() {
		// TODO Auto-generated method stub
		return false;
	}
	public static void turnto(double theta) {
		   double absTheta = Math.abs(theta);
		  if (theta >0) {
		      leftMotor.rotate(convertAngle(leftRadius, track, absTheta), true);
		      rightMotor.rotate(-convertAngle(rightRadius, track, absTheta), false);
		  }
		  else {  
			  leftMotor.rotate(-convertAngle(leftRadius, track, absTheta), false);
			  rightMotor.rotate(convertAngle(rightRadius, track, absTheta), true);
		  }
	  }
	private static double getHeading(double dx, double dy) {
		double angle;
		if(dy > 0 ) {
			angle = (Math.atan(dx/dy) + Math.PI*2) % 6;
		}
		else {
			angle = (Math.atan(dx/dy) + Math.PI);
		}
		return angle;
		
	}
	  private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
		  }
	  private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }
	  private static double min_angle(double ihead, double fhead) {
		  double theta;
		  theta = (fhead + Math.PI*2 - ihead) % (2*Math.PI);
		  if(theta < Math.PI) return theta;
		  else return theta-Math.PI*2;
	  }
}
