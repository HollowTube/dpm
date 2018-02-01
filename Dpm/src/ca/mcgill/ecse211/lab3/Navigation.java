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
	    this.leftRadius = leftRadius;
	    this.rightRadius = rightRadius;
	    this.track = track;
	  }
	  public static boolean travelTo(double xf, double yf) throws OdometerExceptions {

			double xi,yi,dx,dy;
			
			
			 position = odometer.getXYT();
			 xi = position[0];
			 yi = position[1];
			 dx = xf-xi;
			 dy = yf-yi;
			 newheading = Math.atan((dx)/(dy));
			 //TODO convert angle for newheading
			 
			 turnto(newheading);
			 //TODO move robot forward
			 leftMotor.setSpeed(FORWARD_SPEED);
			 rightMotor.setSpeed(FORWARD_SPEED);
			 if (Math.abs(dx)< RANGE_THRESHOLD && Math.abs(dy)< RANGE_THRESHOLD ) {
				 leftMotor.stop();
				 rightMotor.stop();
				 return true;
			 }
			 return false;
	  }
	  
	  public static void turnto(double theta) {
		   double absTheta = Math.abs(theta);
		  if (theta >0) {
		      leftMotor.rotate(convertAngle(leftRadius, track, absTheta), true);
		      rightMotor.rotate(-convertAngle(rightRadius, track, absTheta), false);
		  }
		  else {  
	      leftMotor.rotate(-convertAngle(leftRadius, track, absTheta), true);
	      rightMotor.rotate(convertAngle(rightRadius, track, absTheta), false);
		  }
	  }
		  
	  
	  
	  
	  
	  

	  private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
		  }
	  private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }
}
