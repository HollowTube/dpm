package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 300;
  private static final int FILTER_OUT = 20;
  private static final int MAXDIST = 80;
  private static final double PCONSTFAR = 15;
  private static final double PCONSTCLOSE = 5;
  private static final int REVERSECAP = 200;
		  
  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl = 0;
  private double error;
  private int correction;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	 int leftspeed;
	 int rightspeed;

	// rudimentary filter - toss out invalid samples corresponding to null
	    // signal.
	    // (n.b. this was not included in the Bang-bang controller, but easily
	    // could have).
	    //
	    if (distance >= MAXDIST && filterControl < FILTER_OUT) {
	      // bad value, do not set the distance var, however do increment the
	      // filter value
	      this.distance = this.bandCenter + 6; //ever so slight nudging towards wall to account for huge spikes
	      filterControl++;
	    } else if (distance >= MAXDIST) {
	      // We have repeated large values, so there must actually be nothing
	      // there: leave the distance alone
	      this.distance = 250;
	    } else {
	      // distance went below 255: reset filter and leave
	      // distance alone.
	      filterControl = 0;
	      this.distance = distance;
	    }	
    error= this.bandCenter - this.distance ; //error calculation
    correction  = Math.abs((int) (PCONSTFAR * error)); // correction calculation
    
    if (correction > REVERSECAP) { // setting an upper bound on possible correction
    	correction =  REVERSECAP;
    }
    
    if(Math.abs(error) < bandWidth) {// do nothing if within bandwidth
    	 WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); 
    	 WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    }
    else if (error < 0 ) {//away from wall, turn towards it

    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - correction); 
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + correction);
    }
    else if ( error > 0) {//close to wall, turn away from it

    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + correction); 
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - correction);
    }
  }
  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
