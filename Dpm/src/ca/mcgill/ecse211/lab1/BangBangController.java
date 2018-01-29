package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandWidth;
  private final int motorLow;
  private final int motorHigh;
  private final int MOTOR_SPEED = 300;
  private final int deltaSpeed = 225;
  private static final int MAXDIST = 100;
  private static final int FILTER_OUT = 18;
  private int distance;
  
  int correction;
  int leftspeed;
  int rightspeed;
  int filterControl = 0;

  public BangBangController(int bandCenter, int bandWidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandWidth = bandWidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	int error;
    
    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= MAXDIST && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      this.distance = this.bandCenter; //ever so slight nudging towards wall to account for huge spikes
      filterControl++;
    } else if (distance >= MAXDIST) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }	
    
    error= this.bandCenter - this.distance ;
    if(Math.abs(error) < bandWidth) {// do nothing if within bandwidth
   	 WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); 
   	 WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
     WallFollowingLab.leftMotor.forward();
     WallFollowingLab.rightMotor.forward();
   }
    
    if (error < 0 ) {//away from wall, turn towards it
    // right wheel gets more speed while left stays the same

   	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); 
   	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + deltaSpeed);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
   }
    
   else if ( error > 0) {//close to wall, turn away from it
    // left wheel gets more speed while right stays the same
   	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + deltaSpeed); 
   	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
   }
    
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
