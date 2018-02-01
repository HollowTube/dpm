/*
 * SquareDriver.java
 */
package ca.mcgill.ecse211.lab3;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
/**
 * This class is used to drive the robot on the demo floor.
 */
public class SquareDriver {
  private static final int FORWARD_SPEED = 200;
  private static final int ROTATE_SPEED = 75;
  private static final double TILE_SIZE = 30.48;
  private static Odometer odometer;

  /**
   * This method is meant to drive the robot in a square of size 2x2 Tiles. It is to run in parallel
   * with the odometer and Odometer correcton classes allow testing their functionality.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param width
 * @throws OdometerExceptions 
   */
  public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      double leftRadius, double rightRadius, double track) throws OdometerExceptions {
    // reset the motors
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(1000);
    }
    odometer  = Odometer.getOdometer();
while(true) { // all inside an infinite loop for easier testing
	
    // Sleep for 2 seconds
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
    while (Button.waitForAnyPress() != Button.ID_UP) { //waits until the up button is pressed before its starts
    	  try {
    	      Thread.sleep(500);
    	    } catch (InterruptedException e) {
    	      // There is nothing to be done here
    	    }
    }
    odometer.setXYT(0, 0, 0); // reset display before every run
for (int k = 0; k<4; k++) {
    for (int i = 0; i < 1; i++) {
      // drive forward two tiles
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);

      leftMotor.rotate(convertDistance(leftRadius, 3 * TILE_SIZE), true);
      rightMotor.rotate(convertDistance(rightRadius, 3 * TILE_SIZE), false);

      // turn 90 degrees clockwise
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);

      leftMotor.rotate(convertAngle(leftRadius, track, 90.0), true);
      rightMotor.rotate(-convertAngle(rightRadius, track, 90.0), false);
    }
  }
}
 }
  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
