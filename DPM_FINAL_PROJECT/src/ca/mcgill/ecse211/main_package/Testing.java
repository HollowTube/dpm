package ca.mcgill.ecse211.main_package;

import lejos.hardware.Button;

/**
 * This class is used for testing purposes. It is often modified to the appropriate test.
 * 
 * @author Tritin
 *
 */
public class Testing {
	static MotorControl motorcontrol = MotorControl.getMotor();
	
	/**
	 * Method to perform a certain task for testing.
	 */
	public static void straightLine() {
		while(true) {
			Button.waitForAnyPress();
			motorcontrol.moveSetDistance(120);
		}
	}
}
