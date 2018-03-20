package ca.mcgill.ecse211.main_package;

import lejos.hardware.Button;

public class Testing {
	static MotorControl motorcontrol = MotorControl.getMotor();
	
	public static void straightLine() {
		while(true) {
			Button.waitForAnyPress();
			motorcontrol.moveSetDistance(60);
			motorcontrol.stop();
		}
	}
}
