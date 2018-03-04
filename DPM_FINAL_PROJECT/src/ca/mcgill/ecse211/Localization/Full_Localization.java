package ca.mcgill.ecse211.Localization;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.main_package.Angle_Localization;
import ca.mcgill.ecse211.main_package.LightPoller;
import ca.mcgill.ecse211.main_package.MotorControl;


public class Full_Localization {
	private Odometer odo;
	private Nav nav;
	private EV3UltrasonicSensor us;
	private MotorControl motorcontrol;
	private LightPoller left_LS;
	private LightPoller right_LS;
	private float LS_offset;
	
	public Full_Localization(Odometer odo, Nav nav, EV3UltrasonicSensor us, MotorControl motorcontrol, LightPoller left_LS, LightPoller right_LS){
		this.odo=odo;
		this.nav=nav;
		this.us= us;
		this.motorcontrol = motorcontrol;
		this.left_LS=left_LS;
		this.right_LS=right_LS;
	}
	/*
	 * Method calls methods that do the parts of the full localization
	 * in the order in which they must be performed
	 */
	public void Corner_Localize(){
		UltrasonicLocalizer USL = new UltrasonicLocalizer(odo, nav, us, 2, motorcontrol);
		USL.Localize(); //performs Ultrasonic localization
		try{
			Angle_Localization LSL = new Angle_Localization(left_LS, right_LS);
			//performs light sensor localization
			motorcontrol.forward(); // set the robot in motion towards y = 0 line
			LSL.fix_angle(); // stop at y = 0 line
			odo.setY(-LS_offset); //set odometer y position
			nav.turnTo(90);  //turn towards x = 0 line
			motorcontrol.forward(); //set the robot in motion towards x = 0 line
			LSL.fix_angle(); //stop at x = 0 line
			odo.setX(-LS_offset); //set odometer XS position
			motorcontrol.turnCCW(); //turns toward 0 degrees on point (0,0)
			LSL.fix_angle(); //stops when at 0 degrees
		}catch(Exception e){}
	}

}
