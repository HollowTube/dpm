package ca.mcgill.ecse211.Localization;

import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.main_package.LightPoller;
import ca.mcgill.ecse211.main_package.MotorControl;


public class Full_Localization {
	private Odometer odo;
	private Nav nav;
	private SampleProvider us;
	private MotorControl motorcontrol;
	private LightPoller left_LS;
	private LightPoller right_LS;
	private float LS_offset=3;
	
	public Full_Localization(Odometer odo, Nav nav, SampleProvider us, MotorControl motorcontrol, LightPoller left_LS, LightPoller right_LS){
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
	public void Corner_Localize() throws OdometerExceptions{
		final UltrasonicLocalizer USL = new UltrasonicLocalizer(odo, nav, us, 1, motorcontrol);
		try{
			Thread.sleep(1000); //sleep thread to give ultrasonic localizer time to instantiate
		} catch (Exception e){}
		USL.Localize(); //performs Ultrasonic localization
		final Angle_Localization LSL = new Angle_Localization(left_LS, right_LS);
			//performs light sensor localization
		motorcontrol.setLeftSpeed(200);
		motorcontrol.setRightSpeed(200);
		motorcontrol.forward(); 	// set the robot in motion towards y = 0 line
		LSL.fix_angle(); 			// stop at y = 0 line
		odo.setY(-LS_offset); 		//set odometer y position
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.forward(LS_offset);
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		nav.turnTo(90);  			//turn towards x = 0 line
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.setLeftSpeed(200);
		motorcontrol.setRightSpeed(200);
		motorcontrol.forward(); 	//set the robot in motion towards x = 0 line
		LSL.fix_angle(); 			//stop at x = 0 line
		odo.setX(-LS_offset); 		//set odometer XS position
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.forward(LS_offset);
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		nav.turnTo(-90);  			//turn towards x = 0 line
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.backward();
		LSL.fix_angle(); 			//stops when at 0 degrees
		odo.setY(-LS_offset);
		odo.setTheta(0);
	}
}
