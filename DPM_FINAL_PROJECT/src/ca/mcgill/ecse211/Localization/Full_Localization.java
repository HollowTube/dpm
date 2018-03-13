package ca.mcgill.ecse211.Localization;

import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.main_package.LightPoller;
import ca.mcgill.ecse211.main_package.MotorControl;


public class Full_Localization {
	private Odometer odo;
	private SampleProvider us;
	private MotorControl motorcontrol;
	private LightPoller left_LS;
	private LightPoller right_LS;
	private float LS_offset=3;
	private double TILE_SIZE = 30.48;
	
	public Full_Localization(Odometer odo, SampleProvider us, MotorControl motorcontrol, LightPoller left_LS, LightPoller right_LS){
		this.odo=odo;
		this.us= us;
		this.motorcontrol = motorcontrol;
		this.left_LS=left_LS;
		this.right_LS=right_LS;
	}
	/* 	public void Corner_Localize(int exp_x, int exp_y) throws OdometerExceptions
	 * Method calls methods that do the parts of the full localization
	 * in the order in which they must be performed.
	 * The method first does the ultrasonic localization, followed by a light localization
	 * on the tile on which it is on and goes to the front right coordinate point of the grid lines
	 */
	public void Corner_Localize(int exp_x, int exp_y) throws OdometerExceptions{
		final UltrasonicLocalizer USL = new UltrasonicLocalizer(odo, us, 1, motorcontrol);
		try{
			Thread.sleep(1000); //sleep thread to give ultrasonic localizer time to instantiate
		} catch (Exception e){}
		USL.Localize(); //performs Ultrasonic localization
		Tile_Localize(exp_x, exp_y);
	}
	
	/*	public void Tile_Localize(int exp_x, int exp_y) throws OdometerExceptions
	 * 	Make the robot move forward until it arrives to a black line where it corrects the odometer
	 * 	to the corresponding distance on the field from the origin for one coordinate parameter (x or y).
	 *  It will repeat with the other parameter. The robot will backup with the light sensor correcting its heading
	 *  and finish on the coordinate point with the offset of the light sensors.
	 */
	public void Tile_Localize(int exp_x, int exp_y) throws OdometerExceptions{
		final Angle_Localization LSL = new Angle_Localization(left_LS, right_LS);
			//performs light sensor localization
		motorcontrol.setLeftSpeed(200);
		motorcontrol.setRightSpeed(200);
		motorcontrol.forward(); 	// set the robot in motion towards y = 0 line
		LSL.fix_angle(); 			// stop at y = 0 line
		parameter_correction(exp_x,exp_y);
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.moveSetDistance(LS_offset);
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.turnto(90);  			//turn towards x = 0 line
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.setLeftSpeed(200);
		motorcontrol.setRightSpeed(200);
		motorcontrol.forward(); 	//set the robot in motion towards x = 0 line
		LSL.fix_angle(); 			//stop at x = 0 line
		parameter_correction(exp_x,exp_y);
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.moveSetDistance(LS_offset);
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.turnto(-90);  			//turn towards x = 0 line
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		motorcontrol.backward();
		LSL.fix_angle(); 			//stops when at 0 degrees
		parameter_correction(exp_x,exp_y);
	}
	
	/*	public void parameter_correction(int exp_x, int exp_y)
	 * 	corrects the odometer by taking the coordinate distance from the origin
	 * 	added or subtracted depending on the heading
	 */
	public void parameter_correction(int exp_x, int exp_y){
		double heading = odo.getXYT()[2];
		if (heading > 315 && heading < 45) {
			odo.setY(exp_y*TILE_SIZE-LS_offset); 			//set odometer y position
		} else if (heading > 45 && heading < 135) {
			odo.setX(exp_x*TILE_SIZE-LS_offset); 			//set odometer x position
		} else if (heading > 135 && heading < 225) {
			odo.setY(exp_y*TILE_SIZE+LS_offset); 			//set odometer y position
		} else if (heading > 225 && heading < 315) {
			odo.setX(exp_x*TILE_SIZE+LS_offset); 			//set odometer x position
		}
	}
}
