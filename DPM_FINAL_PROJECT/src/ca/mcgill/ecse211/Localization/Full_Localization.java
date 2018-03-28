package ca.mcgill.ecse211.Localization;

import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.main_package.LightPoller;
import ca.mcgill.ecse211.main_package.MotorControl;

/**
 * This class is in charge of performing the full localization using the Ultrasonic sensor first
 * to get an initial orientation idea and then the lightsensor afterwards to perfectly correct the values.
 * 
 * @author Alexandre
 *
 */
public class Full_Localization {
	private Odometer odo;
	private SampleProvider us;
	private MotorControl motorcontrol;
	private LightPoller left_LS;
	private LightPoller right_LS;
	private float LS_offset=3;
	private double TILE_SIZE = 30.48; 
	private UltrasonicLocalizer USL;
	private Angle_Localization LSL;
	
	/**
	 * Class constructor
	 * @param us Sample from Ultrasonic Sensor
	 * @param motorcontrol Control motors
	 * @param left_LS Left Light sensor
	 * @param right_LS Right Light sensor
	 * @throws OdometerExceptions
	 * 
	 * @author Alexandre Coulombe
	 */
	public Full_Localization(SampleProvider us, MotorControl motorcontrol, LightPoller left_LS, LightPoller right_LS) throws OdometerExceptions{
		this.odo = Odometer.getOdometer();
		this.us= us;
		this.motorcontrol = motorcontrol;
		this.left_LS=left_LS;
		this.right_LS=right_LS;
		this.USL = new UltrasonicLocalizer(odo, us, 2, motorcontrol);
		this.LSL = new Angle_Localization(left_LS, right_LS);
	}
	/**
	 * This method calls the useful methods from other classes in the package 
	 * that do the parts of the full localization in the order in which they must be performed.
	 * 
	 * The method first does the ultrasonic localization, followed by a light localization
	 * on the tile on which it is on and goes to the front right coordinate point of the grid lines.
	 * 
	 * @author Alexandre Coulombe
	 * @param  exp_x	expected x coordinate at the end of localization
	 * @param  exp_y	expected y coordinate at the end of localization
	 * @throws OdometerExceptions
	 */
	public void Corner_Localize(int exp_x, int exp_y) throws OdometerExceptions{
		try{
			Thread.sleep(1000); //sleep thread to give ultrasonic localizer time to instantiate
		} catch (Exception e){}
		USL.Localize(); //performs Ultrasonic localization
		Tile_Localize(exp_x, exp_y);
	}
	
	/**
	 * 	Method to make the robot localize to go on the corner of a tile. 
	 *  The robot moves forward until it arrives to a black line where it corrects the odometer
	 * 	to the corresponding distance on the field from the origin for one coordinate parameter (x or y).
	 *  It will repeat this process with the other parameter.
	 *  <p>
	 *  The robot will backup with the light sensor correcting its heading
	 *  and finish on the coordinate point with the offset of the light sensors.
	 *  
	 *  @param  exp_x	expected x coordinate at the end of localization
	 *  @param  exp_y	expected y coordinate at the end of localization
	 *  @throws OdometerExceptions
	 *  
	 *  @author Alexandre Coulombe
	 */
	public void Tile_Localize(int exp_x, int exp_y) throws OdometerExceptions{
		
		motorcontrol.setLeftSpeed(200);	//performs light sensor localization
		motorcontrol.setRightSpeed(200);
		motorcontrol.forward(); 	// set the robot in motion towards y = 0 line
		LSL.fix_angle(); 			// stop at y = 0 line
		//parameter_correction(exp_x,exp_y);
		
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		
		motorcontrol.moveSetDistance(LS_offset);
		
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		
		motorcontrol.dimeTurn(90);  			//turn towards x = 0 line
		
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		
		motorcontrol.setLeftSpeed(200);
		motorcontrol.setRightSpeed(200);
		motorcontrol.forward(); 	//set the robot in motion towards x = 0 line
		LSL.fix_angle(); 			//stop at x = 0 line
		motorcontrol.moveSetDistance(LS_offset);
		
		try{
			Thread.sleep(200);
		}catch (Exception e){}
		
		motorcontrol.dimeTurn(-90);
		motorcontrol.stop();
		motorcontrol.backward();
		LSL.fix_angle();
		motorcontrol.stop();
		motorcontrol.moveSetDistance(LS_offset);
//		parameter_correction(exp_x,exp_y);
		//odo.setX(exp_x*TILE_SIZE);
		//odo.setY(exp_y*TILE_SIZE);
	}
	
	/**
	 *  Method to change the odometer value for x or y.
	 * 	It corrects the odometer by taking the coordinate distance from the origin
	 * 	added or subtracted depending on the heading.
	 * 
	 *  @param  exp_x	expected x coordinate at the end of localization
	 *  @param  exp_y	expected y coordinate at the end of localization
	 *  @throws OdometerExceptions
	 *  @author Alexandre Coulombe
	 */
	public void parameter_correction(int exp_x, int exp_y){
		double heading = odo.getXYT()[2];
		if (heading > 315 || heading < 45) {
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
